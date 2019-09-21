// Copyright 2019 (c) Christopher A. Taylor.  All rights reserved.

#include "H264Codec.hpp"

namespace zdepth {


//------------------------------------------------------------------------------
// CUDA Context

bool CudaContext::Create(int gpu_index)
{
    if (Context) {
        return true; // Already created
    }

    CUresult result;

    GpuIndex = gpu_index;

    result = cuInit(0);
    if (result != CUDA_SUCCESS) {
        return false;
    }

    result = cuDeviceGet(&Device, gpu_index);
    if (result != CUDA_SUCCESS) {
        return false;
    }

    cudaError_t err = cudaGetDeviceProperties(&Properties, Device);
    if (err != cudaSuccess) {
        return false;
    }

    // Reuse the primary context to play nicer with application code
    result = cuDevicePrimaryCtxRetain(&Context, Device);
    if (result != CUDA_SUCCESS) {
        return false;
    }

    return true;
}

void CudaContext::Destroy()
{
    if (Context) {
        cuDevicePrimaryCtxRelease(Device);
        Context = nullptr;
    }
}


//------------------------------------------------------------------------------
// Video Codec : API

bool H264Codec::Encode(
    int width,
    int height,
    bool keyframe,
    std::vector<uint8_t>& data,
    std::vector<uint8_t>& compressed)
{
    // If resolution changed:
    if (width != Width || height != Height) {
        CleanupCuda();
        EncoderBackend = VideoBackend::Uninitialized;
        DecoderBackend = VideoBackend::Uninitialized;
        Width = width;
        Height = height;
    }

    return EncodeNvenc(keyframe, data, compressed);
}

bool H264Codec::Decode(
    int width,
    int height,
    const uint8_t* data,
    int bytes,
    std::vector<uint8_t>& decoded)
{
    // If resolution changed:
    if (width != Width || height != Height) {
        CleanupCuda();
        EncoderBackend = VideoBackend::Uninitialized;
        DecoderBackend = VideoBackend::Uninitialized;
        Width = width;
        Height = height;
    }

    return DecodeNvdec(data, bytes, decoded);
}


//------------------------------------------------------------------------------
// Video Codec : CUDA Backend

bool H264Codec::EncodeNvenc(
    bool keyframe,
    std::vector<uint8_t>& data,
    std::vector<uint8_t>& compressed)
{
    try {
        if (!CudaEncoder) {
            if (!Context.Create()) {
                return false;
            }

            CudaEncoder = std::make_shared<NvEncoderCuda>(
                Context.Context,
                Width,
                Height,
                NV_ENC_BUFFER_FORMAT_NV12);

            NV_ENC_INITIALIZE_PARAMS initializeParams = { NV_ENC_INITIALIZE_PARAMS_VER };
            NV_ENC_CONFIG encodeConfig = { NV_ENC_CONFIG_VER };
            initializeParams.encodeConfig = &encodeConfig;

            CudaEncoder->CreateDefaultEncoderParams(
                &initializeParams,
                NV_ENC_CODEC_H264_GUID,
                NV_ENC_PRESET_LOW_LATENCY_HQ_GUID);
            // NV_ENC_PRESET_DEFAULT_GUID
            // NV_ENC_PRESET_LOW_LATENCY_HP_GUID

            CudaEncoder->CreateEncoder(&initializeParams);
        }

        const NvEncInputFrame* frame = CudaEncoder->GetNextInputFrame();

        // If no frames available:
        if (!frame) {
            return false;
        }

        NvEncoderCuda::CopyToDeviceFrame(
            Context.Context,
            data.data(),
            0,
            (CUdeviceptr)frame->inputPtr,
            (int)frame->pitch,
            Width,
            Height, 
            CU_MEMORYTYPE_HOST, 
            frame->bufferFormat,
            frame->chromaOffsets,
            frame->numChromaPlanes);

        // The other parameters are filled in by NvEncoder::DoEncode
        NV_ENC_PIC_PARAMS pic_params = { NV_ENC_PIC_PARAMS_VER };
        pic_params.inputPitch = frame->pitch;

        if (keyframe) {
            pic_params.encodePicFlags |= NV_ENC_PIC_FLAG_OUTPUT_SPSPPS | NV_ENC_PIC_FLAG_FORCEIDR;
            pic_params.pictureType = NV_ENC_PIC_TYPE_IDR;
        } else {
            pic_params.pictureType = NV_ENC_PIC_TYPE_P;
        }

        // pic_params.frameIdx = 0; // Optional
        pic_params.inputTimeStamp = NextTimestamp++;
        // pic_params.inputDuration = 0; // TBD
        // pic_params.codecPicParams.h264PicParams; // No tweaks seem useful
        // pic_params.qpDeltaMap = nullptr; // TBD
        // pic_params.qpDeltaMapSize = 0; // TBD

        // Encode frame and wait for the result.
        // This takes under a millisecond on modern gaming laptops.
        std::vector<std::vector<uint8_t>> video1;
        std::vector<std::vector<uint8_t>> video2;
        CudaEncoder->EncodeFrame(video1, &pic_params);
        CudaEncoder->EndEncode(video2);

        // If encode failed:
        if (video1.empty() && video2.empty()) {
            return false;
        }

        compressed.clear();
        size_t size = 0;
        for (auto& unit : video1)
        {
            const size_t unit_size = unit.size();
            compressed.resize(size + unit_size);
            memcpy(compressed.data() + size, unit.data(), unit_size);
            size += unit_size;
        }
        for (auto& unit : video2)
        {
            const size_t unit_size = unit.size();
            compressed.resize(size + unit_size);
            memcpy(compressed.data() + size, unit.data(), unit_size);
            size += unit_size;
        }
    }
    catch (NVENCException& ex) {
        return false;
    }

    return true;
}

bool H264Codec::DecodeNvdec(
    const uint8_t* data,
    int bytes,
    std::vector<uint8_t>& decoded)
{
    CUresult result;

    try {
        if (!CudaDecoder) {
            if (!Context.Create()) {
                return false;
            }

            CudaDecoder = std::make_shared<NvDecoder>(
                Context.Context,
                Width,
                Height,
                false, // Do not use device frame
                cudaVideoCodec_H264,
                nullptr, // No mutex
                true, // Low latency
                false, // Pitched device frame?
                nullptr, // No crop
                nullptr); // No resize
        }

        uint8_t** frames = nullptr;
        int64_t* timestamps = nullptr;
        int frame_count = 0;

        bool success = CudaDecoder->Decode(
            data,
            bytes,
            &frames,
            &frame_count,
            CUVID_PKT_ENDOFPICTURE,
            &timestamps,
            0, // Timestamp
            cudaStreamPerThread); // Use the default per-thread stream
        if (!success) {
            return false;
        }

        if (frame_count < 1) {
            return false;
        }

        const int y_bytes = Width * Height;
        decoded.resize(y_bytes);
        memcpy(decoded.data(), frames[0], y_bytes);
    }
    catch (NVENCException& ex) {
        return false;
    }

    return true;
}

void H264Codec::CleanupCuda()
{
    CudaEncoder.reset();
    CudaDecoder.reset();
    Context.Destroy();
}


} // namespace zdepth
