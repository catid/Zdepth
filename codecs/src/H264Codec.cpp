// Copyright 2019 (c) Christopher A. Taylor.  All rights reserved.

#include "H264Codec.hpp"

namespace zdepth {


//------------------------------------------------------------------------------
// CUDA Context

bool CudaContext::Create(int gpu_index)
{
    CUresult result;

    GpuIndex = gpu_index;

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

bool VideoCodec::Encode(
    int width,
    int height,
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

    if (EncoderBackend == VideoBackend::Uninitialized) {
        // Try CUDA first
        bool success = EncodeNvenc(data, compressed);
        if (success) {
            EncoderBackend = VideoBackend::Cuda;
        }
    }
}

bool VideoCodec::Decode(
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

}


//------------------------------------------------------------------------------
// Video Codec : CUDA Backend

bool VideoCodec::EncodeNvenc(
    std::vector<uint8_t>& data,
    std::vector<uint8_t>& compressed)
{
    try {
        if (!Encoder) {
            Encoder = std::make_shared<NvEncoderCuda>(
                cuContext,
                width,
                height,
                NV_ENC_BUFFER_FORMAT_NV12);

            NV_ENC_INITIALIZE_PARAMS initializeParams = { NV_ENC_INITIALIZE_PARAMS_VER };
            NV_ENC_CONFIG encodeConfig = { NV_ENC_CONFIG_VER };
            initializeParams.encodeConfig = &encodeConfig;

            Encoder->CreateDefaultEncoderParams(
                &initializeParams,
                NV_ENC_CODEC_H264_GUID,
                NV_ENC_PRESET_LOW_LATENCY_HQ_GUID);
            // NV_ENC_PRESET_DEFAULT_GUID
            // NV_ENC_PRESET_LOW_LATENCY_HP_GUID

            Encoder->CreateEncoder(&initializeParams);
            CudaInitialized = true;
        }

        const NvEncInputFrame* encoderInputFrame = Encoder->GetNextInputFrame();

        // If no frames available:
        if (!encoderInputFrame) {
            return false;
        }

        NvEncoderCuda::CopyToDeviceFrame(
            cuContext,
            lows.data(),
            0,
            (CUdeviceptr)encoderInputFrame->inputPtr,
            (int)encoderInputFrame->pitch,
            width,
            height, 
            CU_MEMORYTYPE_HOST, 
            encoderInputFrame->bufferFormat,
            encoderInputFrame->chromaOffsets,
            encoderInputFrame->numChromaPlanes);

        // Encode frame and wait for the result.
        // This takes under a millisecond on modern gaming laptops.
        std::vector<std::vector<uint8_t>> video1;
        std::vector<std::vector<uint8_t>> video2;
        Encoder->EncodeFrame(video1);
        Encoder->EndEncode(video2);

        // If encode failed:
        if (video1.empty() && video2.empty()) {
            return false;
        }
    }
    catch (NVENCException& ex) {
        return false;
    }

    return true;
}

bool VideoCodec::DecodeNvdec(
    const uint8_t* data,
    int bytes,
    std::vector<uint8_t>& decoded)
{
    try {

    }
    catch (NVENCException& ex) {
        return false;
    }

    return true;
}

void VideoCodec::CleanupCuda()
{

}


} // namespace zdepth
