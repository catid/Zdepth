// Copyright 2019 (c) Christopher A. Taylor.  All rights reserved.

#include "VideoCodec.hpp"

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

bool VideoCodec::EncodeBegin(
    const VideoParameters& params,
    bool keyframe,
    const std::vector<uint8_t>& data,
    std::vector<uint8_t>& compressed)
{
    // If resolution changed:
    if (params.Width != Params.Width || params.Height != Params.Height) {
        CleanupCuda();
        EncoderBackend = VideoBackend::Uninitialized;
        DecoderBackend = VideoBackend::Uninitialized;
    }
    Params = params;

    return EncodeBeginNvenc(keyframe, data, compressed);
}

bool VideoCodec::EncodeFinish(
    std::vector<uint8_t>& compressed)
{
    return EncodeFinishNvenc(compressed);
}

bool VideoCodec::Decode(
    int width,
    int height,
    VideoType type,
    const uint8_t* data,
    int bytes,
    std::vector<uint8_t>& decoded)
{
    // If resolution changed:
    if (width != Params.Width || height != Params.Height) {
        CleanupCuda();
        EncoderBackend = VideoBackend::Uninitialized;
        DecoderBackend = VideoBackend::Uninitialized;
    }
    Params.Width = width;
    Params.Height = height;
    Params.Type = type;

    return DecodeNvdec(data, bytes, decoded);
}


//------------------------------------------------------------------------------
// Video Codec : CUDA Backend

bool VideoCodec::EncodeBeginNvenc(
    bool keyframe,
    const std::vector<uint8_t>& data,
    std::vector<uint8_t>& compressed)
{
    try {
        if (!CudaEncoder) {
            if (!Context.Create()) {
                return false;
            }

            CodecGuid = (Params.Type == VideoType::H264) ? NV_ENC_CODEC_H264_GUID : NV_ENC_CODEC_HEVC_GUID;

            CudaEncoder = std::make_shared<NvEncoderCuda>(
                Context.Context,
                Params.Width,
                Params.Height,
                NV_ENC_BUFFER_FORMAT_NV12);

            NV_ENC_INITIALIZE_PARAMS encodeParams = { NV_ENC_INITIALIZE_PARAMS_VER };
            NV_ENC_CONFIG encodeConfig = { NV_ENC_CONFIG_VER };
            encodeParams.encodeConfig = &encodeConfig;

            CudaEncoder->CreateDefaultEncoderParams(
                &encodeParams,
                CodecGuid,
                NV_ENC_PRESET_LOW_LATENCY_HQ_GUID);

            encodeParams.frameRateNum = Params.Fps;
            encodeParams.frameRateDen = 1;
            encodeParams.enablePTD = 1; // Allow NVENC to choose picture types

            const bool supports_intra_refresh = CudaEncoder->GetCapabilityValue(
                CodecGuid,
                NV_ENC_CAPS_SUPPORT_INTRA_REFRESH);

            // Enable intra-refresh for a more consistent frame size:
            if (Params.Type == VideoType::H264) {
                auto& h264Config = encodeConfig.encodeCodecConfig.h264Config;
                h264Config.repeatSPSPPS = 0;
                if (supports_intra_refresh) {
                    h264Config.enableIntraRefresh = 1;
                    h264Config.intraRefreshPeriod = Params.Fps * 10;
                    h264Config.intraRefreshCnt = Params.Fps;
                    h264Config.idrPeriod = NVENC_INFINITE_GOPLENGTH;
                }
            } else { // HEVC:
                auto& hevcConfig = encodeConfig.encodeCodecConfig.hevcConfig;
                hevcConfig.repeatSPSPPS = 0;
                if (supports_intra_refresh) {
                    hevcConfig.enableIntraRefresh = 1;
                    hevcConfig.intraRefreshPeriod = Params.Fps * 10;
                    hevcConfig.intraRefreshCnt = Params.Fps;
                    hevcConfig.idrPeriod = NVENC_INFINITE_GOPLENGTH;
                }
            }

            // Manual IDRs when application requests a keyframe
            encodeConfig.gopLength = NVENC_INFINITE_GOPLENGTH;
            encodeConfig.frameIntervalP = 1;

            // How to pick encoder mode:
            // https://slhck.info/video/2017/03/01/rate-control.html
            // Our goal is to have constant quality and low latency for streaming.

            if (Params.Bitrate != 0)
            {
                // Choose VBR mode allowing for spikes for tricky frames
                // NV_ENC_PARAMS_RC_CBR_LOWDELAY_HQ: Error bound is smaller
                // NV_ENC_PARAMS_RC_CBR_HQ: Seems to have a longer tail of errors
                // NV_ENC_PARAMS_RC_VBR_HQ: Also long error tail
                encodeConfig.rcParams.rateControlMode = NV_ENC_PARAMS_RC_CBR_LOWDELAY_HQ;
                encodeConfig.rcParams.averageBitRate = Params.Bitrate;
                encodeConfig.rcParams.maxBitRate = Params.Bitrate;

                // Tune VBV size 
                encodeConfig.rcParams.vbvBufferSize = Params.Bitrate / Params.Fps;
                encodeConfig.rcParams.vbvInitialDelay = encodeConfig.rcParams.vbvBufferSize;
            }
            else
            {
                // For constraining the quality loss: NV_ENC_PARAMS_RC_CONSTQP
                encodeConfig.rcParams.rateControlMode = NV_ENC_PARAMS_RC_CONSTQP;
            }

            // Disable adaptive quantization for this type of data.
            // It leads to much higher long tail errors.
            encodeConfig.rcParams.enableTemporalAQ = 0;
            encodeConfig.rcParams.enableAQ = 0; // Spatial
            encodeConfig.rcParams.aqStrength = 1; // Lower is better

            // Disable B-frames
            encodeConfig.rcParams.zeroReorderDelay = 1;

            // Enable non-reference P-frame optimization
            encodeConfig.rcParams.enableNonRefP = 1; // requires enablePTD=1

            CudaEncoder->CreateEncoder(&encodeParams);
        }

        const NvEncInputFrame* frame = CudaEncoder->GetNextInputFrame();

        // If no frames available:
        if (!frame) {
            return false;
        }

        NvEncoderCuda::CopyToDeviceFrame(
            Context.Context,
            const_cast<uint8_t*>( data.data() ),
            0,
            (CUdeviceptr)frame->inputPtr,
            (int)frame->pitch,
            Params.Width,
            Params.Height, 
            CU_MEMORYTYPE_HOST, 
            frame->bufferFormat,
            frame->chromaOffsets,
            frame->numChromaPlanes);

        // The other parameters are filled in by NvEncoder::DoEncode
        NV_ENC_PIC_PARAMS pic_params = { NV_ENC_PIC_PARAMS_VER };
        pic_params.inputPitch = frame->pitch;

        if (keyframe) {
            // Force an IDR and prepend SPS, PPS units
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
        CudaEncoder->EncodeFrame(VideoTemp, &pic_params);

        compressed.clear();
        size_t size = 0;
        for (auto& unit : VideoTemp)
        {
            const size_t unit_size = unit.size();
            compressed.resize(size + unit_size);
            memcpy(compressed.data() + size, unit.data(), unit_size);
            size += unit_size;
        }
    }
    catch (NVENCException& /*ex*/) {
        return false;
    }

    return true;
}

bool VideoCodec::EncodeFinishNvenc(
    std::vector<uint8_t>& compressed)
{
    try {
        CudaEncoder->EndEncode(VideoTemp);

        // If encode failed:
        if (VideoTemp.empty()) {
            return false;
        }

        size_t size = compressed.size();
        for (auto& unit : VideoTemp)
        {
            const size_t unit_size = unit.size();
            compressed.resize(size + unit_size);
            memcpy(compressed.data() + size, unit.data(), unit_size);
            size += unit_size;
        }
    }
    catch (NVENCException& /*ex*/) {
        return false;
    }

    return true;
}

bool VideoCodec::DecodeNvdec(
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
                Params.Width,
                Params.Height,
                false, // Do not use device frame
                Params.Type == VideoType::H264 ? cudaVideoCodec_H264 : cudaVideoCodec_HEVC,
                nullptr, // No mutex
                true, // Low latency
                false, // Non-pitched frame
                nullptr, // No crop
                nullptr); // No resize
        }

        uint8_t** frames = nullptr;
        int64_t* timestamps = nullptr;
        int frame_count = 0;

        // Retries are needed according to Nvidia engineers:
        // https://github.com/NVIDIA/NvPipe/blob/b3d0a7511052824ff0481fa6eecb3e95eac1a722/src/NvPipe.cu#L969

        for (int i = 0; i < 3; ++i) {
            bool success = CudaDecoder->Decode(
                data,
                bytes,
                &frames,
                &frame_count,
                CUVID_PKT_ENDOFPICTURE, // Immediate result requested
                &timestamps,
                0, // Timestamp
                cudaStreamPerThread); // Use the default per-thread stream
            if (!success) {
                return false;
            }

            // If we got a frame back:
            if (frame_count >= 1) {
                break;
            }
        }

        if (frame_count < 1) {
            return false;
        }

        const int y_bytes = Params.Width * Params.Height;
        decoded.resize(y_bytes);
        memcpy(decoded.data(), frames[0], y_bytes);
    }
    catch (NVENCException& /*ex*/) {
        return false;
    }

    return true;
}

void VideoCodec::CleanupCuda()
{
    CudaEncoder.reset();
    CudaDecoder.reset();
    Context.Destroy();
}


} // namespace zdepth
