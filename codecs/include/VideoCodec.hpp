// Copyright 2019 (c) Christopher A. Taylor.  All rights reserved.

/*
    Stand-alone library that wraps different methods to encode/decode video.
    Meant to be tweaked and modified for each application, not as a fully
    general video codec library.

    On Intel Windows the best way is to use Nvidia's CUDA nvcuvid library,
    and maybe MediaFoundation for Intel Quick Sync Video (QSV).
    On Intel Linux the best way is to use ffmpeg's vaapi plugin for QSV.
    On Android/iOS there are OS-specific APIs around some pretty unreliable hw.
    Other platforms mostly use V4L2.

    Currently only CUDA is implemented, but it is designed to make it
    easier to add more hardware-accelerated backends.

    Note that most hardware encoders are limited to one/two sessions at a time,
    so it is often not desired to make more than one encoder instance.
*/

/*
    Why not use NvPipe?
    https://github.com/NVIDIA/NvPipe

    While NvPipe does seem to support 16-bit monochrome data, the manner
    in which it does this is not recommended: The high and low bytes are
    split into halves of the Y channel of an image, doubling the resolution.
    So the video encoder runs twice as slow.  Single bit errors in the Y channel
    are then magnified in the resulting decoded values by 256x, which is not
    acceptable for depth data because this is basically unusable.

    Other features of NvPipe are not useful for depth compression, and it
    abstracts away the more powerful nvcuvid API that allows applications to
    dispatch multiple encodes in parallel in a scatter-gather pattern, and to
    tune the encoder parameters like intra-refresh, AQ, and so on.
*/

#pragma once

// Nvidia NVENC/NVDEC uses the attached GPU for efficient encoding via CUDA
#include <cuda_runtime_api.h>
#include <NvEncoder.h>
#include <NvEncoderCuda.h>
#include <NvDecoder.h>

#include <stdint.h>
#include <memory>
#include <vector>

namespace zdepth {


//------------------------------------------------------------------------------
// CUDA Context

struct CudaContext
{
    ~CudaContext()
    {
        Destroy();
    }

    bool Valid() const
    {
        return Context != nullptr;
    }

    CUcontext Context = nullptr;

    CUdevice Device = 0;
    cudaDeviceProp Properties{};
    int GpuIndex = 0;


    // Create the context
    bool Create(int gpu_index = 0);
    void Destroy();
};


//------------------------------------------------------------------------------
// Video Codec

enum class VideoType
{
    // NVENC only supports these two
    H264,
    H265,
};

struct VideoParameters
{
    // Using H265 instead here leads to files with half the error that are about
    // 25% smaller.  But the encoder/decoder is not always available.
    VideoType Type = VideoType::H264;

    // Depth image resolution
    int Width = 0, Height = 0;

    // Frames per second of camera
    int Fps = 30;
};

enum class VideoBackend
{
    Uninitialized,
    Software,
    Cuda,
};

class VideoCodec
{
public:
    bool EncodeBegin(
        const VideoParameters& params,
        bool keyframe,
        const std::vector<uint8_t>& data,
        std::vector<uint8_t>& compressed);
    bool EncodeFinish(
        std::vector<uint8_t>& compressed);

    bool Decode(
        int width,
        int height,
        VideoType type,
        const uint8_t* data,
        int bytes,
        std::vector<uint8_t>& decoded);

protected:
    VideoParameters Params{};

    VideoBackend EncoderBackend = VideoBackend::Uninitialized;
    VideoBackend DecoderBackend = VideoBackend::Uninitialized;

    // Shared state
    uint64_t NextTimestamp = 0;
    std::vector<std::vector<uint8_t>> VideoTemp;

    // CUDA NVENC/NVDEC
    GUID CodecGuid;
    bool CudaNonfunctional = false;
    CudaContext Context;
    std::shared_ptr<NvEncoderCuda> CudaEncoder;
    std::shared_ptr<NvDecoder> CudaDecoder;


    bool EncodeBeginNvenc(
        bool keyframe,
        const std::vector<uint8_t>& data,
        std::vector<uint8_t>& compressed);
    bool EncodeFinishNvenc(
        std::vector<uint8_t>& compressed);
    bool DecodeNvdec(
        const uint8_t* data,
        int bytes,
        std::vector<uint8_t>& decoded);
    void CleanupCuda();
};


} // namespace zdepth
