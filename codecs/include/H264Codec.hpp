// Copyright 2019 (c) Christopher A. Taylor.  All rights reserved.

/*
    Stand-alone library that wraps different methods to encode/decode video.
    Meant to be tweaked and modified for each application, not as a fully
    general video codec library.

    On Intel Windows the best way is to use Nvidia's CUDA nvcuvid library,
    and maybe MediaFoundation if that is not available.
    On Intel Linux the best way is to use ffmpeg's vaapi plugin.
    On Android/iOS there are OS-specific APIs around some pretty unreliable hw.
    Other platforms mostly use V4L2.

    Currently only CUDA and x264 are implemented, but it is designed to make it
    easier to add more hardware-accelerated backends.
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

enum class VideoBackend
{
    Uninitialized,
    Software,
    Cuda,
};

class VideoCodec
{
public:
    bool Encode(
        int width,
        int height,
        std::vector<uint8_t>& data,
        std::vector<uint8_t>& compressed);

    bool Decode(
        int width,
        int height,
        const uint8_t* data,
        int bytes,
        std::vector<uint8_t>& decoded);

protected:
    int Width = 0, Height = 0;

    VideoBackend EncoderBackend = VideoBackend::Uninitialized;
    VideoBackend DecoderBackend = VideoBackend::Uninitialized;

    // CUDA NVENC/NVDEC
    bool CudaNonfunctional = false;
    CudaContext Context;
    std::shared_ptr<NvEncoderCuda> CudaEncoder;
    std::shared_ptr<NvDecoder> CudaDecoder;


    bool EncodeNvenc(
        std::vector<uint8_t>& data,
        std::vector<uint8_t>& compressed);
    bool DecodeNvdec(
        const uint8_t* data,
        int bytes,
        std::vector<uint8_t>& decoded);
    void CleanupCuda();
};


} // namespace zdepth
