// Copyright 2019 (c) Christopher A. Taylor.  All rights reserved.

#include "zdepth.hpp"

#include "libdivide.h"

#include <zstd.h> // Zstd
#include <string.h> // memcpy

#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #include <windows.h>
#elif __MACH__
    #include <sys/file.h>
    #include <mach/mach_time.h>
    #include <mach/mach.h>
    #include <mach/clock.h>

    extern mach_port_t clock_port;
#else
    #include <time.h>
    #include <sys/time.h>
    #include <sys/file.h> // flock
#endif

#include <iostream>
using namespace std;

namespace zdepth {


//------------------------------------------------------------------------------
// Timing

#ifdef _WIN32
// Precomputed frequency inverse
static double PerfFrequencyInverseUsec = 0.;
static double PerfFrequencyInverseMsec = 0.;

static void InitPerfFrequencyInverse()
{
    LARGE_INTEGER freq = {};
    if (!::QueryPerformanceFrequency(&freq) || freq.QuadPart == 0) {
        return;
    }
    const double invFreq = 1. / (double)freq.QuadPart;
    PerfFrequencyInverseUsec = 1000000. * invFreq;
    PerfFrequencyInverseMsec = 1000. * invFreq;
}
#elif __MACH__
static bool m_clock_serv_init = false;
static clock_serv_t m_clock_serv = 0;

static void InitClockServ()
{
    m_clock_serv_init = true;
    host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &m_clock_serv);
}
#endif // _WIN32

uint64_t GetTimeUsec()
{
#ifdef _WIN32
    LARGE_INTEGER timeStamp = {};
    if (!::QueryPerformanceCounter(&timeStamp)) {
        return 0;
    }
    if (PerfFrequencyInverseUsec == 0.) {
        InitPerfFrequencyInverse();
    }
    return (uint64_t)(PerfFrequencyInverseUsec * timeStamp.QuadPart);
#elif __MACH__
    if (!m_clock_serv_init) {
        InitClockServ();
    }

    mach_timespec_t tv;
    clock_get_time(m_clock_serv, &tv);

    return 1000000 * tv.tv_sec + tv.tv_nsec / 1000;
#else
    // This seems to be the best clock to used based on:
    // http://btorpey.github.io/blog/2014/02/18/clock-sources-in-linux/
    // The CLOCK_MONOTONIC_RAW seems to take a long time to query,
    // and so it only seems useful for timing code that runs a small number of times.
    // The CLOCK_MONOTONIC is affected by NTP at 500ppm but doesn't make sudden jumps.
    // Applications should already be robust to clock skew so this is acceptable.
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_nsec / 1000) + static_cast<uint64_t>(ts.tv_sec) * 1000000;
#endif
}


//------------------------------------------------------------------------------
// Constants

// Size of a block for predictor selection purposes
static const int kBlockSize = 8;

// Zstd compression level
static const int kZstdLevel = 1;

const char* DepthResultString(DepthResult result)
{
    switch (result)
    {
    case DepthResult::Success: return "Success";
    case DepthResult::FileTruncated: return "FileTruncated";
    case DepthResult::WrongFormat: return "WrongFormat";
    case DepthResult::Corrupted: return "Corrupted";
    case DepthResult::MissingFrame: return "MissingFrame";
    default: break;
    }
    return "Unknown";
}


//------------------------------------------------------------------------------
// Tools

bool IsDepthFrame(const uint8_t* file_data, unsigned file_bytes)
{
    if (file_bytes < kDepthHeaderBytes) {
        return false;
    }
    if (file_data[0] != kDepthFormatMagic) {
        return false;
    }
    return true;
}

bool IsKeyFrame(const uint8_t* file_data, unsigned file_bytes)
{
    if (!IsDepthFrame(file_data, file_bytes)) {
        return false;
    }
    return (file_data[1] & 1) != 0;
}


//------------------------------------------------------------------------------
// Depth Quantization

uint16_t AzureKinectQuantizeDepth(uint16_t depth)
{
    if (depth <= 200) {
        return 0; // Too close
    }
    if (depth < 750) {
        return depth - 200;
    }
    if (depth < 1500) {
        return 550 + (depth - 750) / 2;
    }
    if (depth < 3000) {
        return 925 + (depth - 1500) / 4;
    }
    if (depth < 6000) {
        return 1300 + (depth - 3000) / 8;
    }
    if (depth < 11840) {
        return 1675 + (depth - 6000) / 16;
    }
    return 0; // Too far
}

uint16_t AzureKinectDequantizeDepth(uint16_t quantized)
{
    if (quantized == 0) {
        return 0;
    }
    if (quantized < 550) {
        return quantized + 200;
    }
    if (quantized < 925) {
        return 750 + (quantized - 550) * 2;
    }
    if (quantized < 1300) {
        return 1500 + (quantized - 925) * 4;
    }
    if (quantized < 1675) {
        return 3000 + (quantized - 1300) * 8;
    }
    if (quantized < 2040) {
        return 6000 + (quantized - 1675) * 16;
    }
    return 0; // Invalid value
}

void QuantizeDepthImage(
    int n,
    const uint16_t* depth,
    std::vector<uint16_t>& quantized)
{
    quantized.resize(n);
    uint16_t* dest = quantized.data();

    for (int i = 0; i < n; ++i) {
        dest[i] = AzureKinectQuantizeDepth(depth[i]);
    }
}

void DequantizeDepthImage(std::vector<uint16_t>& depth_inout)
{
    const int n = static_cast<int>( depth_inout.size() );
    uint16_t* depth = depth_inout.data();

    for (int i = 0; i < n; ++i) {
        depth[i] = AzureKinectDequantizeDepth(depth[i]);
    }
}


//------------------------------------------------------------------------------
// Depth Rescaling

void RescaleImage_11Bits(
    std::vector<uint16_t>& quantized,
    uint16_t& min_value,
    uint16_t& max_value)
{
    uint16_t* data = quantized.data();
    const int size = static_cast<int>( quantized.size() );

    // Find extrema
    int i;
    for (i = 0; i < size; ++i) {
        if (data[i] != 0) {
            break;
        }
    }
    if (i >= size) {
        min_value = max_value = 0;
        return;
    }
    unsigned smallest = data[i];
    unsigned largest = smallest;
    for (; i < size; ++i) {
        const unsigned x = data[i];
        if (x == 0) {
            continue;
        }
        if (smallest > x) {
            smallest = x;
        }
        if (largest < x) {
            largest = x;
        }
    }

    min_value = static_cast<uint16_t>( smallest );
    max_value = static_cast<uint16_t>( largest );

    // Handle edge cases
    const unsigned range = largest - smallest + 1;
    if (range >= 2048) {
        return;
    }
    if (range <= 1) {
        if (smallest != 0) {
            for (i = 0; i < size; ++i) {
                unsigned x = data[i];
                if (x == 0) {
                    continue;
                }
                data[i] = 1;
            }
        }
        return;
    }
    const unsigned rounder = range / 2;

    using branchfree_t = libdivide::branchfree_divider<unsigned>;
    branchfree_t fast_d = range;

    // Rescale the data
    for (i = 0; i < size; ++i) {
        unsigned x = data[i];
        if (x == 0) {
            continue;
        }
        x -= smallest;
        unsigned y = (x * 2047 + rounder) / fast_d;
        data[i] = static_cast<uint16_t>(y + 1);
    }
}

void UndoRescaleImage_11Bits(
    uint16_t min_value,
    uint16_t max_value,
    std::vector<uint16_t>& quantized)
{
    uint16_t* data = quantized.data();
    const int size = static_cast<int>( quantized.size() );

    const unsigned smallest = min_value;
    const unsigned range = max_value - smallest + 1;
    if (range >= 2048) {
        return;
    }
    if (range <= 1) {
        for (int i = 0; i < size; ++i) {
            unsigned x = data[i];
            if (x == 0) {
                continue;
            }
            data[i] = static_cast<uint16_t>( x - 1 + smallest );
        }
        return;
    }

    // Rescale the data
    for (int i = 0; i < size; ++i) {
        unsigned x = data[i];
        if (x == 0) {
            continue;
        }
        --x;
        const unsigned y = (x * range + 1023) / 2047;
        data[i] = static_cast<uint16_t>(y + smallest);
    }
}


//------------------------------------------------------------------------------
// Zstd

void ZstdCompress(
    const std::vector<uint8_t>& uncompressed,
    std::vector<uint8_t>& compressed)
{
    compressed.resize(ZSTD_compressBound(uncompressed.size()));
    const size_t size = ZSTD_compress(
        compressed.data(),
        compressed.size(),
        uncompressed.data(),
        uncompressed.size(),
        kZstdLevel);
    if (ZSTD_isError(size)) {
        compressed.clear();
        return;
    }
    compressed.resize(size);
}

bool ZstdDecompress(
    const uint8_t* compressed_data,
    int compressed_bytes,
    int uncompressed_bytes,
    std::vector<uint8_t>& uncompressed)
{
    uncompressed.resize(uncompressed_bytes);
    const size_t size = ZSTD_decompress(
        uncompressed.data(),
        uncompressed.size(),
        compressed_data,
        compressed_bytes);
    if (ZSTD_isError(size)) {
        return false;
    }
    if (size != static_cast<size_t>( uncompressed_bytes )) {
        return false;
    }
    return true;
}


//------------------------------------------------------------------------------
// DepthCompressor

void DepthCompressor::Compress(
    const VideoParameters& params,
    const uint16_t* unquantized_depth,
    std::vector<uint8_t>& compressed,
    bool keyframe)
{
    DepthHeader header;
    header.Magic = kDepthFormatMagic;
    header.Flags = 0;
    if (keyframe) {
        header.Flags |= DepthFlags_Keyframe;
    }
    if (params.Type == VideoType::H265) {
        header.Flags |= DepthFlags_HEVC;
    }
    header.Width = static_cast<uint16_t>( params.Width );
    header.Height = static_cast<uint16_t>( params.Height );
    const int n = params.Width * params.Height;

    // Enforce keyframe if we have not compressed anything yet
    if (FrameCount == 0) {
        keyframe = true;
    }
    header.FrameNumber = static_cast<uint16_t>( FrameCount );
    ++FrameCount;

    QuantizeDepthImage(n, unquantized_depth, QuantizedDepth);
    RescaleImage_11Bits(QuantizedDepth, header.MinimumDepth, header.MaximumDepth);
    Filter(QuantizedDepth);

    Codec.EncodeBegin(
        params,
        keyframe,
        Low,
        LowOut);

    // Interleave Zstd compression with video encoder work
    ZstdCompress(High, HighOut);
    header.HighUncompressedBytes = static_cast<uint32_t>( High.size() );
    header.HighCompressedBytes = static_cast<uint32_t>( HighOut.size() );

    Codec.EncodeFinish(LowOut);
    header.LowCompressedBytes = static_cast<uint32_t>( LowOut.size() );

    // Calculate output size
    size_t total_size = kDepthHeaderBytes + HighOut.size() + LowOut.size();
    compressed.resize(total_size);
    uint8_t* copy_dest = compressed.data();

    // Write header
    memcpy(copy_dest, &header, kDepthHeaderBytes);
    copy_dest += kDepthHeaderBytes;

    // Concatenate the compressed data
    memcpy(copy_dest, HighOut.data(), HighOut.size());
    copy_dest += HighOut.size();
    memcpy(copy_dest, LowOut.data(), LowOut.size());

    cout << "Zstd compression: " << HighOut.size() << endl;
    cout << "Video compression: " << LowOut.size() << endl;
}

DepthResult DepthCompressor::Decompress(
    const std::vector<uint8_t>& compressed,
    int& width,
    int& height,
    std::vector<uint16_t>& depth_out)
{
    if (compressed.size() < kDepthHeaderBytes) {
        return DepthResult::FileTruncated;
    }
    const uint8_t* src = compressed.data();

    const DepthHeader* header = reinterpret_cast<const DepthHeader*>( src );
    if (header->Magic != kDepthFormatMagic) {
        return DepthResult::WrongFormat;
    }
    const bool keyframe = (header->Flags & DepthFlags_Keyframe) != 0;
    VideoType video_codec_type = VideoType::H264;
    if ((header->Flags & DepthFlags_HEVC) != 0) {
        video_codec_type = VideoType::H265;
    }
    const unsigned frame_number = header->FrameNumber;

    // We can only start decoding on a keyframe because these contain SPS/PPS.
    if (!keyframe && FrameCount == 0) {
        return DepthResult::MissingFrame;
    }
    ++FrameCount;

#if 0 // This is okay I guess since we are using intra-frame compression.
    if (!keyframe && frame_number != CompressedFrameNumber + 1) {
        return DepthResult::MissingPFrame;
    }
#endif

    width = header->Width;
    height = header->Height;
    if (width < 1 || width > 4096 || height < 1 || height > 4096) {
        return DepthResult::Corrupted;
    }

    // Read header
    unsigned total_bytes = kDepthHeaderBytes + header->HighCompressedBytes + header->LowCompressedBytes;
    if (header->HighUncompressedBytes < 2) {
        return DepthResult::Corrupted;
    }
    if (compressed.size() != total_bytes) {
        return DepthResult::FileTruncated;
    }

    src += kDepthHeaderBytes;

    // Compress high bits
    bool success = ZstdDecompress(
        src,
        header->HighCompressedBytes,
        header->HighUncompressedBytes,
        High);
    if (!success) {
        return DepthResult::Corrupted;
    }

    src += header->HighCompressedBytes;

    success = Codec.Decode(
        width,
        height,
        video_codec_type,
        src,
        header->LowCompressedBytes,
        Low);
    if (!success) {
        return DepthResult::Corrupted;
    }

    src += header->LowCompressedBytes;

    Unfilter(width, height, depth_out);
    UndoRescaleImage_11Bits(header->MinimumDepth, header->MaximumDepth, depth_out);
    DequantizeDepthImage(depth_out);

    return DepthResult::Success;
}


//------------------------------------------------------------------------------
// DepthCompressor : Filtering

void DepthCompressor::Filter(
    const std::vector<uint16_t>& depth_in)
{
    const int n = static_cast<int>( depth_in.size() );
    const uint16_t* depth = depth_in.data();

    High.clear();
    Low.clear();
    High.resize(n / 2); // One byte for every two depth values
    Low.resize(n + n / 2); // Leave room for unused chroma channel

    // Split data into high/low parts
    for (int i = 0; i < n; i += 2) {
        const uint16_t depth_0 = depth[i];
        const uint16_t depth_1 = depth[i + 1];

        unsigned high_0 = 0, high_1 = 0;
        uint8_t low_0 = static_cast<uint8_t>( depth_0 );
        uint8_t low_1 = static_cast<uint8_t>( depth_1 );

        if (depth_0 != 0) {
            // Read high bits
            high_0 = depth_0 >> 8;

            // Fold to avoid sharp transitions from 255..0
            if (high_0 & 1) {
                low_0 = 255 - low_0;
            }

            // Preserve zeroes by offseting the values by 1
            ++high_0;
        }

        if (depth_1 != 0) {
            // Read high bits
            high_1 = depth_1 >> 8;

            // Fold to avoid sharp transitions from 255..0
            if (high_1 & 1) {
                low_1 = 255 - low_1;
            }

            // Preserve zeroes by offseting the values by 1
            ++high_1;
        }

        High[i / 2] = static_cast<uint8_t>( high_0 | (high_1 << 4) );
        Low[i] = low_0;
        Low[i + 1] = low_1;
    }
}

void DepthCompressor::Unfilter(
    int width,
    int height,
    std::vector<uint16_t>& depth_out)
{
    const int n = width * height;
    depth_out.resize(n);
    uint16_t* depth = depth_out.data();
    const uint8_t* low_data = Low.data();
    const uint8_t* high_data = High.data();

    for (int i = 0; i < n; i += 2) {
        const uint8_t high = high_data[i / 2];
        uint8_t low_0 = low_data[i];
        uint8_t low_1 = low_data[i + 1];
        unsigned high_0 = high & 15;
        unsigned high_1 = high >> 4;

        if (high_0 == 0) {
            depth[i] = 0;
        } else {
            high_0--;
            if (high_0 & 1) {
                low_0 = 255 - low_0;
            }
            uint16_t x = static_cast<uint16_t>(low_0 | (high_0 << 8));

            // This value is expected to always be at least 1
            if (x == 0) {
                x = 1;
            }

            depth[i] = x;
        }

        if (high_1 == 0) {
            depth[i + 1] = 0;
        } else {
            high_1--;
            if (high_1 & 1) {
                low_1 = 255 - low_1;
            }
            uint16_t y = static_cast<uint16_t>(low_1 | (high_1 << 8));

            // This value is expected to always be at least 1
            if (y == 0) {
                y = 1;
            }

            depth[i + 1] = y;
        }
    }
}


} // namespace zdepth
