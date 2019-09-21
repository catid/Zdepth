// Copyright 2019 (c) Christopher A. Taylor.  All rights reserved.

#include "zdepth.hpp"

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
    case DepthResult::FileTruncated: return "FileTruncated";
    case DepthResult::WrongFormat: return "WrongFormat";
    case DepthResult::Corrupted: return "Corrupted";
    case DepthResult::MissingPFrame: return "MissingPFrame";
    case DepthResult::Success: return "Success";
    default: break;
    }
    return "Unknown";
}


//------------------------------------------------------------------------------
// Tools

// Little-endian 16-bit read
DEPTH_INLINE uint16_t ReadU16_LE(const void* data)
{
#ifdef DEPTH_ALIGNED_ACCESSES
    const uint8_t* u8p = reinterpret_cast<const uint8_t*>(data);
    return ((uint16_t)u8p[1] << 8) | u8p[0];
#else
    const uint16_t* word_ptr = reinterpret_cast<const uint16_t*>(data);
    return *word_ptr;
#endif
}

// Little-endian 32-bit read
DEPTH_INLINE uint32_t ReadU32_LE(const void* data)
{
#ifdef DEPTH_ALIGNED_ACCESSES
    const uint8_t* u8p = reinterpret_cast<const uint8_t*>(data);
    return ((uint32_t)u8p[3] << 24) | ((uint32_t)u8p[2] << 16) | ((uint32_t)u8p[1] << 8) | u8p[0];
#else
    const uint32_t* u32p = reinterpret_cast<const uint32_t*>(data);
    return *u32p;
#endif
}

// Little-endian 16-bit write
DEPTH_INLINE void WriteU16_LE(void* data, uint16_t value)
{
#ifdef DEPTH_ALIGNED_ACCESSES
    uint8_t* u8p = reinterpret_cast<uint8_t*>(data);
    u8p[1] = static_cast<uint8_t>(value >> 8);
    u8p[0] = static_cast<uint8_t>(value);
#else
    uint16_t* word_ptr = reinterpret_cast<uint16_t*>(data);
    *word_ptr = value;
#endif
}

// Little-endian 32-bit write
DEPTH_INLINE void WriteU32_LE(void* data, uint32_t value)
{
#ifdef DEPTH_ALIGNED_ACCESSES
    uint8_t* u8p = reinterpret_cast<uint8_t*>(data);
    u8p[3] = (uint8_t)(value >> 24);
    u8p[2] = static_cast<uint8_t>(value >> 16);
    u8p[1] = static_cast<uint8_t>(value >> 8);
    u8p[0] = static_cast<uint8_t>(value);
#else
    uint32_t* word_ptr = reinterpret_cast<uint32_t*>(data);
    *word_ptr = value;
#endif
}

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
    int width,
    int height,
    const uint16_t* depth,
    std::vector<uint16_t>& quantized)
{
    const int n = width * height;
    quantized.resize(n);
    uint16_t* dest = quantized.data();

    for (int i = 0; i < n; ++i) {
        dest[i] = AzureKinectQuantizeDepth(depth[i]);
    }
}

void DequantizeDepthImage(
    int width,
    int height,
    const uint16_t* quantized,
    std::vector<uint16_t>& depth)
{
    const int n = width * height;
    depth.resize(n);
    uint16_t* dest = depth.data();

    for (int i = 0; i < n; ++i) {
        dest[i] = AzureKinectDequantizeDepth(quantized[i]);
    }
}


//------------------------------------------------------------------------------
// Depth Predictors

// Supported predictor functions
enum PredictorTypes
{
    PredictorType_Larger,
    PredictorType_Up,
    PredictorType_Left,
    PredictorType_UpTrend,
    PredictorType_LeftTrend,
    PredictorType_Average,

    // The following predictor types sample the previous frame:
    PredictorType_PrevFrame,

    // Number of predictors implemented
    PredictorType_Count
};

static inline unsigned ApplyPrediction(int depth, int prediction)
{
    // Apply prediction
    const int32_t delta = static_cast<int32_t>( depth - prediction );

    // Zig-zag encoding to make the signed number positive
    return (delta << 1) ^ (delta >> 31);
}

static inline int UndoPrediction(unsigned zigzag, int prediction)
{
    // Undo zig-zag encoding to get signed number
    const int delta = (zigzag >> 1) ^ -static_cast<int32_t>(zigzag & 1);

    // Undo prediction
    return delta + prediction;
}

static inline int Predict_Larger(int left0, int up0)
{
    return left0 > up0 ? left0 : up0;
}

static inline int Predict_Up(int left0, int up0)
{
    return up0 != 0 ? up0 : left0;
}

static inline int Predict_Left(int left0, int up0)
{
    return left0 != 0 ? left0 : up0;
}

static inline int Predict_UpTrend(int left0, int up0, int up1)
{
    if (up0 && up1) {
        return 2 * up0 - up1;
    }
    return Predict_Larger(left0, up0);
}

static inline int Predict_LeftTrend(int left0, int left1, int up0)
{
    if (left0 && left1) {
        return 2 * left0 - left1;
    }
    return Predict_Larger(left0, up0);
}

static inline int Predict_Average(int left0, int up0)
{
    if (left0 && up0) {
        return (left0 + up0) / 2;
    }
    return Predict_Larger(left0, up0);
}

static inline int Predict_PrevFrame(int prev0, int left0, int up0)
{
    if (prev0) {
        return prev0;
    }
    return Predict_Larger(left0, up0);
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
    int width,
    int height,
    const uint16_t* unquantized_depth,
    std::vector<uint8_t>& compressed,
    bool keyframe)
{
    // Enforce keyframe if we have not compressed anything yet
    if (CompressedFrameNumber == 0) {
        keyframe = true;
    }
    ++CompressedFrameNumber;

    // Quantize the depth image
    QuantizeDepthImage(width, height, unquantized_depth, QuantizedDepth[CurrentFrameIndex]);

    // Get depth for previous frame
    const uint16_t* depth = QuantizedDepth[CurrentFrameIndex].data();

    CurrentFrameIndex = (CurrentFrameIndex + 1) % 2;
    const uint16_t* prev_depth = nullptr;
    if (!keyframe) {
        prev_depth = QuantizedDepth[CurrentFrameIndex].data();
    }


    // FIXME: Preprocess the High part based on previous frame

    ZstdCompress(High, HighOut);
    SplitLow(width, height, depth);
    for (int i = 0; i < kParallelEncoders; ++i) {
        H264[i].EncodeBegin(width, height, keyframe, Low[i], LowOut[i]);
    }
    for (int i = 0; i < kParallelEncoders; ++i) {
        H264[i].EncodeFinish(LowOut[i]);
    }
    WriteCompressedFile(width, height, keyframe, compressed);
}

void DepthCompressor::SplitLow(
    int width,
    int height,
    const uint16_t* depth)
{
    // Split data into high/low parts
    const int n = width * height;
    High.resize(n / 2);
    for (int i = 0; i < kParallelEncoders; ++i) {
        Low[i].resize(n + n / 2);
    }

    for (int i = 0; i < n; i += 2) {
        const uint16_t depth_0 = depth[i];
        const uint16_t depth_1 = depth[i + 1];
        unsigned high_0 = 0, high_1 = 0;
        if (depth_0) {
            high_0 = (depth_0 >> 8) + 1;
        }
        if (depth_1) {
            high_1 = (depth_1 >> 8) + 1;
        }
        High[i / 2] = static_cast<uint8_t>( high_0 | (high_1 << 4) );
        Low[i] = static_cast<uint8_t>( depth_0 );
        Low[i + 1] = static_cast<uint8_t>( depth_1 );
    }
}

void DepthCompressor::WriteCompressedFile(
    int width,
    int height,
    bool keyframe,
    std::vector<uint8_t>& compressed)
{
    compressed.resize(
        kDepthHeaderBytes +
        HighOut.size() +
        LowOut.size());
    uint8_t* copy_dest = compressed.data();

    // Write header
    copy_dest[0] = kDepthFormatMagic;

    uint8_t flags = 0;
    if (keyframe) {
        flags |= 1;
    }
    copy_dest[1] = flags;

    WriteU16_LE(copy_dest + 2, static_cast<uint16_t>( CompressedFrameNumber ));
    WriteU16_LE(copy_dest + 4, static_cast<uint16_t>( width ));
    WriteU16_LE(copy_dest + 6, static_cast<uint16_t>( height ));
    WriteU32_LE(copy_dest + 8, static_cast<uint32_t>( High.size() ));
    WriteU32_LE(copy_dest + 12, static_cast<uint32_t>( HighOut.size() ));
    WriteU32_LE(copy_dest + 16, static_cast<uint32_t>( Low.size() ));
    WriteU32_LE(copy_dest + 20, static_cast<uint32_t>( LowOut.size() ));
    copy_dest += kDepthHeaderBytes;

    // Concatenate the compressed data
    memcpy(copy_dest, HighOut.data(), HighOut.size());
    copy_dest += HighOut.size();
    memcpy(copy_dest, LowOut.data(), LowOut.size());
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
    if (src[0] != kDepthFormatMagic) {
        return DepthResult::WrongFormat;
    }
    bool keyframe = (src[1] & 1) != 0;
    const unsigned frame_number = ReadU16_LE(src + 2);

    if (!keyframe && frame_number != CompressedFrameNumber + 1) {
        return DepthResult::MissingPFrame;
    }
    CompressedFrameNumber = frame_number;

    width = ReadU16_LE(src + 4);
    height = ReadU16_LE(src + 6);
    if (width < 1 || width > 4096 || height < 1 || height > 4096) {
        return DepthResult::Corrupted;
    }

    // Get depth for previous frame
    const int n = width * height;
    QuantizedDepth[CurrentFrameIndex].resize(n);
    uint16_t* depth = QuantizedDepth[CurrentFrameIndex].data();
    CurrentFrameIndex = (CurrentFrameIndex + 1) % 2;
    uint16_t* prev_depth = nullptr;
    if (!keyframe) {
        if (QuantizedDepth[CurrentFrameIndex].size() != static_cast<size_t>( n )) {
            return DepthResult::MissingPFrame;
        }
        prev_depth = QuantizedDepth[CurrentFrameIndex].data();
    }

    High_UncompressedBytes = ReadU32_LE(src + 8);
    const unsigned High_CompressedBytes = ReadU32_LE(src + 12);
    Low_UncompressedBytes = ReadU32_LE(src + 16);
    const unsigned Low_CompressedBytes = ReadU32_LE(src + 20);

    if (High_UncompressedBytes < 2) {
        return DepthResult::Corrupted;
    }

    if (compressed.size() !=
        kDepthHeaderBytes +
        High_CompressedBytes +
        Low_CompressedBytes)
    {
        return DepthResult::FileTruncated;
    }

    const uint8_t* High_Data = src + kDepthHeaderBytes;
    const uint8_t* Low_Data = High_Data + High_CompressedBytes;

    bool success = ZstdDecompress(
        High_Data,
        High_CompressedBytes,
        High_UncompressedBytes,
        High);
    if (!success) {
        return DepthResult::Corrupted;
    }

    // FIXME: Decode

    success = H264.Decode(
        width,
        height,
        Low_Data,
        Low_CompressedBytes,
        Low);
    if (!success) {
        return DepthResult::Corrupted;
    }

    for (int i = 0; i < n; i += 2) {
        const uint8_t high = High[i / 2];
        const uint8_t low_0 = Low[i];
        const uint8_t low_1 = Low[i + 1];
        unsigned high_0 = high & 15;
        unsigned high_1 = high >> 4;
        if (high_0 == 0) {
            depth[i] = 0;
        } else {
            high_0--;
            depth[i] = static_cast<uint16_t>(low_0 | (high_0 << 8));
        }
        if (high_1 == 0) {
            depth[i + 1] = 0;
        } else {
            high_1--;
            depth[i + 1] = static_cast<uint16_t>(low_1 | (high_1 << 8));
        }
    }

    DequantizeDepthImage(width, height, depth, depth_out);
    return DepthResult::Success;
}


} // namespace zdepth
