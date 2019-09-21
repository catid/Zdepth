// Copyright 2019 (c) Christopher A. Taylor.  All rights reserved.

/*
    ZdepthLossy

    Lossy depth buffer compression designed and tested for Azure Kinect DK.
    Based on the Facebook Zstd library for compression.

    The compressor defines a file format and performs full input checking.
    Supports temporal back-references similar to other video formats.

    Hardware acceleration for H.264 video compression is leveraged when
    possible to reduce CPU usage during encoding and decoding.
    To avoid using a lot of CPU time, a single encoder is used.

    Algorithm:

    (1) Quantize depth to 11 bits based on sensor accuracy at range.
    (2) Compress high 3 bits of each depth measurement with Zstd.
    (3) Compress low 8 bits of each depth measurement with H.264.
*/

#pragma once

#include <stdint.h>
#include <vector>

// Compiler-specific force inline keyword
#if defined(_MSC_VER)
    #define DEPTH_INLINE inline __forceinline
#else // _MSC_VER
    #define DEPTH_INLINE inline __attribute__((always_inline))
#endif // _MSC_VER

// Architecture check
#if defined(__arm__) || defined(_M_ARM)
    // Use aligned accesses on ARM
    #define DEPTH_ALIGNED_ACCESSES
#endif // ANDROID

#include "H264Codec.hpp"

namespace zdepth {


//------------------------------------------------------------------------------
// Constants

// First byte of the file format
static const uint8_t kDepthFormatMagic = 202; // 0xCA

// Number of bytes in header
static const int kDepthHeaderBytes = 32;

// Number of encoders to run in parallel
static const int kParallelEncoders = 4;

/*
    File format:

    Format Magic is used to quickly check that the file is of this format.
    Words are stored in little-endian byte order.

    0: <Format Magic = 202 (1 byte)>
    1: <Flags (1 byte)>
    2: <Frame Number (2 bytes)>
    4: <Width (2 bytes)>
    6: <Height (2 bytes)>
    8: <High Uncompressed Bytes (4 bytes)>
    12: <High Compressed Bytes (4 bytes)>
    16: <Low0 Compressed Bytes (4 bytes)>
    20: <Low1 Compressed Bytes (4 bytes)>
    24: <Low2 Compressed Bytes (4 bytes)>
    28: <Low3 Compressed Bytes (4 bytes)>
    Followed by compressed data.

    The compressed and uncompressed sizes are of packed data for High,Low0-3.

    Flags = 1 for I-frames and 0 for P-frames.
    The P-frames are able to use predictors that reference the previous frame.
    The decoder keeps track of the previously decoded Frame Number and rejects
    frames that cannot be decoded due to a missing previous frame.
*/

enum class DepthResult
{
    FileTruncated,
    WrongFormat,
    Corrupted,
    MissingPFrame, // Missing previous referenced frame
    Success
};

const char* DepthResultString(DepthResult result);


//------------------------------------------------------------------------------
// Tools

bool IsDepthFrame(const uint8_t* file_data, unsigned file_bytes);
bool IsKeyFrame(const uint8_t* file_data, unsigned file_bytes);


//------------------------------------------------------------------------------
// Depth Quantization

/*
    Azure Kinect DK sensor whitepaper:
    https://docs.microsoft.com/en-us/windows/mixed-reality/ISSCC-2018

    Minimum operating range = 200 mm.

    The Kinect has 1-2mm accuracy up to about 4 meters.
    Depth uncertainty < 0.2% of range:

        <750 mm  : 1.5 mm precision (or better)
        <1500 mm : 3 mm precision (or better)
        <3000 mm : 6 mm precision (or better)
        <6000 mm : 12 mm precision (or better)
        <12000 mm : 24 mm precision (or better)

    Our quantization table:

        [0, 200] mm      -> 0            (no depth data)
        [201, 750) mm    -> [1, 550)     (lossless)
        [750, 1500) mm   -> [550, 925)   (quantized 2x)
        [1500, 3000) mm  -> [925, 1300)  (quantized 4x)
        [3000, 6000) mm  -> [1300, 1675) (quantized 8x)
        [6000, 11840) mm -> [1675, 2040) (quantized 16x)
        Larger depth     -> 0            (no depth data)

    Reverse quantization table:

        0            -> 0                (no depth data)
        [1, 550)     -> [201, 750) mm    (lossless)
        [550, 925)   -> [750, 1500) mm   (quantized 2x)
        [925, 1300)  -> [1500, 3000) mm  (quantized 4x)
        [1300, 1675) -> [3000, 6000) mm  (quantized 8x)
        [1675, 2040) -> [6000, 11840) mm (quantized 16x)
        Larger values are invalid.
*/

// Quantize depth from 200..11840 mm to a value from 0..2040
uint16_t AzureKinectQuantizeDepth(uint16_t depth);

// Reverse quantization back to original depth
uint16_t AzureKinectDequantizeDepth(uint16_t quantized);

// Quantize depth for a whole image
void QuantizeDepthImage(
    int width,
    int height,
    const uint16_t* depth,
    std::vector<uint16_t>& quantized);
void DequantizeDepthImage(
    int width,
    int height,
    const uint16_t* quantized,
    std::vector<uint16_t>& depth);


//------------------------------------------------------------------------------
// Zstd

void ZstdCompress(
    const std::vector<uint8_t>& uncompressed,
    std::vector<uint8_t>& compressed);

bool ZstdDecompress(
    const uint8_t* compressed_data,
    int compressed_bytes,
    int uncompressed_bytes,
    std::vector<uint8_t>& uncompressed);


//------------------------------------------------------------------------------
// DepthCompressor

class DepthCompressor
{
public:
    // Compress depth array to buffer
    // Set keyframe to indicate this frame should not reference the previous one
    void Compress(
        int width,
        int height,
        const uint16_t* unquantized_depth,
        std::vector<uint8_t>& compressed,
        bool keyframe);

    // Decompress buffer to depth array.
    // Resulting depth buffer is row-first, stride=width*2 (no surprises).
    // Returns false if input is invalid
    DepthResult Decompress(
        const std::vector<uint8_t>& compressed,
        int& width,
        int& height,
        std::vector<uint16_t>& depth_out);

protected:
    // Depth values quantized for current and last frame
    std::vector<uint16_t> QuantizedDepth[2];
    unsigned CurrentFrameIndex = 0;
    unsigned CompressedFrameNumber = 0;

    std::vector<uint8_t> High;
    std::vector<uint8_t> Low[kParallelEncoders];

    int High_UncompressedBytes = 0;

    // Results of compression
    std::vector<uint8_t> HighOut, LowOut[kParallelEncoders];

    // Video compressor used for low bits
    H264Codec H264[kParallelEncoders];

    void SplitLow(
        int width,
        int height,
        const uint16_t* depth);
    void CombineLow(
        int width,
        int height,
        const uint16_t* depth);

    void CompressHigh(
        int width,
        int height,
        const uint16_t* depth,
        const uint16_t* prev_depth);
    bool DecompressImage(
        int width,
        int height,
        const uint8_t* data,
        int bytes);

    void WriteCompressedFile(
        int width,
        int height,
        bool keyframe,
        std::vector<uint8_t>& compressed);
};


} // namespace zdepth
