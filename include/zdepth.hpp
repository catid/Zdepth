// Copyright 2019 (c) Christopher A. Taylor.  All rights reserved.

/*
    Zdepth

    Lossy depth buffer compression designed and tested for Azure Kinect DK.
    Based on the Facebook Zstd library and H.264/HEVC for compression.

    Zdepth defines a file format and performs full input checking.

    Hardware acceleration for H.264/HEVC video compression is leveraged when
    possible to reduce CPU usage during encoding and decoding.
*/

/*
    Compression algorithm:

        (1) Special case for zero.
            This ensures that the H.264 encoders do not flip zeroes.
        (1) Quantize depth to 11 bits based on sensor accuracy at range.
            Eliminate data that we do not need to encode.
        (2) Rescale the data so that it ranges from 0 to 2047.
            This is done to reduce the problems introduced by H.264.
        (3) Compress high 3 bits with Zstd.
        (4) Compress low 8 bits with H.264.

    High 3-bit compression with Zstd:

        (1) Prediction filtering (similar to PNG format).
            From the lossless depth encoder I found that the best predictor
            was one that predicted the larger of the depth value above and
            to the left.  The results are in the range -7..7.
        (2) Zig-zag encode to a range of 1..15.
            After subtracting with prediction, the result is signed so to
            make the result unsigned, we zig-zag encode, where zero is still
            zero but other values are twice as large and positive.
        (3) Combine 4-bit nibbles together into bytes.
        (4) Encode with Zstd.

    Low 8-bit compression with H.264:

        (1) Folding to avoid sharp transitions in the low bits.
            The low bits are submitted to the H.264 encoder, meaning that
            wherever the 8-bit value rolls over it transitions from
            255..0 again.  This sharp transition causes problems for the
            encoder so to solve that we fold every other 8-bit range
            by subtracting it from 255.  So instead the roll-over becomes
            253, 254, 255, 254, 253, ... 1, 0, 1, 2, ...
            This cuts the error about in half in testing.
        (2) Split the image into equal ranges of depth values.
            In practice this can be done by looking at the high bits.
        (3) Rescale back to the 0..255 range.
            After folding the data, we find the smallest and largest
            values in the image, and rescale the image values back to the
            0..255 range so that errors introduced by lossy compression
            have less impact on the result.
        (4) Compress the resulting data as an image with H.264.
            We use the best hardware acceleration available on the platform
            and attempt to run the multiple encoders in parallel.

    Further details are in the DepthCompressor::Filter() code.
*/

/*
    This is based on the research of:

    F. Nenci, L. Spinello and C. Stachniss,
    "Effective compression of range data streams for remote robot operations
    using H.264," 2014 IEEE/RSJ International Conference on Intelligent Robots
    and Systems, Chicago, IL, 2014, pp. 3794-3799.

    The main departure is in losslessly compressing the high bits.

    Uses libdivide: https://github.com/ridiculousfish/libdivide
    Uses Zstd: https://github.com/facebook/zstd
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

#include "VideoCodec.hpp"

namespace zdepth {


//------------------------------------------------------------------------------
// Constants

// First byte of the file format
static const uint8_t kDepthFormatMagic = 202; // 0xCA

enum DepthFlags
{
    DepthFlags_Keyframe = 1,    // Frame is an IDR
    DepthFlags_HEVC = 2,        // Use HEVC instead of H.264
};

// Number of bytes in header
static const int kDepthHeaderBytes = 44;

// Number of encoders to run in parallel
static const int kParallelEncoders = 2;

/*
    File format:

    Format Magic is used to quickly check that the file is of this format.
    Words are stored in little-endian byte order.

    Flags = 1 for I-frames and 0 for P-frames.
    The P-frames are able to use predictors that reference the previous frame.
    The decoder keeps track of the previously decoded Frame Number and rejects
    frames that cannot be decoded due to a missing previous frame.
*/

#pragma pack(push)
#pragma pack(1)

struct DepthHeader
{
    uint8_t Magic;
    uint8_t Flags;
    uint16_t FrameNumber;
    uint16_t Width;
    uint16_t Height;
    uint16_t MinimumDepth;
    uint16_t MaximumDepth;
    uint32_t HighUncompressedBytes;
    uint32_t HighCompressedBytes;
    uint32_t LowCompressedBytes[kParallelEncoders];
    uint8_t LowMinimum[kParallelEncoders];
    uint8_t LowMaximum[kParallelEncoders];
    // Compressed data follows: High bits, then low bits.
};

#pragma pack(pop)

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
    int n,
    const uint16_t* depth,
    std::vector<uint16_t>& quantized);

// This modifies the depth image in-place
void DequantizeDepthImage(std::vector<uint16_t>& depth_inout);


//------------------------------------------------------------------------------
// Depth Rescaling

/*
    The purpose of doing depth rescaling is for the benefit of accuracy in the
    H.264 lossy encoder.  If the whole scene does not contain any data far away
    then some of the video encoders will go unused unless we rescale the scene.
*/

// Rescale depth for a whole image to the range of 0..2047.
// This modifies the data in-place.
// Returns the minimum and maximum values in the data, needed for the decoder.
void RescaleImage_11Bits(
    std::vector<uint16_t>& quantized,
    uint16_t& min_value,
    uint16_t& max_value);

// Undo image rescaling.
// This modifies the data in-place.
void UndoRescaleImage_11Bits(
    uint16_t min_value,
    uint16_t max_value,
    std::vector<uint16_t>& quantized);

// Rescale depth for a whole image to the range of 0..255.
// This modifies the data in-place.
// Returns the minimum and maximum values in the data, needed for the decoder.
void RescaleImage_8Bits(
    std::vector<uint8_t>& quantized,
    uint8_t& min_value,
    uint8_t& max_value);

// Undo image rescaling.
// This modifies the data in-place.
void UndoRescaleImage_8Bits(
    uint8_t min_value,
    uint8_t max_value,
    std::vector<uint8_t>& quantized);


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
        VideoType video_codec_type, // H264 or HEVC
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
    // Depth values quantized
    std::vector<uint16_t> QuantizedDepth;
    unsigned CompressedFrameNumber = 0;

    std::vector<uint8_t> High;
    std::vector<uint8_t> Low[kParallelEncoders];

    // Results of compression
    std::vector<uint8_t> HighOut, LowOut[kParallelEncoders];

    // Video compressor used for low bits
    VideoCodec Codec[kParallelEncoders];


    // Transform the data for compression by Zstd/H.264
    void Filter(
        int width,
        int height,
        const std::vector<uint16_t>& depth_in);
    void Unfilter(
        int width,
        int height,
        std::vector<uint16_t>& depth_out);
};


} // namespace zdepth
