// Copyright 2019 (c) Christopher A. Taylor.  All rights reserved.

#include "zdepth.hpp"
using namespace zdepth;


//------------------------------------------------------------------------------
// Timing

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
// RVL

// RVL library for performance baseline

// Paper: https://www.microsoft.com/en-us/research/publication/fast-lossless-depth-image-compression/
// Video presentation: https://www.youtube.com/watch?v=WYU2upBs2hA

// RVL author suggests that H.264 is a bad idea to use.
// But it seems like some masking can be used to avoid messing up the edges...

// Effective Compression of Range Data Streams for Remote Robot Operations using H.264
// http://www2.informatik.uni-freiburg.de/~stachnis/pdf/nenci14iros.pdf

// Adapting Standard Video Codecs for Depth Streaming
// http://reality.cs.ucl.ac.uk/projects/depth-streaming/depth-streaming.pdf

inline void EncodeVLE(int* &pBuffer, int& word, int& nibblesWritten, int value)
{
    do
    {
        int nibble = value & 0x7; // lower 3 bits
        if (value >>= 3) {
            nibble |= 0x8; // more to come
        }
        word <<= 4;
        word |= nibble;
        if (++nibblesWritten == 8) // output word
        {
            *pBuffer++ = word;
            nibblesWritten = 0;
            word = 0;
        }
    } while (value);
}

inline int DecodeVLE(int* &pBuffer, int& word, int& nibblesWritten)
{
    unsigned int nibble;
    int value = 0, bits = 29;
    do
    {
        if (!nibblesWritten)
        {
            word = *pBuffer++; // load word
            nibblesWritten = 8;
        }
        nibble = word & 0xf0000000;
        value |= (nibble << 1) >> bits;
        word <<= 4;
        nibblesWritten--;
        bits -= 3;
    } while (nibble & 0x80000000);
    return value;
}

int CompressRVL(short* input, char* output, int numPixels)
{
    int word{ 0 };
    int nibblesWritten;
    int *pBuffer;
    int *buffer = pBuffer = (int*)output;
    nibblesWritten = 0;
    short *end = input + numPixels;
    short previous = 0;
    while (input != end)
    {
        int zeros = 0, nonzeros = 0;
        for (; (input != end) && !*input; input++, zeros++);
        EncodeVLE(pBuffer, word, nibblesWritten, zeros); // number of zeros
        for (short* p = input; (p != end) && *p++; nonzeros++);
        EncodeVLE(pBuffer, word, nibblesWritten, nonzeros); // number of nonzeros
        for (int i = 0; i < nonzeros; i++)
        {
            short current = *input++;
            int delta = current - previous;
            int positive = (delta << 1) ^ (delta >> 31);
            EncodeVLE(pBuffer, word, nibblesWritten, positive); // nonzero value
            previous = current;
        }
    }
    if (nibblesWritten) // last few values
    {
        *pBuffer++ = word << 4 * (8 - nibblesWritten);
    }
    return int((char*)pBuffer - (char*)buffer); // num bytes
}

void DecompressRVL(char* input, short* output, int numPixels)
{
    int word, nibblesWritten;
    int *pBuffer = (int*)input;
    nibblesWritten = 0;
    short current, previous = 0;
    int numPixelsToDecode = numPixels;
    while (numPixelsToDecode)
    {
        int zeros = DecodeVLE(pBuffer, word, nibblesWritten); // number of zeros
        numPixelsToDecode -= zeros;
        for (; zeros; zeros--) {
            *output++ = 0;
        }
        int nonzeros = DecodeVLE(pBuffer, word, nibblesWritten); // number of nonzeros
        numPixelsToDecode -= nonzeros;
        for (; nonzeros; nonzeros--)
        {
            int positive = DecodeVLE(pBuffer, word, nibblesWritten); // nonzero value
            int delta = (positive >> 1) ^ -(positive & 1);
            current = previous + static_cast<short>(delta);
            *output++ = current;
            previous = current;
        }
    }
}


//------------------------------------------------------------------------------
// Test Vectors

#include "test_vectors.inl"


//------------------------------------------------------------------------------
// Test Application

#include <iostream>
using namespace std;

static zdepth::DepthCompressor compressor, decompressor;

bool TestFrame(const uint16_t* frame, bool keyframe)
{
    std::vector<uint8_t> compressed;

    static_assert(Width % kBlockSize == 0, "Width is not a multiple of block size.");
    static_assert(Height % kBlockSize == 0, "Height is not a multiple of block size.");

    const uint64_t t0 = GetTimeUsec();

    compressor.Compress(Width, Height, frame, compressed, keyframe);

    const uint64_t t1 = GetTimeUsec();

    int width, height;
    std::vector<uint16_t> depth;
    zdepth::DepthResult result = decompressor.Decompress(compressed, width, height, depth);

    const uint64_t t2 = GetTimeUsec();

    if (result != zdepth::DepthResult::Success) {
        cout << "Failed: decompressor.Decompress returned " << zdepth::DepthResultString(result) << endl;
        return false;
    }
    if (width != Width ||
        height != Height)
    {
        cout << "Decompression failed: Resolution" << endl;
        return false;
    }

    for (int i = 0; i < depth.size(); ++i) {
        if (AzureKinectQuantizeDepth(depth[i]) != AzureKinectQuantizeDepth(frame[i])) {
            cout << "Decompression failed: Contents did not match at offset = " << i << endl;
            return false;
        }
    }

    const unsigned original_bytes = Width * Height * 2;
    cout << endl;
    cout << "Zdepth Compression: " << original_bytes << " bytes -> " << compressed.size() <<
        " bytes (ratio = " << original_bytes / (float)compressed.size() << ":1) ("
        << (compressed.size() * 30 * 8) / 1000000.f << " Mbps @ 30 FPS)" << endl;
    cout << "Zdepth Speed: Compressed in " << (t1 - t0) / 1000.f << " msec. Decompressed in " << (t2 - t1) / 1000.f << " msec" << endl;

    const int n = Width * Height;
    std::vector<uint16_t> quantized(n);
    compressed.resize(n * 3);

    const uint64_t t3 = GetTimeUsec();
    QuantizeDepthImage(Width, Height, frame, quantized);
    const int compressed_bytes = CompressRVL((short*)quantized.data(), (char*)compressed.data(), n);
    compressed.resize(compressed_bytes);
    const uint64_t t4 = GetTimeUsec();

    std::vector<uint8_t> recompressed;
    std::vector<uint8_t> decompressed;

    const uint64_t t5 = GetTimeUsec();
    ZstdCompress(compressed, recompressed);
    const uint64_t t6 = GetTimeUsec();
    ZstdDecompress(recompressed.data(), static_cast<int>(recompressed.size()), static_cast<int>(compressed.size()), decompressed);
    const uint64_t t7 = GetTimeUsec();
    quantized.resize(n * 2);
    DecompressRVL((char*)decompressed.data(), (short*)quantized.data(), n);
    DequantizeDepthImage(Width, Height, quantized.data(), depth);
    const uint64_t t8 = GetTimeUsec();

    for (int i = 0; i < depth.size(); ++i) {
        if (AzureKinectQuantizeDepth(depth[i]) != AzureKinectQuantizeDepth(frame[i])) {
            cout << "Decompression failed: Contents did not match at offset = " << i << endl;
            return false;
        }
    }

    cout << endl;
    cout << "Quantization+RVL Compression: " << original_bytes << " bytes -> " << compressed.size() <<
        " bytes (ratio = " << original_bytes / (float)compressed.size() << ":1) ("
        << (compressed.size() * 30 * 8) / 1000000.f << " Mbps @ 30 FPS)" << endl;
    cout << "Quantization+RVL Speed: Compressed in " << (t4 - t3) / 1000.f << " msec. Decompressed in " << (t8 - t7) / 1000.f << " msec" << endl;

    cout << endl;
    cout << "Quantization+RVL+Zstd Compression: " << original_bytes << " bytes -> " << recompressed.size() <<
        " bytes (ratio = " << original_bytes / (float)recompressed.size() << ":1) ("
        << (recompressed.size() * 30 * 8) / 1000000.f << " Mbps @ 30 FPS)" << endl;
    cout << "Quantization+RVL+Zstd Speed: Compressed in " << (t6 - t5 + t4 - t3) / 1000.f << " msec. Decompressed in " << (t8 - t6) / 1000.f << " msec" << endl;

    return true;
}

bool TestPattern(const uint16_t* frame0, const uint16_t* frame1)
{
    cout << endl;
    cout << "===================================================================" << endl;
    cout << "+ Test: Frame 0 Keyframe=true compression" << endl;
    cout << "===================================================================" << endl;

    if (!TestFrame(frame0, true)) {
        cout << "Failure: frame0 failed";
        return false;
    }

    cout << endl;
    cout << "===================================================================" << endl;
    cout << "+ Test: Frame 1 Keyframe=false compression" << endl;
    cout << "===================================================================" << endl;

    if (!TestFrame(frame1, false)) {
        cout << "Failure: frame1 failed";
        return false;
    }
    return true;
}

int main(int argc, char* argv[])
{
    (void)(argc); (void)(argv);

    cout << endl;
    cout << "-------------------------------------------------------------------" << endl;
    cout << "Test vector: Room" << endl;
    cout << "-------------------------------------------------------------------" << endl;

    if (!TestPattern(TestVector0_Room0, TestVector0_Room1)) {
        cout << "Test failure: Room test vector" << endl;
        return -1;
    }

    cout << endl;
    cout << "-------------------------------------------------------------------" << endl;
    cout << "Test vector: Ceiling" << endl;
    cout << "-------------------------------------------------------------------" << endl;

    if (!TestPattern(TestVector1_Ceiling0, TestVector1_Ceiling1)) {
        cout << "Test failure: Ceiling test vector" << endl;
        return -2;
    }

    cout << endl;
    cout << "-------------------------------------------------------------------" << endl;
    cout << "Test vector: Person" << endl;
    cout << "-------------------------------------------------------------------" << endl;

    if (!TestPattern(TestVector2_Person0, TestVector2_Person1)) {
        cout << "Test failure: Person test vector" << endl;
        return -3;
    }
    cout << endl;

    cout << "Test success" << endl;
    return 0;
}
