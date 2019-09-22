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
// Test Vectors

#include "test_vectors.inl"


//------------------------------------------------------------------------------
// Test Application

#include <iostream>
using namespace std;

static zdepth::DepthCompressor compressor, decompressor;

static void GraphResult(size_t n, const uint16_t* depth, const uint16_t* frame)
{
    const int width = 200;
    const int height = 280;
    const int stride = 320;
    const int offx = 100;
    const int offy = 0;

    cout << "Error plot:" << endl;
    for (int yy = 0; yy < height; ++yy) {
        for (int xx = 0; xx < width; ++xx) {
            const int i = xx + offx + (yy + offy) * stride;
            const int x = AzureKinectQuantizeDepth(depth[i]);
            const int y = AzureKinectQuantizeDepth(frame[i]);

            unsigned z = std::abs(x - y);
            if (z == 0) {
                cout << " ";
            }
            else if (z < 16) {
                cout << ".";
            }
            else {
                cout << "!";
            }
        }
        cout << endl;
    }

#if 0
    cout << "High bits plot:" << endl;
    for (int yy = 0; yy < height; ++yy) {
        for (int xx = 0; xx < width; ++xx) {
            const int i = xx + offx + (yy + offy) * stride;
            const int x = AzureKinectQuantizeDepth(frame[i]);

            if (x == 0) {
                cout << " ";
            }
            else if (x & 1) {
                cout << ".";
            }
            else {
                cout << ",";
            }
        }
        cout << endl;
    }
#endif
}

static bool CompareFrames(size_t n, const uint16_t* depth, const uint16_t* frame)
{
    std::vector<unsigned> error_hist(256);

    for (int i = 0; i < n; ++i) {
        const int x = AzureKinectQuantizeDepth(depth[i]);
        const int y = AzureKinectQuantizeDepth(frame[i]);

        unsigned z = std::abs(x - y);
        if (z == 0) {
            continue;
        }
        if (z > 255) {
            z = 255;
        }
        error_hist[z]++;
    }
#if 1
    for (int i = 0; i < 256; ++i) {
        if (error_hist[i]) {
            cout << "Error hist: " << i << " : " << error_hist[i] << endl;
        }
    }
#endif

    //GraphResult(n, depth, frame);

    return true;
}

bool TestRescale()
{
    {
        std::vector<uint16_t> q;

        // start
        for (unsigned i = 0; i < 2048; ++i)
        {
            // end
            for (unsigned j = i; j < 2048; ++j)
            {
                int n = j - i + 1;
                q.resize(n);

                for (unsigned k = 0; k < n; ++k) {
                    q[k] = k + i;
                }

                uint16_t min_value, max_value;
                RescaleImage_11Bits(q, min_value, max_value);
                UndoRescaleImage_11Bits(min_value, max_value, q);

                for (unsigned k = 0; k < n; ++k) {
                    if (q[k] != k + i) {
                        cout << "FAILED" << endl;
                        return false;
                    }
                }
            }
        }
    }

    return true;
}

bool TestFrame(const uint16_t* frame, bool keyframe)
{
    std::vector<uint8_t> compressed;

    const uint64_t t0 = GetTimeUsec();

    VideoParameters params{};
    params.Width = Width;
    params.Height = Height;
    params.Type = VideoType::H264;
    params.Fps = 30;

    compressor.Compress(
        params,
        frame,
        compressed,
        keyframe);

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

    if (!CompareFrames(depth.size(), depth.data(), frame)) {
        cout << "Decompression result corrupted" << endl;
        return false;
    }

    const unsigned original_bytes = Width * Height * 2;
    cout << endl;
    cout << "Zdepth Compression: " << original_bytes << " bytes -> " << compressed.size() << 
        " bytes (ratio = " << original_bytes / (float)compressed.size() << ":1) ("
        << (compressed.size() * 30 * 8) / 1000000.f << " Mbps @ 30 FPS)" << endl;
    cout << "Zdepth Speed: Compressed in " << (t1 - t0) / 1000.f << " msec. Decompressed in " << (t2 - t1) / 1000.f << " msec" << endl;

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
    cout << "Test vector: Ceiling" << endl;
    cout << "-------------------------------------------------------------------" << endl;

    if (!TestPattern(TestVector1_Ceiling0, TestVector1_Ceiling1)) {
        cout << "Test failure: Ceiling test vector" << endl;
        return -2;
    }

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
