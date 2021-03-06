# Zdepth

Lossless depth buffer compression designed and tested for Microsoft Azure Kinect DK.
Based on the Facebook Zstd library.

There is a lossy version of this library here that produces files half the size,
in trade for slower compression and lower quality:
https://github.com/catid/ZdepthLossy

The compressor defines a file format and performs full input checking.
Supports temporal back-references similar to other video formats.

We do not use H.264 or other video codecs because the main goal is to
measure the limits of lossless real-time compression of depth data.
Another goal is to be 2x smaller than the best published software (RVL).

Algorithm:

+ Quantize depth based on sensor accuracy at range.
+ Compress run-lengths of zeroes with Zstd.
+ For each 8x8 block of the depth image, determine the best predictor.
+ Zig-zag encode, 12-bit pack, and compress the residuals with Zstd.
+ Store residuals near edges in a different context than on surface.

The predictors are similar to the way the PNG format works.


## Benchmarks

For streaming depth compression, I believe this is currently the best open-source option available online.

It runs in 1-2 milliseconds and produces compressed lossless depth video at 4-5 Mbps.  This is similar to the bandwidth needed for HD video (1080p@30FPS).

Detailed benchmarks and test vectors available at the end of the document.


## Applications

This is useful as part of a larger real-time volumetric video streaming system
enabled by the Microsoft Azure Kinect DK.  The only data that needs to be sent
for each frame are the compressed depth frames generated by this library,
and the JPEGs from the camera.

The video recorder reads a 16-bit depth image that is compressed with this
library.  These depth images are transmitted to the video player computer over
a network.  The video parameter set must include the calibration data for the
depth sensor.  The video player uses this library to decompress the depth image
and then uses the calibration data to generate xyz, uv, and indices for
rendering the mesh.

There is software here that can convert from 16-bit depth to xyzuv/indices in
only 3 milliseconds here: https://github.com/catid/ZdepthLossy


## Building and Using

There's example usage in the `tests` folder.

This project provides a CMakeLists.txt that generates a `zdepth` target.  Zdepth supports CMake `find_package()`.

```
    # Example CMake ExternalProject include
    include(ExternalProject)
    ExternalProject_Add(
        zdepth
        GIT_REPOSITORY  https://github.com/catid/Zdepth.git
        GIT_TAG         master
        GIT_SHALLOW     TRUE
        SOURCE_DIR      ${CMAKE_SOURCE_DIR}/extern/src/zdepth
        CMAKE_ARGS      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/extern/bin/zdepth
    )
```

Link to the target and include the header:

    #include "zdepth.hpp"

Compress 16-bit depth image to vector of bytes:

    zdepth::DepthCompressor compressor;

    ...

    std::vector<uint8_t> compressed;
    const bool is_keyframe = true;
    compressor.Compress(Width, Height, frame, compressed, is_keyframe);

Re-use the same DepthCompressor object for multiple frames for best performance.

Setting `is_keyframe = false` will use ~10% less bandwidth (e.g. 4 Mbps instead of 5 Mbps)
but has limitations just like normal video: You must guarantee delivery of all the frames
instead of dropping frames opportunistically.

Decompress vector of bytes back to 16-bit depth image:

    zdepth::DepthCompressor decompressor;

    int width, height;
    std::vector<uint16_t> depth;
    zdepth::DepthResult result = decompressor.Decompress(compressed, width, height, depth);
    if (result != zdepth::DepthResult::Success) {
        // Handle input error
    }

You must use different objects for the compressor and decompressor, or it will fail to decode.


## File Format

    Format Magic is used to quickly check that the file is of this format.
    Words are stored in little-endian byte order.

    0: <Format Magic = 202 (1 byte)>
    1: <Flags (1 byte)>
    2: <Frame Number (2 bytes)>
    4: <Width (2 bytes)>
    6: <Height (2 bytes)>
    8: <Zeroes Uncompressed Bytes (4 bytes)>
    12: <Zeroes Compressed Bytes (4 bytes)>
    16: <Blocks Uncompressed Bytes (4 bytes)>
    20: <Blocks Compressed Bytes (4 bytes)>
    24: <Edges Uncompressed Bytes (4 bytes)>
    28: <Edges Compressed Bytes (4 bytes)>
    32: <Surfaces Uncompressed Bytes (4 bytes)>
    36: <Surfaces Compressed Bytes (4 bytes)>

    Followed by compressed Zeroes, then Blocks, then Edges, then Surfaces.

    The compressed and uncompressed sizes are of packed data for Zstd.

    Flags = 1 for I-frames and 0 for P-frames.
    The P-frames are able to use predictors that reference the previous frame.
    The decoder keeps track of the previously decoded Frame Number and rejects
    frames that cannot be decoded due to a missing previous frame.

For more details on algorithms and format please check out the source code.
Feel free to modify the format for your data to improve the performance.


## Detailed Benchmark Results

    -------------------------------------------------------------------
    Test vector: Room
    -------------------------------------------------------------------

![Room Image](tests/room.jpg?raw=true "Room Image")

    ===================================================================
    + Test: Frame 0 Keyframe=true compression
    ===================================================================

    Zdepth Compression: 184320 bytes -> 22626 bytes (ratio = 8.14638:1) (5.43024 Mbps @ 30 FPS)
    Zdepth Speed: Compressed in 1.674 msec. Decompressed in 0.804 msec

    Quantization+RVL Compression: 184320 bytes -> 40312 bytes (ratio = 4.57234:1) (9.67488 Mbps @ 30 FPS)
    Quantization+RVL Speed: Compressed in 0.379 msec. Decompressed in 0.493 msec

    Quantization+RVL+Zstd Compression: 184320 bytes -> 29460 bytes (ratio = 6.25662:1) (7.0704 Mbps @ 30 FPS)
    Quantization+RVL+Zstd Speed: Compressed in 0.609 msec. Decompressed in 0.592 msec

    ===================================================================
    + Test: Frame 1 Keyframe=false compression
    ===================================================================

    Zdepth Compression: 184320 bytes -> 18703 bytes (ratio = 9.8551:1) (4.48872 Mbps @ 30 FPS)
    Zdepth Speed: Compressed in 1.471 msec. Decompressed in 0.679 msec

    Quantization+RVL Compression: 184320 bytes -> 40188 bytes (ratio = 4.58644:1) (9.64512 Mbps @ 30 FPS)
    Quantization+RVL Speed: Compressed in 0.446 msec. Decompressed in 0.491 msec

    Quantization+RVL+Zstd Compression: 184320 bytes -> 29291 bytes (ratio = 6.29272:1) (7.02984 Mbps @ 30 FPS)
    Quantization+RVL+Zstd Speed: Compressed in 0.683 msec. Decompressed in 0.593 msec

    -------------------------------------------------------------------
    Test vector: Ceiling
    -------------------------------------------------------------------

![Ceiling Image](tests/ceiling.jpg?raw=true "Ceiling Image")

    ===================================================================
    + Test: Frame 0 Keyframe=true compression
    ===================================================================

    Zdepth Compression: 184320 bytes -> 16499 bytes (ratio = 11.1716:1) (3.95976 Mbps @ 30 FPS)
    Zdepth Speed: Compressed in 1.463 msec. Decompressed in 0.845 msec

    Quantization+RVL Compression: 184320 bytes -> 42084 bytes (ratio = 4.37981:1) (10.1002 Mbps @ 30 FPS)
    Quantization+RVL Speed: Compressed in 0.376 msec. Decompressed in 0.461 msec

    Quantization+RVL+Zstd Compression: 184320 bytes -> 23251 bytes (ratio = 7.9274:1) (5.58024 Mbps @ 30 FPS)
    Quantization+RVL+Zstd Speed: Compressed in 0.69 msec. Decompressed in 0.583 msec

    ===================================================================
    + Test: Frame 1 Keyframe=false compression
    ===================================================================

    Zdepth Compression: 184320 bytes -> 15495 bytes (ratio = 11.8955:1) (3.7188 Mbps @ 30 FPS)
    Zdepth Speed: Compressed in 1.447 msec. Decompressed in 0.667 msec

    Quantization+RVL Compression: 184320 bytes -> 41948 bytes (ratio = 4.39401:1) (10.0675 Mbps @ 30 FPS)
    Quantization+RVL Speed: Compressed in 0.374 msec. Decompressed in 0.465 msec

    Quantization+RVL+Zstd Compression: 184320 bytes -> 23143 bytes (ratio = 7.9644:1) (5.55432 Mbps @ 30 FPS)
    Quantization+RVL+Zstd Speed: Compressed in 0.658 msec. Decompressed in 0.586 msec

    -------------------------------------------------------------------
    Test vector: Person
    -------------------------------------------------------------------

![Person Image](tests/person.jpg?raw=true "Person Image")

    ===================================================================
    + Test: Frame 0 Keyframe=true compression
    ===================================================================

    Zdepth Compression: 184320 bytes -> 23665 bytes (ratio = 7.78872:1) (5.6796 Mbps @ 30 FPS)
    Zdepth Speed: Compressed in 1.442 msec. Decompressed in 0.69 msec

    Quantization+RVL Compression: 184320 bytes -> 45016 bytes (ratio = 4.09454:1) (10.8038 Mbps @ 30 FPS)
    Quantization+RVL Speed: Compressed in 0.422 msec. Decompressed in 0.506 msec

    Quantization+RVL+Zstd Compression: 184320 bytes -> 33524 bytes (ratio = 5.49815:1) (8.04576 Mbps @ 30 FPS)
    Quantization+RVL+Zstd Speed: Compressed in 0.67 msec. Decompressed in 0.61 msec

    ===================================================================
    + Test: Frame 1 Keyframe=false compression
    ===================================================================

    Zdepth Compression: 184320 bytes -> 19848 bytes (ratio = 9.28658:1) (4.76352 Mbps @ 30 FPS)
    Zdepth Speed: Compressed in 1.514 msec. Decompressed in 0.718 msec

    Quantization+RVL Compression: 184320 bytes -> 45120 bytes (ratio = 4.08511:1) (10.8288 Mbps @ 30 FPS)
    Quantization+RVL Speed: Compressed in 0.4 msec. Decompressed in 0.514 msec

    Quantization+RVL+Zstd Compression: 184320 bytes -> 33667 bytes (ratio = 5.4748:1) (8.08008 Mbps @ 30 FPS)
    Quantization+RVL+Zstd Speed: Compressed in 0.642 msec. Decompressed in 0.619 msec


#### Credits

Zdepth uses the Zstd library (slightly modified).

Inspired by prior work:
+ Microsoft Fast Lossless Depth Image Compression (RVL library) available [here](https://www.microsoft.com/en-us/research/publication/fast-lossless-depth-image-compression/)
+ Edges/Surface split context was inspired by BCIF image format.
+ Trying different predictors was inspired by the PNG image format.
+ Encoding zero run lengths mask was inspired by my previous work on the GCIF image format.

Software by Christopher A. Taylor mrcatid@gmail.com

Please reach out if you need support or would like to collaborate on a project.
