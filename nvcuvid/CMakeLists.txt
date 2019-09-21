cmake_minimum_required(VERSION 3.5)
project(nvcuvid LANGUAGES CXX)


################################################################################
# Dependencies

if (WIN32)
    set(CMAKE_FIND_LIBRARY_PREFIXES "")
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".lib")
else()
    set(CMAKE_FIND_LIBRARY_PREFIXES "lib")
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
endif()

# FIXME: Only 64-bit version works
set(LIB_SELECT "x64")

if (WIN32)
    set(NVCUVID_LIB "${CMAKE_SOURCE_DIR}/nvcuvid/vendored/${LIB_SELECT}/nvcuvid.lib")
else()
    find_library(NVCUVID_LIB
        NAMES
            nvcuvid
        NO_CMAKE_SYSTEM_PATH
    )
endif()

# FIXME: Only CUDA v10.1 works
find_library(CUDART_STATIC_LIB
    NAMES
        cudart_static
    HINTS
        "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v10.1/lib/${LIB_SELECT}/"
    NO_CMAKE_SYSTEM_PATH
)
find_library(CUDA_LIB
    NAMES
        cuda
    HINTS
        "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v10.1/lib/${LIB_SELECT}/"
    NO_CMAKE_SYSTEM_PATH
)
find_library(NVJPEG_LIB
    NAMES
        nvjpeg
    HINTS
        "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v10.1/lib/${LIB_SELECT}/"
    NO_CMAKE_SYSTEM_PATH
)

find_path(CUDA_INCLUDE_DIR cuda.h
    HINTS
        "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v10.1/include"
    REQUIRED
)

if (WIN32)
    set(CMAKE_FIND_LIBRARY_PREFIXES "")
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".dll")
else()
    set(CMAKE_FIND_LIBRARY_PREFIXES "lib")
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".so")
endif()

find_library(NVJPEG_DYNLIB
    NAMES
        nvjpeg64_10
    HINTS
        "C:/Program Files/NVIDIA Corporation/Installer2/nvjpeg_10.1.{11DCEEFC-510C-4982-8AE1-29E17B8EDD87}/bin"
    NO_CMAKE_SYSTEM_PATH
)

message("NVCUVID_LIB: ${NVCUVID_LIB}")
message("CUDART_STATIC_LIB: ${CUDART_STATIC_LIB}")
message("CUDA_LIB: ${CUDA_LIB}")
message("NVJPEG_LIB: ${NVJPEG_LIB}")
message("CUDA_INCLUDE_DIR: ${CUDA_INCLUDE_DIR}")
message("NVJPEG_DYNLIB: ${NVJPEG_DYNLIB}")


################################################################################
# Source

set(INCLUDE_FILES
    include/cuviddec.h
    include/nvcuvid.h
    include/NvDecoder.h
    include/NvEncoder.h
    include/nvEncodeAPI.h
    include/NvEncoderCuda.h
)

set(SOURCE_FILES
    ${INCLUDE_FILES}
    src/NvDecoder.cpp
    src/NvEncoder.cpp
    src/NvEncoderCuda.cpp
)

include_directories(include ${CUDA_INCLUDE_DIR})


################################################################################
# Targets

# nvcuvid library

add_library(nvcuvid STATIC ${SOURCE_FILES})
target_include_directories(nvcuvid PUBLIC include ${CUDA_INCLUDE_DIR})
target_link_libraries(nvcuvid PUBLIC
    ${NVCUVID_LIB}
    ${CUDART_STATIC_LIB}
    ${CUDA_LIB}
    ${NVJPEG_LIB}
)

install(FILES ${INCLUDE_FILES} DESTINATION include)
install(TARGETS nvcuvid DESTINATION lib)
