cmake_minimum_required(VERSION 3.5)
project(zdepth LANGUAGES CXX)
include("cmake/zdepth-config-version.cmake")
set(CMAKE_CXX_STANDARD 11)


################################################################################
# Build Options

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# In debug mode, add -DDEBUG
add_compile_options("$<$<CONFIG:DEBUG>:-DDEBUG>")

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
else()
    # Warnings
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")

    # Remove Asio warning
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-local-typedefs")

    # Static library: -fPIC
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

    set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -march=native -fstack-protector")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native")
endif()


################################################################################
# Subprojects

if (NOT TARGET zstd)
    add_subdirectory(zstd)
endif()


################################################################################
# Source

set(INCLUDE_FILES
    include/zdepth.hpp
)

set(SOURCE_FILES
    ${INCLUDE_FILES}
    src/zdepth.cpp
)

include_directories(include)


################################################################################
# Targets

# zdepth library

add_library(zdepth STATIC ${SOURCE_FILES})
add_library(zdepth::zdepth ALIAS zdepth)

target_link_libraries(zdepth PUBLIC zstd)

target_include_directories(zdepth PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

# zdepth test application

add_executable(zdepth_test tests/zdepth_test.cpp)
target_link_libraries(zdepth_test PRIVATE zdepth)


################################################################################
# Install

install(FILES LICENSE DESTINATION ${CMAKE_INSTALL_PREFIX})
install(FILES README.md DESTINATION ${CMAKE_INSTALL_PREFIX})
install(DIRECTORY cmake include DESTINATION ${CMAKE_INSTALL_PREFIX})
install(TARGETS zdepth EXPORT zdepth LIBRARY DESTINATION lib)

################################################################################
# Generate zdepthConfig.cmake and zdepthConfigVersion.cmake for cmake projects

set(export_dest_dir "lib/cmake/zdepth")
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    zdepthConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
    )

export(TARGETS zdepth zstd
    NAMESPACE zdepth::
    FILE "${CMAKE_CURRENT_BINARY_DIR}/zdepthConfig.cmake")

install(EXPORT zdepth
        DESTINATION ${export_dest_dir}
        NAMESPACE zdepth::
        FILE zdepthConfig.cmake)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/zdepthConfigVersion.cmake"
        DESTINATION "${export_dest_dir}"
        )
