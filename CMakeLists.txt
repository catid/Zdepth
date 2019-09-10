cmake_minimum_required(VERSION 3.5)
project(zdepth VERSION 1.0.0 LANGUAGES CXX)

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
    add_subdirectory(zstd zstd)
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

target_link_libraries(zdepth PUBLIC zstd)

target_include_directories(zdepth PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

set_property(TARGET zdepth PROPERTY VERSION ${PROJECT_VERSION})
set_property(TARGET zdepth PROPERTY SOVERSION ${PROJECT_VERSION_MAJOR})

# zdepth test application

add_executable(zdepth_test tests/zdepth_test.cpp)
target_link_libraries(zdepth_test PRIVATE zdepth)


################################################################################
# Install

install(FILES LICENSE DESTINATION ".")
install(FILES README.md DESTINATION ".")

install(FILES ${INCLUDE_FILES} DESTINATION include)

install(TARGETS zdepth_test DESTINATION bin)

install(TARGETS
        zdepth
        zstd
    EXPORT zdepth-export
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
    EXPORT
        zdepth-export
    FILE
        ZdepthTargets.cmake
    NAMESPACE
        zdepth::
    DESTINATION
        "${CMAKE_INSTALL_LIBDIR}/cmake/zdepth"
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/ZdepthConfigVersion.cmake
    VERSION
        ${PROJECT_VERSION}
    COMPATIBILITY
        AnyNewerVersion
)
configure_package_config_file(
    ${CMAKE_CURRENT_LIST_DIR}/cmake/ZdepthConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/ZdepthConfig.cmake
    INSTALL_DESTINATION
        "${CMAKE_INSTALL_LIBDIR}/cmake/zdepth"
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/ZdepthConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/ZdepthConfigVersion.cmake
    DESTINATION
        "${CMAKE_INSTALL_LIBDIR}/cmake/zdepth"
)

export(
    EXPORT
        zdepth-export
    FILE
        ${CMAKE_CURRENT_BINARY_DIR}/ZdepthTargets.cmake
    NAMESPACE
        zdepth::
)
