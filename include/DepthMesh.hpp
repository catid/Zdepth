// Copyright 2019 (c) Christopher A. Taylor.  All rights reserved.

/*
    Depth mesh generator

    Takes 16-bit depth image and calibration as input.
    Produces OpenGL-compatible mesh x,y,z,u,v float vertices and a triangle index buffer.
    Zeroes out depth image values that cannot be used for the mesh for better compression.
*/

#pragma once

#include <k4a/k4a.h> // Azure Kinect SDK (C API)

#include <stdint.h>
#include <vector>

namespace core {

//#define ENABLE_ENOKI


//------------------------------------------------------------------------------
// DepthMesher

class DepthMesher
{
public:
    // Must be called before other functions
    void Initialize(const k4a_calibration_t& calibration, int triangle_thresh_mm = 100);

    // OpenGL-compatible x, y, z, u, v coordinates without padding.
    // Must match the dimensions from Initialize().
    // Zeroes out depth entries that contain invalid data.
    void GenerateCoordinates(uint16_t* depth, std::vector<float>& coordinates);

    // OpenGL-compatible 3 indices for each triangle.
    // Call after GenerateCoordinates().
    // Triangle vertices are wound such that the right-hand rule yields
    // normals pointing towards the camera.
    void GenerateTriangleIndices(const uint16_t* depth, std::vector<uint32_t>& indices);

protected:
    k4a_calibration_t Calibration;
    int DepthWidth, DepthHeight;
    int ColorWidth, ColorHeight;
    int TriangleThreshMm;

    std::vector<k4a_float2_t> DepthLookup;
    std::vector<unsigned> RowIndices;
};


} // namespace core
