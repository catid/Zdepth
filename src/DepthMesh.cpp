// Copyright 2019 (c) Christopher A. Taylor.  All rights reserved.

#include "DepthMesh.hpp"

namespace core {


//------------------------------------------------------------------------------
// DepthMesher

void DepthMesher::Initialize(
    const k4a_calibration_t& calibration,
    int triangle_thresh_mm)
{
    Calibration = calibration;
    ColorWidth = Calibration.color_camera_calibration.resolution_width;
    ColorHeight = Calibration.color_camera_calibration.resolution_height;
    TriangleThreshMm = triangle_thresh_mm;

    const int width = DepthWidth = Calibration.depth_camera_calibration.resolution_width;
    const int height = DepthHeight = Calibration.depth_camera_calibration.resolution_height;

    // Precalculate affine factors for 2D -> 3D conversion:

    const int n = width * height;
    DepthLookup.resize(n);

    int index = 0;
    for (int y = 0; y < height; y++)
    {
        k4a_float2_t p;
        p.xy.y = (float)y;

        for (int x = 0; x < width; x++, index++)
        {
            p.xy.x = (float)x;

            k4a_float3_t ray{};
            int valid = 0;

            // 3D point relative to depth camera from 2D depth camera pixel at 1 mm
            k4a_calibration_2d_to_3d(
                &Calibration,
                &p,
                1.f,
                K4A_CALIBRATION_TYPE_DEPTH,
                K4A_CALIBRATION_TYPE_DEPTH,
                &ray,
                &valid);

            k4a_float2_t lookup;
            if (valid) {
                lookup.xy.x = ray.xyz.x;
                lookup.xy.y = ray.xyz.y;
            } else {
                lookup.xy.x = lookup.xy.y = nanf("");
            }
            DepthLookup[index] = lookup;
        }
    }
}

void DepthMesher::GenerateCoordinates(uint16_t* depth, std::vector<float>& coordinates)
{
    const int width = DepthWidth;
    const int height = DepthHeight;
    const int n = width * height;

    const k4a_float2_t* lookup = DepthLookup.data();

    const float kInverseMeters = 1.f / 1000.f;
    const float inv_color_width = 1.f / static_cast<float>( ColorWidth );
    const float inv_color_height = 1.f / static_cast<float>( ColorHeight );

    // Extrinsics transform from depth -> color camera
    const k4a_calibration_extrinsics_t* extrinsics = &Calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR];
    const float* R = extrinsics->rotation;
    const float* t = extrinsics->translation;
    const k4a_calibration_camera_t* color_calibration = &Calibration.color_camera_calibration;
    const k4a_calibration_intrinsic_parameters_t *params = &color_calibration->intrinsics.parameters;
    const float cx = params->param.cx;
    const float cy = params->param.cy;
    const float fx = params->param.fx;
    const float fy = params->param.fy;
    const float k1 = params->param.k1;
    const float k2 = params->param.k2;
    const float k3 = params->param.k3;
    const float k4 = params->param.k4;
    const float k5 = params->param.k5;
    const float k6 = params->param.k6;
    const float codx = params->param.codx; // center of distortion is set to 0 for Brown Conrady model
    const float cody = params->param.cody;
    const float p1 = params->param.p1;
    const float p2 = params->param.p2;
    const float max_radius_for_projection = color_calibration->metric_radius;

    float dist_coeff = 1.f;
    if (color_calibration->intrinsics.type != K4A_CALIBRATION_LENS_DISTORTION_MODEL_RATIONAL_6KT)
    {
        // the only difference from Rational6ktCameraModel is 2 multiplier for the tangential coefficient term xyp*p1
        // and xyp*p2
        dist_coeff = 2.f;
    }

    coordinates.clear();
    coordinates.resize(n * 5);
    float* coordinates_next = coordinates.data();

    for (int i = 0; i < n; ++i)
    {
        const uint16_t depth_mm = depth[i];
        if (depth_mm == 0) {
            continue;
        }

        const k4a_float2_t scale = lookup[i];
        if (isnan(scale.xy.x)) {
            depth[i] = 0;
            continue;
        }

        // 73% of data is non-zero:

        // Convert to 3D (millimeters) relative to depth camera
        const float depth_x_mm = depth_mm * scale.xy.x;
        const float depth_y_mm = depth_mm * scale.xy.y;
        const float depth_z_mm = depth_mm;

        const float color_x_mm = R[0] * depth_x_mm + R[1] * depth_y_mm + R[2] * depth_z_mm + t[0];
        const float color_y_mm = R[3] * depth_x_mm + R[4] * depth_y_mm + R[5] * depth_z_mm + t[1];
        const float color_z_mm = R[6] * depth_x_mm + R[7] * depth_y_mm + R[8] * depth_z_mm + t[2];

        const float inv_z = 1.f / color_z_mm;
        const float x_proj = color_x_mm * inv_z;
        const float y_proj = color_y_mm * inv_z;

        const float xp = x_proj - codx;
        const float yp = y_proj - cody;

        const float xp2 = xp * xp;
        const float yp2 = yp * yp;
        const float xyp = xp * yp;
        const float rs = xp2 + yp2;

        if (rs > max_radius_for_projection * max_radius_for_projection) {
            depth[i] = 0;
            continue;
        }

        const float rss = rs * rs;
        const float rsc = rss * rs;
        const float a = 1.f + k1 * rs + k2 * rss + k3 * rsc;
        const float b = 1.f + k4 * rs + k5 * rss + k6 * rsc;
        float bi = 1.f;
        if (b != 0.f) {
            bi /= b;
        }
        const float d = a * bi;

        float xp_d = xp * d;
        float yp_d = yp * d;

        const float rs_2xp2 = rs + 2.f * xp2;
        const float rs_2yp2 = rs + 2.f * yp2;

        xp_d += rs_2xp2 * p2 + dist_coeff * xyp * p1;
        yp_d += rs_2yp2 * p1 + dist_coeff * xyp * p2;

        const float xp_d_cx = xp_d + codx;
        const float yp_d_cy = yp_d + cody;

        // Convert xyz to meters and normalized uv
        float u = xp_d_cx * fx + cx;
        float v = yp_d_cy * fy + cy;
        u *= inv_color_width;
        v *= inv_color_height;

        coordinates_next[0] = color_x_mm * kInverseMeters;
        coordinates_next[1] = color_y_mm * kInverseMeters;
        coordinates_next[2] = color_z_mm * kInverseMeters;
        coordinates_next[3] = u;
        coordinates_next[4] = v;
        coordinates_next += 5;
    } // next point

    // Resize to fit
    const uintptr_t size = static_cast<uintptr_t>( coordinates_next - coordinates.data() );
    coordinates.resize(size);
}

// Throw out triangles with too much depth mismatch
static bool CheckDepth(int a, int b, int c, int thresh_mm)
{
    if (abs(a - b) > thresh_mm) {
        return false;
    }
    if (abs(a - c) > thresh_mm) {
        return false;
    }
    if (abs(b - c) > thresh_mm) {
        return false;
    }
    return true;
}

void DepthMesher::GenerateTriangleIndices(const uint16_t* depth, std::vector<uint32_t>& indices)
{
    const int width = DepthWidth;
    const int height = DepthHeight;
    const int n = width * height;

    indices.clear();
    indices.resize(n * 2 * 3);
    unsigned* indices_next = indices.data();

    RowIndices.clear();
    RowIndices.resize(width * 2);
    unsigned* row_indices = RowIndices.data();

    const int thresh_mm = TriangleThreshMm;

    unsigned index = 0;
    for (int y = 0; y < height; ++y, depth += width)
    {
        // Offset into row_indices for current and previous rows
        const unsigned current_row_offset = (y % 2 == 0) ? width : 0;
        const unsigned prev_row_offset = (y % 2 != 0) ? width : 0;

        uint16_t c_depth = 0, depth_mm = 0;
        for (int x = 0; x < width; ++x, c_depth = depth_mm)
        {
            depth_mm = depth[x];
            if (depth_mm == 0) {
                continue;
            }

            const unsigned d_index = index;
            row_indices[x + current_row_offset] = index++;

            if (x == 0 || y == 0) {
                continue;
            }

            /*
                We are at position D.  If A,B,C are available,
                then we construct triangles from them where possible,
                and these triangles will be unique and complete.

                    A -- B
                    | \  |
                    |  \ |
                    C -- D
            */

            const uint16_t a_depth = depth[x - 1 - width];
            const uint16_t b_depth = depth[x - width];

            if (a_depth != 0) {
                if (c_depth != 0 && CheckDepth(a_depth, c_depth, depth_mm, thresh_mm)) {
                    const unsigned a_index = row_indices[x + prev_row_offset - 1];
                    indices_next[0] = a_index; // A
                    indices_next[1] = d_index - 1; // C
                    indices_next[2] = d_index; // D
                    indices_next += 3;
                }
                if (b_depth != 0 && CheckDepth(b_depth, a_depth, depth_mm, thresh_mm)) {
                    const unsigned a_index = row_indices[x + prev_row_offset - 1];
                    const unsigned b_index = row_indices[x + prev_row_offset];
                    indices_next[0] = b_index; // B
                    indices_next[1] = a_index; // A
                    indices_next[2] = d_index; // D
                    indices_next += 3;
                }
            } else if (b_depth != 0 && c_depth != 0 &&
                CheckDepth(b_depth, c_depth, depth_mm, thresh_mm))
            {
                const unsigned b_index = row_indices[x + prev_row_offset];
                indices_next[0] = b_index; // B
                indices_next[1] = d_index - 1; // C
                indices_next[2] = d_index; // D
                indices_next += 3;
            } // end if
        } // next x
    } // next y

    // Resize to fit
    const uintptr_t size = static_cast<uintptr_t>( indices_next - indices.data() );
    indices.resize(size);
}


} // namespace core
