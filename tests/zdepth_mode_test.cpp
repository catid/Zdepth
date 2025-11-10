// Copyright 2025 Tencent. All rights reserved.

#include <iostream>
#include <vector>

#include "zdepth.hpp"

std::vector<uint16_t> generate_depth_image(int width, int height, int low_value, int high_value) {
  std::vector<uint16_t> depth_image;
  depth_image.resize(height * width);
  int pixel_value = low_value;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      depth_image[y * width + x] = pixel_value;
      ++pixel_value;
      if (pixel_value > high_value) {
        pixel_value = low_value;
      }
    }
  }
  return depth_image;
}

bool Test(int width, int height, int low, int high, zdepth::EncodeMode mode) {
  std::vector<uint16_t> depth_image = generate_depth_image(width, height, low, high);
  zdepth::DepthCompressor encoder;
  encoder.set_encode_mode(mode);
  std::vector<uint8_t> compressed;
  zdepth::DepthResult ret = encoder.Compress(width, height, depth_image.data(), compressed, true);
  if (ret != zdepth::DepthResult::Success) {
    std::cerr << "encoder.Compress " << DepthResultString(ret) << std::endl;
    return false;
  }

  std::vector<uint16_t> decoded_depth_data;
  zdepth::DepthCompressor decoder;
  int depth_width, depth_height;
  ret = decoder.Decompress(compressed, depth_width, depth_height, decoded_depth_data);
  if (ret != zdepth::DepthResult::Success) {
    std::cerr << "decoder.Decompress " << DepthResultString(ret) << std::endl;
    return false;
  }
  if (depth_width != width || depth_height != height) {
    std::cerr << "decoder.Decompress invalid dimension" << std::endl;
    return false;
  }
  if (decoded_depth_data.size() != depth_image.size()) {
    std::cerr << "decoder.Decompress invalid size" << std::endl;
    return false;
  }
  if (low >= zdepth::GetCutoffDepth(mode)) {
    for (size_t i = 0; i < depth_image.size(); ++i) {
      if (decoded_depth_data[i] != 0) {
        std::cerr << "Unexpected decoded value " << i << " " << depth_image[i] << " " << decoded_depth_data[i]
                  << std::endl;
        return false;
      }
    }
  } else {
    for (size_t i = 0; i < depth_image.size(); ++i) {
      if (depth_image[i] != decoded_depth_data[i]) {
        std::cerr << "Unexpected decoded value " << i << " " << depth_image[i] << " " << decoded_depth_data[i]
                  << std::endl;
        return false;
      }
    }
  }
  return true;
}

int main() {
  for (zdepth::EncodeMode mode : {zdepth::EncodeMode::kNotQuantized511mm, zdepth::EncodeMode::kNotQuantized1023mm,
                                  zdepth::EncodeMode::kNotQuantized2047mm, zdepth::EncodeMode::kNotQuantized4095mm,
                                  zdepth::EncodeMode::kNotQuantized8191mm}) {
    for (int width = 16; width <= 640; width += zdepth::kBlockSize) {
      for (int height = 16; height <= 640; height += zdepth::kBlockSize) {
        std::cerr << "Testing " << EncodeModeString(mode) << " width=" << width << " height=" << height << std::endl;
        if (!Test(width, height, 0, 0, mode)) {
          std::cerr << "Failed in 0" << std::endl;
          return 1;
        }
        if (!Test(width, height, 1, zdepth::GetCutoffDepth(mode) - 1, mode)) {
          std::cerr << "Failed in normal range" << std::endl;
          return 1;
        }
        if (!Test(width, height, zdepth::GetCutoffDepth(mode), 65535, mode)) {
          std::cerr << "Failed in over range" << std::endl;
          return 1;
        }
      }
    }
  }

  return 0;
}
