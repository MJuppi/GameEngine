#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ge {

struct TextureData {
    std::string path;
    int width = 0;
    int height = 0;
    int channels = 0;
    std::vector<uint8_t> pixels;
    bool hdr = false;
};

} // namespace ge
