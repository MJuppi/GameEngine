#define STB_IMAGE_IMPLEMENTATION
#include "engine/asset/TextureLoader.h"

#include <cctype>
#include <stdexcept>

#include "stb_image.h"

namespace ge {

static std::string getExtension(const std::string& path) {
    const auto dot = path.find_last_of('.');
    if (dot == std::string::npos) {
        return {};
    }

    std::string ext = path.substr(dot);
    for (auto& c : ext) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return ext;
}

TextureData loadTextureFile(const std::string& path) {
    const std::string extension = getExtension(path);
    if (extension.empty()) {
        throw std::runtime_error("TextureLoader: Texture path has no extension: " + path);
    }

    TextureData texture;
    texture.path = path;

    if (extension == ".hdr") {
        float* pixels = stbi_loadf(path.c_str(), &texture.width, &texture.height, &texture.channels, 0);
        if (!pixels) {
            throw std::runtime_error("TextureLoader: Failed to load HDR texture: " + path);
        }

        texture.hdr = true;
        texture.pixels.assign(reinterpret_cast<uint8_t*>(pixels), reinterpret_cast<uint8_t*>(pixels) + texture.width * texture.height * texture.channels * sizeof(float));
        stbi_image_free(pixels);
        return texture;
    }

    unsigned char* pixels = stbi_load(path.c_str(), &texture.width, &texture.height, &texture.channels, 4);
    if (!pixels) {
        throw std::runtime_error("TextureLoader: Failed to load texture: " + path);
    }

    texture.channels = 4;
    texture.pixels.assign(pixels, pixels + static_cast<size_t>(texture.width) * texture.height * 4);
    stbi_image_free(pixels);
    return texture;
}

} // namespace ge
