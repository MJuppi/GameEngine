#pragma once

#include <string>
#include <unordered_map>
#include <filesystem>

namespace ge {

struct AssetManifest {
    std::unordered_map<std::string, std::string> meshes;
    std::unordered_map<std::string, std::string> textures;

    static AssetManifest load(const std::filesystem::path& manifestPath);
};

} // namespace ge
