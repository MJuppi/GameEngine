#include "engine/asset/AssetLoader.h"
#include "engine/asset/TextureLoader.h"
#include "engine/mesh/GltfMeshLoader.h"
#include "engine/mesh/ObjMeshLoader.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <stdexcept>

namespace ge {

MeshData AssetLoader::loadMesh(const std::string& path) {
    const std::filesystem::path filePath(path);
    const auto extension = getExtension(filePath);
    if (extension.empty()) {
        throw std::runtime_error("AssetLoader: Mesh path has no extension: " + path);
    }

    const auto normalizedPath = std::filesystem::absolute(filePath).string();
    if (extension == ".gltf" || extension == ".glb") {
        return loadGltfFile(normalizedPath);
    }

    if (extension == ".obj") {
        return loadObjFile(normalizedPath);
    }

    throw std::runtime_error("AssetLoader: Unsupported mesh format: " + extension);
}

TextureData AssetLoader::loadTexture(const std::string& path) {
    const std::filesystem::path filePath(path);
    const auto extension = getExtension(filePath);
    if (extension.empty()) {
        throw std::runtime_error("AssetLoader: Texture path has no extension: " + path);
    }

    const auto normalizedPath = std::filesystem::absolute(filePath).string();
    return loadTextureFile(normalizedPath);
}

std::string AssetLoader::getExtension(const std::filesystem::path& path) {
    auto ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return ext;
}

} // namespace ge
