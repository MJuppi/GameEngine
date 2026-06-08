#include "engine/asset/AssetManager.h"
#include "engine/mesh/GltfMeshLoader.h"
#include "engine/mesh/ObjMeshLoader.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <stdexcept>

namespace ge {

const MeshData& AssetManager::loadMesh(const std::string& path) {
    const std::filesystem::path filePath(path);
    const auto extension = getExtension(filePath);
    if (extension.empty()) {
        throw std::runtime_error("AssetManager: Mesh path has no extension: " + path);
    }

    const auto normalizedPath = std::filesystem::absolute(filePath).string();
    auto it = cache_.find(normalizedPath);
    if (it != cache_.end()) {
        return it->second;
    }

    MeshData mesh;
    if (extension == ".gltf" || extension == ".glb") {
        mesh = loadGltfFile(normalizedPath);
    } else if (extension == ".obj") {
        mesh = loadObjFile(normalizedPath);
    } else {
        throw std::runtime_error("AssetManager: Unsupported mesh format: " + extension);
    }

    auto [insertIt, inserted] = cache_.emplace(normalizedPath, std::move(mesh));
    return insertIt->second;
}

bool AssetManager::hasMesh(const std::string& path) const {
    const auto normalizedPath = std::filesystem::absolute(std::filesystem::path(path)).string();
    return cache_.find(normalizedPath) != cache_.end();
}

const MeshData& AssetManager::getMesh(const std::string& path) const {
    const auto normalizedPath = std::filesystem::absolute(std::filesystem::path(path)).string();
    auto it = cache_.find(normalizedPath);
    if (it == cache_.end()) {
        throw std::runtime_error("AssetManager: Mesh not loaded: " + path);
    }
    return it->second;
}

std::string AssetManager::getExtension(const std::filesystem::path& path) {
    auto ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return ext;
}

} // namespace ge
