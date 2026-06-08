#include "engine/asset/AssetManager.h"
#include "engine/asset/AssetLoader.h"

#include <filesystem>
#include <stdexcept>

namespace ge {

const MeshData& AssetManager::loadMesh(const std::string& path) {
    const auto normalizedPath = std::filesystem::absolute(std::filesystem::path(path)).string();
    auto it = meshCache_.find(normalizedPath);
    if (it != meshCache_.end()) {
        return it->second;
    }

    MeshData mesh = AssetLoader::loadMesh(normalizedPath);
    auto [insertIt, inserted] = meshCache_.emplace(normalizedPath, std::move(mesh));
    return insertIt->second;
}

bool AssetManager::hasMesh(const std::string& path) const {
    const auto normalizedPath = std::filesystem::absolute(std::filesystem::path(path)).string();
    return meshCache_.find(normalizedPath) != meshCache_.end();
}

const MeshData& AssetManager::getMesh(const std::string& path) const {
    const auto normalizedPath = std::filesystem::absolute(std::filesystem::path(path)).string();
    auto it = meshCache_.find(normalizedPath);
    if (it == meshCache_.end()) {
        throw std::runtime_error("AssetManager: Mesh not loaded: " + path);
    }
    return it->second;
}

const TextureData& AssetManager::loadTexture(const std::string& path) {
    const auto normalizedPath = std::filesystem::absolute(std::filesystem::path(path)).string();
    auto it = textureCache_.find(normalizedPath);
    if (it != textureCache_.end()) {
        return it->second;
    }

    TextureData texture = AssetLoader::loadTexture(normalizedPath);
    auto [insertIt, inserted] = textureCache_.emplace(normalizedPath, std::move(texture));
    return insertIt->second;
}

bool AssetManager::hasTexture(const std::string& path) const {
    const auto normalizedPath = std::filesystem::absolute(std::filesystem::path(path)).string();
    return textureCache_.find(normalizedPath) != textureCache_.end();
}

const TextureData& AssetManager::getTexture(const std::string& path) const {
    const auto normalizedPath = std::filesystem::absolute(std::filesystem::path(path)).string();
    auto it = textureCache_.find(normalizedPath);
    if (it == textureCache_.end()) {
        throw std::runtime_error("AssetManager: Texture not loaded: " + path);
    }
    return it->second;
}

} // namespace ge
