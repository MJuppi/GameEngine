#include "engine/asset/AssetManager.h"
#include "engine/asset/AssetLoader.h"
#include "engine/asset/AssetManifest.h"

#include <filesystem>
#include <stdexcept>

namespace ge {

const MeshData& AssetManager::loadMesh(const std::string& path) {
    std::string resolvedPath = path;
    auto itm = manifest_.meshes.find(path);
    if (itm != manifest_.meshes.end()) {
        resolvedPath = itm->second;
    }

    const auto normalizedPath = std::filesystem::absolute(std::filesystem::path(resolvedPath)).string();
    auto it = meshCache_.find(normalizedPath);
    if (it != meshCache_.end()) {
        return *it->second;
    }

    auto meshPtr = std::make_shared<MeshData>(AssetLoader::loadMesh(normalizedPath));
    auto [insertIt, inserted] = meshCache_.emplace(normalizedPath, meshPtr);
    return *insertIt->second;
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
    return *it->second;
}

const TextureData& AssetManager::loadTexture(const std::string& path) {
    std::string resolvedPath = path;
    auto itt = manifest_.textures.find(path);
    if (itt != manifest_.textures.end()) {
        resolvedPath = itt->second;
    }

    const auto normalizedPath = std::filesystem::absolute(std::filesystem::path(resolvedPath)).string();
    auto it = textureCache_.find(normalizedPath);
    if (it != textureCache_.end()) {
        return *it->second;
    }

    auto texPtr = std::make_shared<TextureData>(AssetLoader::loadTexture(normalizedPath));
    auto [insertIt, inserted] = textureCache_.emplace(normalizedPath, texPtr);
    return *insertIt->second;
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
    return *it->second;
}

void AssetManager::loadManifest(const std::filesystem::path& manifestPath) {
    manifest_ = AssetManifest::load(manifestPath);
}

std::future<std::shared_ptr<MeshData>> AssetManager::loadMeshAsync(const std::string& path) {
    return std::async(std::launch::async, [this, path]() {
        const auto normalizedPath = std::filesystem::absolute(std::filesystem::path(path)).string();
        // Check cache again inside async context
        {
            auto it = meshCache_.find(normalizedPath);
            if (it != meshCache_.end()) return it->second;
        }
        auto meshPtr = std::make_shared<MeshData>(AssetLoader::loadMesh(normalizedPath));
        meshCache_.emplace(normalizedPath, meshPtr);
        return meshPtr;
    });
}

std::future<std::shared_ptr<TextureData>> AssetManager::loadTextureAsync(const std::string& path) {
    return std::async(std::launch::async, [this, path]() {
        const auto normalizedPath = std::filesystem::absolute(std::filesystem::path(path)).string();
        {
            auto it = textureCache_.find(normalizedPath);
            if (it != textureCache_.end()) return it->second;
        }
        auto texPtr = std::make_shared<TextureData>(AssetLoader::loadTexture(normalizedPath));
        textureCache_.emplace(normalizedPath, texPtr);
        return texPtr;
    });
}

} // namespace ge
