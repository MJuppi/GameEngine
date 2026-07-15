#include "engine/asset/AssetManager.h"
#include "engine/asset/AssetLoader.h"
#include "engine/asset/AssetManifest.h"

#include <filesystem>
#include <stdexcept>

namespace ge {

const MeshData& AssetManager::loadMesh(const std::string& path) {
    std::string pathOrAlias = path;
    auto itm = manifest_.meshes.find(path);
    if (itm != manifest_.meshes.end()) {
        pathOrAlias = itm->second;
    }

    // Use AssetLoader to find the actual file on disk before caching
    const auto resolvedPath = AssetLoader::resolveAssetPath(pathOrAlias).string();

    auto it = meshCache_.find(resolvedPath);
    if (it != meshCache_.end()) {
        return *it->second;
    }

    auto meshPtr = std::make_shared<MeshData>(AssetLoader::loadMesh(resolvedPath));
    auto [insertIt, inserted] = meshCache_.emplace(resolvedPath, meshPtr);
    return *insertIt->second;
}

bool AssetManager::hasMesh(const std::string& path) const {
    try {
        const auto resolvedPath = AssetLoader::resolveAssetPath(path).string();
        return meshCache_.find(resolvedPath) != meshCache_.end();
    } catch (...) {
        return false;
    }
}

const MeshData& AssetManager::getMesh(const std::string& path) const {
    const auto resolvedPath = AssetLoader::resolveAssetPath(path).string();
    auto it = meshCache_.find(resolvedPath);
    if (it == meshCache_.end()) {
        throw std::runtime_error("AssetManager: Mesh not loaded: " + path);
    }
    return *it->second;
}

const TextureData& AssetManager::loadTexture(const std::string& path) {
    std::string pathOrAlias = path;
    auto itt = manifest_.textures.find(path);
    if (itt != manifest_.textures.end()) {
        pathOrAlias = itt->second;
    }

    const auto resolvedPath = AssetLoader::resolveAssetPath(pathOrAlias).string();

    auto it = textureCache_.find(resolvedPath);
    if (it != textureCache_.end()) {
        return *it->second;
    }

    auto texPtr = std::make_shared<TextureData>(AssetLoader::loadTexture(resolvedPath));
    auto [insertIt, inserted] = textureCache_.emplace(resolvedPath, texPtr);
    return *insertIt->second;
}

bool AssetManager::hasTexture(const std::string& path) const {
    try {
        const auto resolvedPath = AssetLoader::resolveAssetPath(path).string();
        return textureCache_.find(resolvedPath) != textureCache_.end();
    } catch (...) {
        return false;
    }
}

const TextureData& AssetManager::getTexture(const std::string& path) const {
    const auto resolvedPath = AssetLoader::resolveAssetPath(path).string();
    auto it = textureCache_.find(resolvedPath);
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
        std::string pathOrAlias = path;
        auto itm = manifest_.meshes.find(path);
        if (itm != manifest_.meshes.end()) {
            pathOrAlias = itm->second;
        }
        const auto resolvedPath = AssetLoader::resolveAssetPath(pathOrAlias).string();

        // Check cache again inside async context
        {
            auto it = meshCache_.find(resolvedPath);
            if (it != meshCache_.end()) return it->second;
        }
        auto meshPtr = std::make_shared<MeshData>(AssetLoader::loadMesh(resolvedPath));
        meshCache_.emplace(resolvedPath, meshPtr);
        return meshPtr;
    });
}

std::future<std::shared_ptr<TextureData>> AssetManager::loadTextureAsync(const std::string& path) {
    return std::async(std::launch::async, [this, path]() {
        std::string pathOrAlias = path;
        auto itt = manifest_.textures.find(path);
        if (itt != manifest_.textures.end()) {
            pathOrAlias = itt->second;
        }
        const auto resolvedPath = AssetLoader::resolveAssetPath(pathOrAlias).string();

        {
            auto it = textureCache_.find(resolvedPath);
            if (it != textureCache_.end()) return it->second;
        }
        auto texPtr = std::make_shared<TextureData>(AssetLoader::loadTexture(resolvedPath));
        textureCache_.emplace(resolvedPath, texPtr);
        return texPtr;
    });
}

} // namespace ge
