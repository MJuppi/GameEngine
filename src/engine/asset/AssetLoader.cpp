#include "engine/asset/AssetLoader.h"
#include "engine/asset/TextureLoader.h"
#include "engine/mesh/GltfMeshLoader.h"
#include "engine/mesh/ObjMeshLoader.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <stdexcept>
#include <sstream>
#include <vector>

namespace ge {

MeshData AssetLoader::loadMesh(const std::string& path) {
    const std::filesystem::path filePath(path);
    const auto resolved = resolveAssetPath(filePath);
    const auto extension = getExtension(resolved);
    if (extension.empty()) {
        throw std::runtime_error("AssetLoader: Mesh path has no extension: " + resolved.string());
    }

    const auto normalizedPath = std::filesystem::absolute(resolved).string();
    if (extension == ".gltf" || extension == ".glb") {
        return loadGltfFile(normalizedPath);
    }

    if (extension == ".obj") {
        return loadObjFile(normalizedPath);
    }

    throw std::runtime_error("AssetLoader: Unsupported mesh format: " + extension + " (" + normalizedPath + ")");
}

Model AssetLoader::loadModel(const std::string& path) {
    Model model;
    model.mesh = loadMesh(path);
    model.bounds = computeMeshBounds(model.mesh);
    return model;
}

TextureData AssetLoader::loadTexture(const std::string& path) {
    const std::filesystem::path filePath(path);
    const auto resolved = resolveAssetPath(filePath);
    const auto extension = getExtension(resolved);
    if (extension.empty()) {
        throw std::runtime_error("AssetLoader: Texture path has no extension: " + resolved.string());
    }

    const auto normalizedPath = std::filesystem::absolute(resolved).string();
    return loadTextureFile(normalizedPath);
}

std::string AssetLoader::getExtension(const std::filesystem::path& path) {
    auto ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return ext;
}

// Try to resolve a path by checking the given path, then searching up the tree
// for an `assets/` directory and common subfolders. Returns an absolute path
// if the file exists, otherwise throws with helpful diagnostic info.
std::filesystem::path AssetLoader::resolveAssetPath(const std::filesystem::path& input) {
    // If absolute and exists, return immediately.
    if (input.is_absolute()) {
        if (std::filesystem::exists(input)) return std::filesystem::absolute(input);
        std::ostringstream msg;
        msg << "AssetLoader: file not found: " << input.string();
        throw std::runtime_error(msg.str());
    }

    // If relative and exists as-is, return.
    if (std::filesystem::exists(input)) return std::filesystem::absolute(input);

    // Search upward for an `assets/` root and try common subfolders.
    std::vector<std::string> subdirs = {"", "models", "textures", "shaders", "fonts", "audio"};
    std::filesystem::path cur = std::filesystem::current_path();
    while (true) {
        for (const auto& sd : subdirs) {
            std::filesystem::path candidate = cur / "assets" / sd / input;
            if (std::filesystem::exists(candidate)) return std::filesystem::absolute(candidate);
        }

        if (cur == cur.root_path()) break;
        cur = cur.parent_path();
    }

    // As a last attempt, try under current working dir's assets without subdir.
    {
        std::filesystem::path candidate = std::filesystem::current_path() / "assets" / input;
        if (std::filesystem::exists(candidate)) return std::filesystem::absolute(candidate);
    }

    std::ostringstream tried;
    tried << "AssetLoader: could not locate '" << input.string() << "'. Searched: current path and nearest assets/ subfolders.";
    throw std::runtime_error(tried.str());
}

} // namespace ge
