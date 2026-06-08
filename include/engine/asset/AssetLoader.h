#pragma once

#include "engine/asset/TextureData.h"
#include "engine/mesh/MeshData.h"

#include <filesystem>
#include <string>

namespace ge {

class AssetLoader {
public:
    AssetLoader() = delete;
    ~AssetLoader() = delete;

    AssetLoader(const AssetLoader&) = delete;
    AssetLoader& operator=(const AssetLoader&) = delete;

    [[nodiscard]] static MeshData loadMesh(const std::string& path);
    [[nodiscard]] static TextureData loadTexture(const std::string& path);

private:
    [[nodiscard]] static std::string getExtension(const std::filesystem::path& path);
};

} // namespace ge
