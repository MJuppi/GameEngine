#pragma once

#include "engine/mesh/MeshData.h"

#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>

namespace ge {

class AssetManager {
public:
    AssetManager() = default;
    ~AssetManager() = default;

    AssetManager(const AssetManager&) = delete;
    AssetManager& operator=(const AssetManager&) = delete;

    [[nodiscard]] const MeshData& loadMesh(const std::string& path);
    [[nodiscard]] bool hasMesh(const std::string& path) const;
    [[nodiscard]] const MeshData& getMesh(const std::string& path) const;

private:
    [[nodiscard]] static std::string getExtension(const std::filesystem::path& path);

    std::unordered_map<std::string, MeshData> cache_;
};

} // namespace ge
