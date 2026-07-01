#include "engine/asset/AssetManifest.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace ge {

// Very small JSON-like parser tailored for the simple manifest format used here.
// It only extracts top-level string mappings under "meshes" and "textures".
AssetManifest AssetManifest::load(const std::filesystem::path& manifestPath) {
    if (!std::filesystem::exists(manifestPath)) {
        throw std::runtime_error("AssetManifest: manifest not found: " + manifestPath.string());
    }

    std::ifstream in(manifestPath);
    if (!in) throw std::runtime_error("AssetManifest: failed to open manifest: " + manifestPath.string());

    AssetManifest m;
    std::string line;
    std::string section;
    while (std::getline(in, line)) {
        // trim
        auto l = line;
        l.erase(0, l.find_first_not_of(" \t\r\n"));
        l.erase(l.find_last_not_of(" \t\r\n") + 1);
        if (l.empty()) continue;

        if (l.rfind("\"meshes\"", 0) == 0) { section = "meshes"; continue; }
        if (l.rfind("\"textures\"", 0) == 0) { section = "textures"; continue; }

        // match lines like "key": "value"
        auto keyStart = l.find('"');
        if (keyStart == std::string::npos) continue;
        auto keyEnd = l.find('"', keyStart + 1);
        if (keyEnd == std::string::npos) continue;
        std::string key = l.substr(keyStart + 1, keyEnd - keyStart - 1);

        auto valStart = l.find('"', keyEnd + 1);
        if (valStart == std::string::npos) continue;
        auto valEnd = l.find('"', valStart + 1);
        if (valEnd == std::string::npos) continue;
        std::string val = l.substr(valStart + 1, valEnd - valStart - 1);

        if (section == "meshes") m.meshes.emplace(key, val);
        else if (section == "textures") m.textures.emplace(key, val);
    }

    return m;
}

} // namespace ge
