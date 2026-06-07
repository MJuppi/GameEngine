#include "game/Level.h"
#include "engine/mesh/MeshData.h"
#include "engine/mesh/ObjMeshLoader.h"
#include "engine/mesh/GltfMeshLoader.h"

#include <iostream>
#include <filesystem>

namespace ge {

Level::Level(const std::string& name, const std::string& meshPath)
    : name_(name), meshPath_(meshPath), loaded_(false) {
}

static std::string findModelInAssetDirectory() {
    const std::filesystem::path modelDir = "assets/models";
    if (!std::filesystem::exists(modelDir) || !std::filesystem::is_directory(modelDir)) {
        return {};
    }

    for (const auto& entry : std::filesystem::directory_iterator(modelDir)) {
        if (!entry.is_regular_file()) {
            continue;
        }

        const auto ext = entry.path().extension();
        if (ext == ".obj" || ext == ".gltf" || ext == ".glb") {
            return entry.path().string();
        }
    }

    return {};
}

void Level::load() {
    if (loaded_) {
        return;  // Already loaded
    }

    if (meshPath_.empty()) {
        meshPath_ = findModelInAssetDirectory();
    }

    if (meshPath_.empty()) {
        // No mesh path found — build a small example level composed of
        // a unit cube (as a placeholder), a large inverted skybox cube and
        // a ground plane. Assign distinct materials for each part.
        MeshData cube = makeUnitCubeMesh();
        MeshData sky = makeSkyboxMesh(1, 100.0f);
        MeshData ground = makeGroundPlaneMesh(2, 50.0f, -1.5f);

        // Compose into a single MeshData by concatenating vertices/indices
        // and preserving material indices used above.
        mesh_.materials.clear();
        mesh_.materials.push_back(makeDefaultMaterial("default"));
        mesh_.materials.push_back(makeDefaultMaterial("sky"));
        mesh_.materials.push_back(makeDefaultMaterial("ground"));

        // Configure material colors: default (neutral gray), sky (light blue),
        // ground (earthy green).
        mesh_.materials[0].diffuse[0] = 0.8f; // default diffuse (R)
        mesh_.materials[0].diffuse[1] = 0.8f; // default diffuse (G)
        mesh_.materials[0].diffuse[2] = 0.8f; // default diffuse (B)

        mesh_.materials[1].diffuse[0] = 0.53f; // sky blue RGB
        mesh_.materials[1].diffuse[1] = 0.81f;
        mesh_.materials[1].diffuse[2] = 0.92f;

        mesh_.materials[2].diffuse[0] = 0.30f; // ground / grass-ish
        mesh_.materials[2].diffuse[1] = 0.45f;
        mesh_.materials[2].diffuse[2] = 0.20f;

        // Start with cube
        mesh_.vertices = cube.vertices;
        mesh_.indices = cube.indices;

        // Append sky (adjust indices)
        uint32_t base = static_cast<uint32_t>(mesh_.vertices.size());
        for (const Vertex& v : sky.vertices) {
            mesh_.vertices.push_back(v);
        }
        for (uint32_t idx : sky.indices) {
            mesh_.indices.push_back(base + idx);
        }

        // Append ground
        base = static_cast<uint32_t>(mesh_.vertices.size());
        for (const Vertex& v : ground.vertices) {
            mesh_.vertices.push_back(v);
        }
        for (uint32_t idx : ground.indices) {
            mesh_.indices.push_back(base + idx);
        }

        // Ensure mesh transforms and winding are consistent
        centerMesh(mesh_);
        orientMeshYUpToZUp(mesh_);
        flipMeshWinding(mesh_);

        loaded_ = true;
        return;
    }

    try {
        const auto ext = std::filesystem::path(meshPath_).extension();
        
        if (ext == ".gltf" || ext == ".glb") {
            mesh_ = loadGltfFile(meshPath_);
        } else if (ext == ".obj") {
            mesh_ = loadObjFile(meshPath_);
        } else {
            throw std::runtime_error("Unsupported mesh format: " + ext.string());
        }

        // Process the loaded mesh
        centerMesh(mesh_);
        orientMeshYUpToZUp(mesh_);
        flipMeshWinding(mesh_);
        
        std::cout << "Loaded level '" << name_ << "' with mesh: " << meshPath_ << '\n';
        loaded_ = true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load level '" << name_ << "': " << e.what() << '\n';
        std::cerr << "Falling back to built-in default level for '" << name_ << "'.\n";
        meshPath_.clear();
        loaded_ = false;
        load();
    }
}

void Level::unload() {
    if (!loaded_) {
        return;
    }
    
    // Clear mesh data
    mesh_ = MeshData();
    loaded_ = false;
}

}  // namespace ge
