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

void Level::load() {
    if (loaded_) {
        return;  // Already loaded
    }

    if (meshPath_.empty()) {
        // No mesh path provided, use default cube
        mesh_ = makeUnitCubeMesh();
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
        std::cerr << "Falling back to unit cube for level '" << name_ << "'.\n";
        mesh_ = makeUnitCubeMesh();
        loaded_ = true;
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
