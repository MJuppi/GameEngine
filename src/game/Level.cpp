#include "game/Level.h"
#include "engine/mesh/MeshData.h"

#include <iostream>

namespace ge {

namespace {

constexpr const char* kDefaultMeshPath = "assets/models/SuomiKP.obj";

void postProcessMesh(MeshData& mesh) {
    centerMesh(mesh);
    orientMeshYUpToZUp(mesh);
    flipMeshWinding(mesh);
}

MeshData makeFallbackMesh() {
    return makeUnitCubeMesh();
}

} // namespace

Level::Level(const std::string& name, const std::string& meshPath)
    : name_(name), meshPath_(meshPath), loaded_(false) {
}

void Level::load() {
    if (loaded_) {
        return;
    }

    const std::string pathToLoad = meshPath_.empty() ? kDefaultMeshPath : meshPath_;
    const bool usedDefaultPath = meshPath_.empty();

    try {
        const auto& loadedMesh = assetManager_.loadMesh(pathToLoad);
        mesh_ = loadedMesh;
        postProcessMesh(mesh_);

        if (usedDefaultPath) {
            meshPath_ = kDefaultMeshPath;
        }

        std::cout << "Loaded level '" << name_ << "' with mesh: " << pathToLoad << '\n';
        loaded_ = true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load level '" << name_ << "' from '" << pathToLoad
                  << "': " << e.what() << '\n';
        std::cerr << "Falling back to built-in default level for '" << name_ << "'.\n";

        mesh_ = makeFallbackMesh();
        postProcessMesh(mesh_);
        loaded_ = true;
    }
}

void Level::unload() {
    if (!loaded_) {
        return;
    }
    
    mesh_ = MeshData();
    loaded_ = false;
}

}  // namespace ge
