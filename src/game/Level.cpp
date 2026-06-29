#include "game/Level.h"
#include "engine/asset/AssetManager.h"
#include <iostream>

namespace ge {
namespace {

constexpr const char* kDefaultMeshPath = "assets/models/TestCube.obj";

void postProcessMesh(MeshData& mesh) {
    centerMesh(mesh);
    orientMeshYUpToZUp(mesh);
    flipMeshWinding(mesh);
}

MeshData makeFallbackMesh() {
    return makeUnitCubeMesh();
}

} // namespace

Level::Level(std::string name, std::string meshPath)
    : name_(std::move(name)), meshPath_(std::move(meshPath)) {}

void Level::load(AssetManager& assetManager) {
    if (loaded_) return;

    const std::string pathToLoad = meshPath_.empty() ? kDefaultMeshPath : meshPath_;
    const bool usedDefault = meshPath_.empty();

    try {
        mesh_ = assetManager.loadMesh(pathToLoad);
        postProcessMesh(mesh_);

        if (usedDefault) {
            meshPath_ = kDefaultMeshPath;
        }

        std::cout << "Loaded level '" << name_ << "' from " << pathToLoad << '\n';
        loaded_ = true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load level '" << name_ << "' (" << pathToLoad
                  << "): " << e.what() << "\nFalling back to unit cube.\n";
        mesh_ = makeFallbackMesh();
        postProcessMesh(mesh_);
        loaded_ = true;
    }
}

void Level::unload() {
    if (!loaded_) return;
    mesh_ = MeshData{};
    loaded_ = false;
}

} // namespace ge
