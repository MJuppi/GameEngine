#include "game/Level.h"
#include "engine/asset/AssetManager.h"
#include "engine/mesh/MeshData.h"
#include "engine/mesh/ObjMeshLoader.h"
#include "engine/mesh/GltfMeshLoader.h"
#include "engine/scene/SceneGraph.h"

#include <iostream>
#include <filesystem>

namespace ge {

namespace {

void buildDefaultScene(SceneGraph& sceneGraph) {
    sceneGraph.clear();

    MeshData cube = makeUnitCubeMesh();
    MeshData sky = makeSkyboxMesh(1, 100.0f);
    MeshData ground = makeGroundPlaneMesh(2, 50.0f, -1.5f);

    sceneGraph.addNode({"default_cube", std::move(cube)});
    sceneGraph.addNode({"skybox", std::move(sky)});
    sceneGraph.addNode({"ground", std::move(ground)});
}

} // namespace

Level::Level(const std::string& name, const std::string& meshPath)
    : name_(name), meshPath_(meshPath), loaded_(false) {
}

void Level::load() {
    if (loaded_) {
        return;
    }

    sceneGraph_.clear();

    try {
        if (meshPath_.empty()) {
            buildDefaultScene(sceneGraph_);
        } else {
            const std::filesystem::path meshFile(meshPath_);
            if (!std::filesystem::exists(meshFile)) {
                throw std::runtime_error("Mesh file not found: " + meshPath_);
            }

            const auto& loadedMesh = assetManager_.loadMesh(meshPath_);
            sceneGraph_.addNode({"level_mesh", loadedMesh});
        }

        mesh_ = sceneGraph_.buildMesh();
        centerMesh(mesh_);
        orientMeshYUpToZUp(mesh_);
        flipMeshWinding(mesh_);

        std::cout << "Loaded level '" << name_ << "' with mesh: "
                  << (meshPath_.empty() ? "<built-in default>" : meshPath_) << '\n';
        loaded_ = true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load level '" << name_ << "': " << e.what() << '\n';
        std::cerr << "Falling back to built-in default level for '" << name_ << "'.\n";
        meshPath_.clear();
        sceneGraph_.clear();
        buildDefaultScene(sceneGraph_);
        mesh_ = sceneGraph_.buildMesh();
        centerMesh(mesh_);
        orientMeshYUpToZUp(mesh_);
        flipMeshWinding(mesh_);
        loaded_ = true;
    }
}

void Level::unload() {
    if (!loaded_) {
        return;
    }
    
    sceneGraph_.clear();
    mesh_ = MeshData();
    loaded_ = false;
}

}  // namespace ge
