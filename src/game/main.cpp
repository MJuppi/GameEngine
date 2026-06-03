// Minimal game bootstrap that uses the engine API.
#include "engine/Engine.h"
#include "engine/mesh/MeshData.h"
#include "engine/mesh/ObjMeshLoader.h"
#include "engine/mesh/GltfMeshLoader.h"

#include <filesystem>
#include <iostream>
#include <algorithm>

int main(int /*argc*/, char** /*argv*/) {
    using namespace ge;
    namespace fs = std::filesystem;

    MeshData mesh = makeUnitCubeMesh();
    const fs::path modelDir = "assets/models";
    std::vector<fs::path> modelFiles;

    if (fs::exists(modelDir) && fs::is_directory(modelDir)) {
        for (const auto& entry : fs::directory_iterator(modelDir)) {
            if (!entry.is_regular_file()) {
                continue;
            }

            const auto ext = entry.path().extension();
            if (ext == ".obj" || ext == ".gltf" || ext == ".glb") {
                modelFiles.push_back(entry.path());
            }
        }
    }

    if (!modelFiles.empty()) {
        std::sort(modelFiles.begin(), modelFiles.end());
        bool loadedModel = false;

        for (const auto& selectedModel : modelFiles) {
            try {
                const auto ext = selectedModel.extension();
                if (ext == ".gltf" || ext == ".glb") {
                    mesh = loadGltfFile(selectedModel.string());
                } else {
                    mesh = loadObjFile(selectedModel.string());
                }
                std::cout << "Loaded model: " << selectedModel.string() << '\n';
                loadedModel = true;
                break;
            } catch (const std::exception& e) {
                std::cerr << "Failed to load model '" << selectedModel.string() << "': " << e.what() << '\n';
            }
        }

        if (!loadedModel) {
            std::cerr << "All supported models in " << modelDir.string() << " failed to load. Falling back to unit cube." << '\n';
            mesh = makeUnitCubeMesh();
        } else {
            centerMesh(mesh);
            orientMeshYUpToZUp(mesh);
            flipMeshWinding(mesh);
        }
    } else {
        std::cerr << "No supported models found in " << modelDir.string() << ". Falling back to unit cube." << '\n';
    }

    Engine engine(std::move(mesh));
    engine.run();
    return 0;
}
