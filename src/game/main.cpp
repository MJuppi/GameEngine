// Minimal game bootstrap that uses the engine API.
#include "engine/Engine.h"
#include "engine/mesh/MeshData.h"
#include "engine/mesh/ObjMeshLoader.h"
#include "engine/mesh/GltfMeshLoader.h"

#include <filesystem>
#include <iostream>

int main(int argc, char** argv) {
    using namespace ge;
    namespace fs = std::filesystem;

    MeshData mesh;
    if (argc > 1) {
        try {
            const auto ext = fs::path(argv[1]).extension();
            if (ext == ".gltf" || ext == ".glb") {
                mesh = loadGltfFile(argv[1]);
            } else {
                mesh = loadObjFile(argv[1]);
            }
        } catch (const std::exception& e) {
            std::cerr << "Failed to load model '" << argv[1] << "': " << e.what() << '\n';
            mesh = makeUnitCubeMesh();
        }
    } else {
        const fs::path defaultModel = "assets/models/SuomiKP.obj";
        if (fs::exists(defaultModel)) {
            try {
                mesh = loadObjFile(defaultModel.string());
                std::cout << "Loaded default scene: " << defaultModel.string() << '\n';
            } catch (const std::exception& e) {
                std::cerr << "Failed to load default model: " << e.what() << '\n';
                mesh = makeUnitCubeMesh();
            }
        } else {
            mesh = makeUnitCubeMesh();
        }
    }

    Engine engine(std::move(mesh));
    engine.run();
    return 0;
}
