// =============================================================================
// main.cpp — Program entry
// =============================================================================
// Constructs the Engine and runs the main loop until the window closes.
//
// Optional: pass a path to a Wavefront .obj file to load your own mesh, e.g.:
//   GameEngine.exe path/to/model.obj
// =============================================================================

#include "engine/Engine.h"
#include "engine/mesh/MeshData.h"
#include "engine/mesh/ObjMeshLoader.h"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

/// First .obj in models/, or models/SuomiKP.obj if present (run from project root).
std::string findDefaultModelPath() {
    namespace fs = std::filesystem;

    const fs::path preferred = "models/SuomiKP.obj";
    if (fs::exists(preferred)) {
        return preferred.string();
    }

    const fs::path modelsDir = "models";
    if (!fs::is_directory(modelsDir)) {
        return {};
    }

    for (const auto& entry : fs::directory_iterator(modelsDir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".obj") {
            return entry.path().string();
        }
    }
    return {};
}

bool tryLoadObj(const std::string& path, ge::MeshData& outMesh) {
    try {
        outMesh = ge::loadObjFile(path);
        ge::orientMeshYUpToZUp(outMesh);
        ge::flipMeshWinding(outMesh);
        ge::centerMesh(outMesh);
        const ge::MeshBounds bounds = ge::computeMeshBounds(outMesh);
        std::cout << "Loaded mesh: " << path << " ("
                  << outMesh.vertices.size() << " vertices, "
                  << (outMesh.indices.size() / 3) << " triangles, "
                  << outMesh.materials.size() << " materials, radius "
                  << bounds.radius << ")\n";
        for (const auto& mat : outMesh.materials) {
            std::cout << "  - " << mat.name << " Kd(" << mat.diffuse[0] << ", "
                      << mat.diffuse[1] << ", " << mat.diffuse[2] << ")\n";
        }
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Could not load OBJ \"" << path << "\": " << e.what() << '\n';
        return false;
    }
}

} // namespace

int main(int argc, char* argv[]) {
    try {
        // Validate loader without opening a window: GameEngine.exe --check models/foo.obj
        if (argc >= 3 && std::string(argv[1]) == "--check") {
            ge::MeshData mesh;
            if (!tryLoadObj(argv[2], mesh)) {
                return 1;
            }
            return 0;
        }

        ge::MeshData mesh = ge::makeUnitCubeMesh();
        std::string modelPath;

        if (argc >= 2) {
            modelPath = argv[1];
        } else {
            modelPath = findDefaultModelPath();
            if (!modelPath.empty()) {
                std::cout << "No model argument — using " << modelPath << '\n';
            }
        }

        if (!modelPath.empty()) {
            if (!tryLoadObj(modelPath, mesh)) {
                std::cerr << "Falling back to the built-in cube.\n";
                mesh = ge::makeUnitCubeMesh();
            }
        }

        ge::Engine engine(std::move(mesh));
        engine.run();
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
