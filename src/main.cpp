// =============================================================================
// main.cpp — Program entry
// =============================================================================
// Constructs the Engine and runs the main loop until the window closes.
//
// Optional: pass a path to a Wavefront .obj file or glTF file to load your own mesh, e.g.:
//   GameEngine.exe path/to/model.obj
//   GameEngine.exe path/to/model.gltf
//   GameEngine.exe path/to/model.glb
// =============================================================================

#include "engine/Engine.h"
#include "engine/mesh/MeshData.h"
#include "engine/mesh/ObjMeshLoader.h"
#include "engine/mesh/GltfMeshLoader.h"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

/// First .obj or glTF in models/, or models/SuomiKP.obj if present (run from project root).
std::string findDefaultModelPath() {
    namespace fs = std::filesystem;

    // Prefer the new assets path
    const fs::path preferredAssets = "assets/models/SuomiKP.obj";
    if (fs::exists(preferredAssets)) {
        return preferredAssets.string();
    }

    const fs::path assetsDir = "assets/models";
    if (fs::is_directory(assetsDir)) {
        for (const auto& entry : fs::directory_iterator(assetsDir)) {
            if (entry.is_regular_file()) {
                const auto ext = entry.path().extension();
                if (ext == ".obj" || ext == ".gltf" || ext == ".glb") {
                    return entry.path().string();
                }
            }
        }
    }

    // Legacy fallback for older layouts.
    const fs::path modelsDir = "models";
    if (!fs::is_directory(modelsDir)) {
        return {};
    }

    // Return the first supported model found in the models directory.
    for (const auto& entry : fs::directory_iterator(modelsDir)) {
        if (entry.is_regular_file()) {
            const auto ext = entry.path().extension();
            if (ext == ".obj" || ext == ".gltf" || ext == ".glb") {
                return entry.path().string();
            }
        }
    }
    return {};
}

bool tryLoadObj(const std::string& path, ge::MeshData& outMesh) {
    // Attempt to load an OBJ file, orient it for the renderer, and report info.
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

bool tryLoadGltf(const std::string& path, ge::MeshData& outMesh) {
    // Attempt to load a glTF file, orient it for the renderer, and report info.
    try {
        outMesh = ge::loadGltfFile(path);
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
        std::cerr << "Could not load glTF \"" << path << "\": " << e.what() << '\n';
        return false;
    }
}

bool tryLoadModel(const std::string& path, ge::MeshData& outMesh) {
    // Try to load any supported model format based on file extension.
    namespace fs = std::filesystem;
    const auto ext = fs::path(path).extension();
    
    if (ext == ".obj") {
        return tryLoadObj(path, outMesh);
    } else if (ext == ".gltf" || ext == ".glb") {
        return tryLoadGltf(path, outMesh);
    } else {
        std::cerr << "Unknown file format: " << path << " (expected .obj, .gltf, or .glb)\n";
        return false;
    }
}

} // namespace

int main(int argc, char* argv[]) {
    try {
        // Validate loader without opening a window: GameEngine.exe --check models/foo.obj
        if (argc >= 3 && std::string(argv[1]) == "--check") {
            ge::MeshData mesh;
            if (!tryLoadModel(argv[2], mesh)) {
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
            if (!tryLoadModel(modelPath, mesh)) {
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
