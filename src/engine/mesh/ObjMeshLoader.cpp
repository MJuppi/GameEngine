#include "engine/mesh/ObjMeshLoader.h"

#include "engine/mesh/Material.h"
#include "engine/mesh/MtlLoader.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace ge {

namespace {

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;
};

int resolveIndex(int rawIndex, size_t count) {
    // Convert OBJ's 1-based or negative indices into zero-based indices.
    if (rawIndex > 0) {
        return rawIndex - 1;
    }
    if (rawIndex < 0) {
        return static_cast<int>(count) + rawIndex;
    }
    return -1;
}

void trimInPlace(std::string& s) {
    // Trim leading/trailing whitespace and CR/LF from a line.
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) {
        s.erase(s.begin());
    }
    while (!s.empty() &&
           (s.back() == ' ' || s.back() == '\t' || s.back() == '\r' || s.back() == '\n')) {
        s.pop_back();
    }
}

bool startsWith(std::string_view line, std::string_view prefix) {
    return line.size() >= prefix.size() && line.compare(0, prefix.size(), prefix) == 0;
}

std::vector<std::string> splitByChar(const std::string& s, char delim) {
    // Split a string by a single character delimiter (used for corner tokens like "v/vt/vn").
    std::vector<std::string> parts;
    std::string token;
    for (char c : s) {
        if (c == delim) {
            parts.push_back(token);
            token.clear();
        } else {
            token.push_back(c);
        }
    }
    parts.push_back(token);
    return parts;
}

uint32_t resolveMaterialIndex(
    const std::string& name,
    const std::vector<Material>& materials,
    std::unordered_map<std::string, uint32_t>& cache)
{
    // Map material name to its index in the materials vector (with simple caching).
    const auto found = cache.find(name);
    if (found != cache.end()) {
        return found->second;
    }

    for (uint32_t i = 0; i < materials.size(); ++i) {
        if (materials[i].name == name) {
            cache[name] = i;
            return i;
        }
    }

    return 0;
}

Vertex makeVertexFromCorner(
    const std::string& cornerToken,
    const std::vector<Vec3>& positions,
    const std::vector<Vec3>& normals,
    const std::vector<Vec2>& texCoords,
    uint32_t materialIndex)
{
    // Parse a face corner token like "1/2/3" and construct a Vertex with position, normal and UV.
    const std::vector<std::string> parts = splitByChar(cornerToken, '/');
    if (parts.empty() || parts[0].empty()) {
        throw std::runtime_error("OBJ face corner is missing a vertex index");
    }

    const int rawV = std::stoi(parts[0]);
    const int vi = resolveIndex(rawV, positions.size());
    if (vi < 0 || vi >= static_cast<int>(positions.size())) {
        throw std::runtime_error("OBJ face references an invalid vertex index");
    }

    Vertex vtx{};
    vtx.position[0] = positions[static_cast<size_t>(vi)].x;
    vtx.position[1] = positions[static_cast<size_t>(vi)].y;
    vtx.position[2] = positions[static_cast<size_t>(vi)].z;
    vtx.materialIndex = materialIndex;

    int rawVt = 0;
    if (parts.size() >= 2 && !parts[1].empty()) {
        rawVt = std::stoi(parts[1]);
    }

    if (rawVt != 0) {
        const int ti = resolveIndex(rawVt, texCoords.size());
        if (ti >= 0 && ti < static_cast<int>(texCoords.size())) {
            vtx.texCoord[0] = texCoords[static_cast<size_t>(ti)].x;
            vtx.texCoord[1] = texCoords[static_cast<size_t>(ti)].y;
        }
    }

    int rawVn = 0;
    if (parts.size() >= 3 && !parts[2].empty()) {
        rawVn = std::stoi(parts[2]);
    }

    if (rawVn != 0) {
        const int ni = resolveIndex(rawVn, normals.size());
        if (ni >= 0 && ni < static_cast<int>(normals.size())) {
            const Vec3& n = normals[static_cast<size_t>(ni)];
            vtx.normal[0] = n.x;
            vtx.normal[1] = n.y;
            vtx.normal[2] = n.z;
        } else {
            vtx.normal[0] = 0.0f;
            vtx.normal[1] = 0.0f;
            vtx.normal[2] = 1.0f;
        }
    } else {
        vtx.normal[0] = 0.0f;
        vtx.normal[1] = 0.0f;
        vtx.normal[2] = 1.0f;
    }

    return vtx;
}

} // namespace

MeshData loadObjFile(const std::string& path) {
    // Load a Wavefront OBJ file, its optional MTL library and convert to MeshData.
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Could not open OBJ file: " + path);
    }

    const std::filesystem::path objPath(path);
    const std::filesystem::path objDir = objPath.parent_path();

    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
    std::vector<Vec2> texCoords;
    MeshData mesh;
    mesh.materials = {makeDefaultMaterial("default")};

    std::string mtlLibraryPath;
    std::string currentMaterialName = mesh.materials[0].name;
    std::unordered_map<std::string, uint32_t> materialIndexCache{{currentMaterialName, 0}};
    uint32_t currentMaterialIndex = 0;
    bool mtlLoaded = false;

    std::string line;
    while (std::getline(file, line)) {
        trimInPlace(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }

        const std::string_view view = line;

        if (startsWith(view, "mtllib ")) {
            std::string mtlFileName = line.substr(7);
            trimInPlace(mtlFileName);
            mtlLibraryPath = (objDir / mtlFileName).string();
            continue;
        }

        if (startsWith(view, "usemtl ")) {
            if (!mtlLoaded && !mtlLibraryPath.empty()) {
                mesh.materials = loadMtlFile(mtlLibraryPath);
                mtlLoaded = true;
                materialIndexCache.clear();
            }

            currentMaterialName = line.substr(7);
            trimInPlace(currentMaterialName);
            currentMaterialIndex = resolveMaterialIndex(currentMaterialName, mesh.materials, materialIndexCache);
            continue;
        }

        if (startsWith(view, "v ")) {
            std::istringstream iss(line.substr(2));
            Vec3 p{};
            iss >> p.x >> p.y >> p.z;
            positions.push_back(p);
            continue;
        }

        if (startsWith(view, "vn ")) {
            std::istringstream iss(line.substr(3));
            Vec3 n{};
            iss >> n.x >> n.y >> n.z;
            normals.push_back(n);
            continue;
        }

        if (startsWith(view, "vt ")) {
            std::istringstream iss(line.substr(3));
            Vec2 t{};
            iss >> t.x >> t.y;
            texCoords.push_back(t);
            continue;
        }

        if (startsWith(view, "f ")) {
            if (!mtlLoaded && !mtlLibraryPath.empty()) {
                mesh.materials = loadMtlFile(mtlLibraryPath);
                mtlLoaded = true;
                materialIndexCache.clear();
                currentMaterialIndex =
                    resolveMaterialIndex(currentMaterialName, mesh.materials, materialIndexCache);
            }

            std::istringstream iss(line.substr(2));
            std::vector<std::string> corners;
            std::string corner;
            while (iss >> corner) {
                corners.push_back(corner);
            }
            if (corners.size() < 3) {
                throw std::runtime_error("OBJ face has fewer than 3 corners");
            }

            const uint32_t baseIndex = static_cast<uint32_t>(mesh.vertices.size());
            for (const std::string& c : corners) {
                mesh.vertices.push_back(
                    makeVertexFromCorner(c, positions, normals, texCoords, currentMaterialIndex));
            }

            for (size_t i = 1; i + 1 < corners.size(); ++i) {
                mesh.indices.push_back(baseIndex);
                mesh.indices.push_back(static_cast<uint32_t>(baseIndex + i));
                mesh.indices.push_back(static_cast<uint32_t>(baseIndex + i + 1));
            }
        }
    }

    if (mesh.vertices.empty() || mesh.indices.empty()) {
        throw std::runtime_error("OBJ file contained no drawable faces: " + path);
    }

    if (mesh.materials.size() > kMaxGpuMaterials) {
        throw std::runtime_error("Too many materials (max " + std::to_string(kMaxGpuMaterials) + ")");
    }

    return mesh;
}

} // namespace ge
