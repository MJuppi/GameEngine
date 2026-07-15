#include "engine/mesh/ObjMeshLoader.h"
#include "engine/mesh/MtlLoader.h"
#include <glm/glm.hpp>
#include <filesystem>
#include <fstream>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace ge {

namespace {
void trim(std::string& s) {
    s.erase(0, s.find_first_not_of(" \t\r\n"));
    s.erase(s.find_last_not_of(" \t\r\n") + 1);
}

struct ObjVertex {
    int p = -1, t = -1, n = -1;
    bool operator==(const ObjVertex& other) const { return p == other.p && t == other.t && n == other.n; }
};

struct ObjVertexHash {
    size_t operator()(const ObjVertex& v) const {
        return ((std::hash<int>()(v.p) ^ (std::hash<int>()(v.t) << 1)) >> 1) ^ (std::hash<int>()(v.n) << 1);
    }
};

ObjVertex parseObjVertex(std::string_view s) {
    ObjVertex v;
    size_t f = s.find('/'), l = s.rfind('/');
    v.p = std::stoi(std::string(s.substr(0, f)));
    if (f != std::string_view::npos) {
        if (l > f + 1) v.t = std::stoi(std::string(s.substr(f + 1, l - f - 1)));
        if (l != std::string_view::npos) v.n = std::stoi(std::string(s.substr(l + 1)));
    }
    return v;
}
} // namespace

MeshData loadObjFile(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("No OBJ: " + path);

    std::vector<glm::vec3> p, n;
    std::vector<glm::vec2> t;
    std::unordered_map<ObjVertex, uint32_t, ObjVertexHash> cache;
    MeshData mesh;
    mesh.materials = {makeDefaultMaterial()};
    uint32_t curMatIdx = 0;
    std::string line, tag;

    while (std::getline(file, line)) {
        trim(line);
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        iss >> tag;

        if (tag == "v") { glm::vec3 v; iss >> v.x >> v.y >> v.z; p.push_back(v); }
        else if (tag == "vt") { glm::vec2 v; iss >> v.x >> v.y; t.push_back(v); }
        else if (tag == "vn") { glm::vec3 v; iss >> v.x >> v.y >> v.z; n.push_back(v); }
        else if (tag == "f") {
            std::vector<uint32_t> faceIndices;
            std::string s;
            while (iss >> s) {
                ObjVertex ov = parseObjVertex(s);
                if (ov.p < 0) ov.p += (int)p.size() + 1;
                if (ov.t < 0) ov.t += (int)t.size() + 1;
                if (ov.n < 0) ov.n += (int)n.size() + 1;

                if (cache.count(ov)) faceIndices.push_back(cache[ov]);
                else {
                    uint32_t idx = (uint32_t)mesh.vertices.size();
                    Vertex v{};
                    v.position[0] = p[ov.p - 1].x; v.position[1] = p[ov.p - 1].y; v.position[2] = p[ov.p - 1].z;
                    if (ov.t > 0) { v.texCoord[0] = t[ov.t - 1].x; v.texCoord[1] = t[ov.t - 1].y; }
                    if (ov.n > 0) { v.normal[0] = n[ov.n - 1].x; v.normal[1] = n[ov.n - 1].y; v.normal[2] = n[ov.n - 1].z; }
                    v.materialIndex = curMatIdx;
                    mesh.vertices.push_back(v);
                    cache[ov] = idx;
                    faceIndices.push_back(idx);
                }
            }
            for (size_t i = 1; i + 1 < faceIndices.size(); ++i) {
                mesh.indices.push_back(faceIndices[0]);
                mesh.indices.push_back(faceIndices[i]);
                mesh.indices.push_back(faceIndices[i + 1]);
            }
        } else if (tag == "mtllib") {
            std::string mtlName; iss >> mtlName;
            auto mtlPath = std::filesystem::path(path).parent_path() / mtlName;
            if (std::filesystem::exists(mtlPath)) mesh.materials = loadMtlFile(mtlPath.string());
        } else if (tag == "usemtl") {
            std::string mtlName; iss >> mtlName;
            for (uint32_t i = 0; i < mesh.materials.size(); ++i) if (mesh.materials[i].name == mtlName) curMatIdx = i;
        }
    }
    return mesh;
}

} // namespace ge
