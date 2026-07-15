#include "engine/mesh/MtlLoader.h"
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace ge {

namespace {
void trim(std::string& s) {
    s.erase(0, s.find_first_not_of(" \t\r\n"));
    s.erase(s.find_last_not_of(" \t\r\n") + 1);
}
} // namespace

std::vector<Material> loadMtlFile(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("No MTL: " + path);

    std::vector<Material> materials;
    std::string line, tag;

    while (std::getline(file, line)) {
        trim(line);
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        iss >> tag;

        if (tag == "newmtl") {
            Material m;
            iss >> m.name;
            materials.push_back(m);
        } else if (!materials.empty()) {
            Material& m = materials.back();
            if (tag == "Kd") iss >> m.diffuse.x >> m.diffuse.y >> m.diffuse.z;
            else if (tag == "Ks") iss >> m.specular.x >> m.specular.y >> m.specular.z;
            else if (tag == "Ns") iss >> m.shininess;
            else if (tag == "d") iss >> m.alpha;
            else if (tag == "map_Kd") { iss >> m.texturePath; trim(m.texturePath); }
        }
    }
    if (materials.empty()) throw std::runtime_error("Empty MTL: " + path);
    return materials;
}

} // namespace ge
