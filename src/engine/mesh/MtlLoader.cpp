#include "engine/mesh/MtlLoader.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace ge {

namespace {

void trimInPlace(std::string& s) {
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

Material& currentOrCreate(std::vector<Material>& materials) {
    if (materials.empty()) {
        materials.push_back(makeDefaultMaterial());
    }
    return materials.back();
}

} // namespace

std::vector<Material> loadMtlFile(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Could not open MTL file: " + path);
    }

    std::vector<Material> materials;
    std::string line;

    while (std::getline(file, line)) {
        trimInPlace(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }

        const std::string_view view = line;

        if (startsWith(view, "newmtl ")) {
            Material m;
            m.name = line.substr(7);
            trimInPlace(m.name);
            materials.push_back(m);
            continue;
        }

        Material& m = currentOrCreate(materials);

        if (startsWith(view, "Kd ")) {
            std::istringstream iss(line.substr(3));
            iss >> m.diffuse[0] >> m.diffuse[1] >> m.diffuse[2];
        } else if (startsWith(view, "Ks ")) {
            std::istringstream iss(line.substr(3));
            iss >> m.specular[0] >> m.specular[1] >> m.specular[2];
        } else if (startsWith(view, "Ka ")) {
            std::istringstream iss(line.substr(3));
            iss >> m.ambient[0] >> m.ambient[1] >> m.ambient[2];
        } else if (startsWith(view, "Ns ")) {
            std::istringstream iss(line.substr(3));
            iss >> m.shininess;
        } else if (startsWith(view, "d ")) {
            std::istringstream iss(line.substr(2));
            iss >> m.alpha;
        }
    }

    if (materials.empty()) {
        throw std::runtime_error("MTL file contained no materials: " + path);
    }

    return materials;
}

} // namespace ge
