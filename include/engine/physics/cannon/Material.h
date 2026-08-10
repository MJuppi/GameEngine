#pragma once

#include <string>

namespace ge {
namespace cannon {

class Material {
public:
    Material();
    explicit Material(const std::string& name);

    std::string name;
    float friction;
    float restitution;
};

class ContactMaterial {
public:
    ContactMaterial();
    ContactMaterial(const Material& materialA, const Material& materialB, float friction, float restitution);

    const Material* materials[2];
    float friction;
    float restitution;
    float contactEquationStiffness;
    float contactEquationRelaxation;
};

} // namespace cannon
} // namespace ge
