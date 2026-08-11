#pragma once

#include <string>

namespace ge {
namespace cannon {

class Material {
public:
    Material();
    explicit Material(const std::string& name);

    int id;
    std::string name;
    float friction;
    float restitution;

    static int idCounter;
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
    float frictionEquationStiffness;
    float frictionEquationRelaxation;
};

} // namespace cannon
} // namespace ge
