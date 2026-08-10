#include "engine/physics/cannon/Material.h"

namespace ge {
namespace cannon {

Material::Material() : name("default"), friction(0.3f), restitution(0.0f) {}
Material::Material(const std::string& name_) : name(name_), friction(0.3f), restitution(0.0f) {}

ContactMaterial::ContactMaterial()
    : materials{nullptr, nullptr}, friction(0.3f), restitution(0.0f), contactEquationStiffness(1e7f), contactEquationRelaxation(4.0f) {}

ContactMaterial::ContactMaterial(const Material& materialA, const Material& materialB, float friction_, float restitution_)
    : materials{&materialA, &materialB}, friction(friction_), restitution(restitution_), contactEquationStiffness(1e7f), contactEquationRelaxation(4.0f) {}

} // namespace cannon
} // namespace ge
