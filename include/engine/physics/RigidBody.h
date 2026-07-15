#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>
#include <optional>
#include "engine/mesh/MeshData.h"

namespace ge {

class Collider;
struct Material;

struct RigidBodyProps {
    float mass = 1.0f;
    float restitution = 0.5f;
    float friction = 0.5f;
    float linearDamping = 0.1f;
    float angularDamping = 0.1f;
    bool isKinematic = false;
    bool useGravity = true;
};

class RigidBody {
public:
    RigidBody(std::unique_ptr<Collider> collider,
             const glm::mat4& transform,
             const RigidBodyProps& props);
    ~RigidBody() = default;

    void setTransform(const glm::mat4& transform);
    void setPosition(const glm::vec3& position);
    void setVelocity(const glm::vec3& velocity);
    void setAngularVelocity(const glm::vec3& angularVelocity);
    void setProps(const RigidBodyProps& props);

    const Collider& getCollider() const { return *collider_; }
    const glm::mat4& getWorldTransform() const { return worldTransform_; }
    const glm::vec3& getPosition() const { return position_; }
    const glm::vec3& getVelocity() const { return velocity_; }
    const glm::vec3& getAngularVelocity() const { return angularVelocity_; }
    const RigidBodyProps& getProps() const { return props_; }
    const glm::mat3& getInverseInertiaTensor() const {
        updateInertiaTensor();
        return inverseInertiaTensor_;
    }
    std::shared_ptr<Material> getMaterial() const { return material_; }

    const std::optional<MeshData>& getMesh() const { return mesh_; }
    void setMesh(MeshData mesh) { mesh_ = std::move(mesh); }

    void addForce(const glm::vec3& force);
    void addTorque(const glm::vec3& torque);
    void integrate(float deltaTime);

    void updateTransform();
    void updateInertiaTensor() const;
    void setMaterial(std::shared_ptr<Material> material) { material_ = std::move(material); }

private:
    std::unique_ptr<Collider> collider_;
    glm::mat4 transform_;
    glm::mat4 worldTransform_;
    glm::vec3 position_;
    RigidBodyProps props_;
    glm::vec3 velocity_;
    glm::vec3 angularVelocity_;
    glm::vec3 acceleration_;
    glm::vec3 totalForce_;
    glm::vec3 totalTorque_;
    glm::quat rotation_;
    std::shared_ptr<Material> material_;
    std::optional<MeshData> mesh_;

    mutable glm::mat3 inverseInertiaTensor_;
    mutable bool inverseInertiaTensorDirty_;

    void applyDamping(glm::vec3& velocity, float damping, float deltaTime);
    void resetForces();
};

} // namespace ge
