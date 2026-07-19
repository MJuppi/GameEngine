#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>
#include <optional>
#include <cstdint>
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
    bool isTrigger = false;
    uint32_t collisionLayer = 0x1;
    uint32_t collisionMask = 0xFFFFFFFF;
};

struct RigidBodyState {
    glm::vec3 position{0.0f};
    glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 velocity{0.0f};
    glm::vec3 angularVelocity{0.0f};

    glm::vec3 prevPosition{0.0f};
    glm::quat prevRotation{1.0f, 0.0f, 0.0f, 0.0f};

    glm::vec3 acceleration{0.0f};
    glm::vec3 totalForce{0.0f};
    glm::vec3 totalTorque{0.0f};
};

class RigidBody {
public:
    RigidBody(std::unique_ptr<Collider> collider,
             const glm::mat4& transform,
             const RigidBodyProps& props);
    ~RigidBody() = default;

    void setTransform(const glm::mat4& transform);
    void setPosition(const glm::vec3& position) { state_.position = state_.prevPosition = position; updateTransform(); }
    void setVelocity(const glm::vec3& velocity) { state_.velocity = velocity; }
    void setAngularVelocity(const glm::vec3& angularVelocity) { state_.angularVelocity = angularVelocity; }
    void setProps(const RigidBodyProps& props) { props_ = props; inverseInertiaTensorDirty_ = true; }

    const Collider& getCollider() const { return *collider_; }
    const glm::mat4& getWorldTransform() const { return worldTransform_; }
    const glm::vec3& getPosition() const { return state_.position; }
    const glm::vec3& getVelocity() const { return state_.velocity; }
    const glm::vec3& getAngularVelocity() const { return state_.angularVelocity; }
    const RigidBodyProps& getProps() const { return props_; }
    const glm::mat3& getInverseInertiaTensor() const {
        updateInertiaTensor();
        return inverseInertiaTensor_;
    }
    std::shared_ptr<Material> getMaterial() const { return material_; }

    const std::optional<MeshData>& getMesh() const { return mesh_; }
    void setMesh(MeshData mesh) { mesh_ = std::move(mesh); }

    void addForce(const glm::vec3& force) { state_.totalForce += force; }
    void addTorque(const glm::vec3& torque) { state_.totalTorque += torque; }
    void integrate(float deltaTime);

    void updateTransform();
    void updateInertiaTensor() const;
    void setMaterial(std::shared_ptr<Material> material) { material_ = std::move(material); }

    [[nodiscard]] glm::mat4 getInterpolatedTransform(float alpha) const;

private:
    std::unique_ptr<Collider> collider_;
    glm::mat4 baseTransform_; // Stores initial scale and local offset
    glm::mat4 worldTransform_;

    RigidBodyProps props_;
    RigidBodyState state_;

    std::shared_ptr<Material> material_;
    std::optional<MeshData> mesh_;

    mutable glm::mat3 inverseInertiaTensor_;
    mutable bool inverseInertiaTensorDirty_;

    void applyDamping(glm::vec3& velocity, float damping, float deltaTime);
    void resetForces();
};

} // namespace ge
