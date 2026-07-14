#include "engine/physics/RigidBody.h"
#include "engine/physics/Collider.h"
#include "engine/physics/BoxCollider.h"
#include "engine/physics/SphereCollider.h"
#include <glm/gtc/quaternion.hpp>

namespace ge {

RigidBody::RigidBody(std::unique_ptr<Collider> collider,
                     const glm::mat4& transform,
                     const RigidBodyProps& props)
    : collider_(std::move(collider)),
      transform_(transform),
      worldTransform_(transform),
      position_(glm::vec3(transform[3])),
      props_(props),
      velocity_(0.0f),
      angularVelocity_(0.0f),
      acceleration_(0.0f),
      totalForce_(0.0f),
      totalTorque_(0.0f),
      rotation_(glm::quat_cast(transform)),
      inverseInertiaTensorDirty_(true)
{
    updateTransform();
}

void RigidBody::setTransform(const glm::mat4& transform) {
    transform_ = transform;
    position_ = glm::vec3(transform[3]);
    rotation_ = glm::quat_cast(transform);
    inverseInertiaTensorDirty_ = true;
    updateTransform();
}

void RigidBody::setPosition(const glm::vec3& position) {
    position_ = position;
    transform_[3] = glm::vec4(position, 1.0f);
    updateTransform();
}

void RigidBody::setVelocity(const glm::vec3& velocity) {
    velocity_ = velocity;
}

void RigidBody::setAngularVelocity(const glm::vec3& angularVelocity) {
    angularVelocity_ = angularVelocity;
}

void RigidBody::setProps(const RigidBodyProps& props) {
    props_ = props;
    inverseInertiaTensorDirty_ = true;
}

void RigidBody::addForce(const glm::vec3& force) {
    totalForce_ += force;
}

void RigidBody::addTorque(const glm::vec3& torque) {
    totalTorque_ += torque;
}

void RigidBody::updateTransform() {
    // Build the world transform from position and rotation
    worldTransform_ = glm::mat4_cast(rotation_);
    worldTransform_[3] = glm::vec4(position_, 1.0f);

    // Apply scale if needed (extracted from the transform matrix)
    if (transform_ != glm::mat4(1.0f)) {
        glm::vec3 scale = glm::vec3(
            glm::length(glm::vec3(transform_[0])),
            glm::length(glm::vec3(transform_[1])),
            glm::length(glm::vec3(transform_[2]))
        );
        worldTransform_ = glm::scale(worldTransform_, scale);
    }
}

void RigidBody::updateInertiaTensor() const {
    if (!inverseInertiaTensorDirty_) return;

    float mass = props_.mass;
    if (mass <= 0.0f || props_.isKinematic) {
        inverseInertiaTensor_ = glm::mat3(0.0f);
        inverseInertiaTensorDirty_ = false;
        return;
    }

    glm::vec3 inertia(1.0f);

    if (collider_->getType() == "Box") {
        const auto& box = static_cast<const BoxCollider&>(*collider_);
        glm::vec3 h = box.getHalfExtents();
        // I = 1/12 * m * (side^2 + side^2)
        // side = 2 * halfExtent
        // I = 1/12 * m * (4*h^2 + 4*h^2) = 1/3 * m * (h^2 + h^2)
        inertia.x = (1.0f / 3.0f) * mass * (h.y * h.y + h.z * h.z);
        inertia.y = (1.0f / 3.0f) * mass * (h.x * h.x + h.z * h.z);
        inertia.z = (1.0f / 3.0f) * mass * (h.x * h.x + h.y * h.y);
    } else if (collider_->getType() == "Sphere") {
        const auto& sphere = static_cast<const SphereCollider&>(*collider_);
        float r = sphere.getRadius();
        float val = (2.0f / 5.0f) * mass * r * r;
        inertia = glm::vec3(val);
    }

    inverseInertiaTensor_ = glm::mat3(1.0f);
    inverseInertiaTensor_[0][0] = 1.0f / inertia.x;
    inverseInertiaTensor_[1][1] = 1.0f / inertia.y;
    inverseInertiaTensor_[2][2] = 1.0f / inertia.z;

    // Transform inertia tensor to world space: I_inv_world = R * I_inv_local * R^T
    glm::mat3 rotation = glm::mat3_cast(rotation_);
    inverseInertiaTensor_ = rotation * inverseInertiaTensor_ * glm::transpose(rotation);

    inverseInertiaTensorDirty_ = false;
}

void RigidBody::resetForces() {
    totalForce_ = glm::vec3(0.0f);
    totalTorque_ = glm::vec3(0.0f);
    acceleration_ = glm::vec3(0.0f);
}

void RigidBody::applyDamping(glm::vec3& velocity, float damping, float deltaTime) {
    velocity *= glm::pow(1.0f - damping, deltaTime);
}

void RigidBody::integrate(float deltaTime) {
    if (props_.isKinematic) {
        // Kinematic bodies are controlled externally
        return;
    }

    // Update inertia tensor if needed
    updateInertiaTensor();

    // Calculate acceleration from total force
    if (props_.mass > 0.0f) {
        acceleration_ = totalForce_ / props_.mass;
    }

    // Update velocity
    velocity_ += acceleration_ * deltaTime;

    applyDamping(velocity_, props_.linearDamping, deltaTime);

    // Update position
    position_ += velocity_ * deltaTime;

    // Update angular velocity
    if (props_.mass > 0.0f) {
        // angularAcceleration = inverseInertiaTensor * totalTorque;
        glm::vec3 angularAcceleration = inverseInertiaTensor_ * totalTorque_;
        angularVelocity_ += angularAcceleration * deltaTime;
    }

    applyDamping(angularVelocity_, props_.angularDamping, deltaTime);

    // Update rotation (using quaternions)
    if (glm::length(angularVelocity_) > 0.001f) {
        glm::quat angularVelocityQuat(0.0f, angularVelocity_);
        glm::quat rotationDelta = angularVelocityQuat * (deltaTime * 0.5f);
        rotation_ = glm::normalize(rotation_ + rotationDelta * rotation_);
        inverseInertiaTensorDirty_ = true;
    }

    // Update the world transform
    updateTransform();

    // Reset forces for next frame
    resetForces();
}

} // namespace ge
