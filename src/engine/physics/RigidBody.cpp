#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <cmath>

#include "engine/physics/RigidBody.h"
#include "engine/physics/BoxCollider.h"
#include "engine/physics/SphereCollider.h"

namespace ge {
namespace {

glm::vec3 extractScale(const glm::mat4& transform) {
    return glm::vec3(
        glm::length(glm::vec3(transform[0])),
        glm::length(glm::vec3(transform[1])),
        glm::length(glm::vec3(transform[2]))
    );
}

glm::quat extractRotation(const glm::mat4& transform) {
    const glm::vec3 scale = extractScale(transform);
    if (glm::length2(scale) <= 1e-8f) {
        return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    }

    glm::mat3 rotationMatrix(1.0f);
    rotationMatrix[0] = glm::vec3(transform[0]) / scale.x;
    rotationMatrix[1] = glm::vec3(transform[1]) / scale.y;
    rotationMatrix[2] = glm::vec3(transform[2]) / scale.z;
    return glm::quat_cast(rotationMatrix);
}
} // namespace

RigidBody::RigidBody(std::unique_ptr<Collider> collider,
                     const glm::mat4& transform,
                     const RigidBodyProps& props)
    : collider_(std::move(collider)),
      baseTransform_(transform),
      worldTransform_(transform),
      props_(props),
      inverseInertiaTensorDirty_(true) {
    state_.rotation = extractRotation(transform);
    state_.position = glm::vec3(transform[3]) + state_.rotation * props_.centerOfMassOffset;
    state_.prevPosition = state_.position;
    state_.prevRotation = state_.rotation;
    updateTransform();
}

void RigidBody::setTransform(const glm::mat4& transform) {
    baseTransform_ = transform;
    state_.rotation = extractRotation(transform);
    state_.position = glm::vec3(transform[3]) + state_.rotation * props_.centerOfMassOffset;
    state_.prevPosition = state_.position;
    state_.prevRotation = state_.rotation;
    inverseInertiaTensorDirty_ = true;
    updateTransform();
}

glm::vec3 RigidBody::getLocalScale() const {
    if (baseTransform_ == glm::mat4(1.0f)) {
        return glm::vec3(1.0f);
    }
    return extractScale(baseTransform_);
}

void RigidBody::updateTransform() {
    const glm::vec3 scale = getLocalScale();
    glm::mat4 transform = glm::mat4_cast(state_.rotation);
    transform[0] *= scale.x;
    transform[1] *= scale.y;
    transform[2] *= scale.z;

    const glm::vec3 shapeOrigin = state_.position - state_.rotation * props_.centerOfMassOffset;
    transform[3] = glm::vec4(shapeOrigin, 1.0f);
    worldTransform_ = transform;
}

void RigidBody::updateInertiaTensor() const {
    if (!inverseInertiaTensorDirty_) {
        return;
    }

    const float mass = props_.mass;
    if (mass <= 0.0f || props_.isKinematic) {
        inverseInertiaTensor_ = glm::mat3(0.0f);
        inverseInertiaTensorDirty_ = false;
        return;
    }

    const glm::vec3 scale = getLocalScale();
    glm::vec3 inertia(1.0f);

    if (collider_->getType() == ColliderType::Box) {
        const auto& box = static_cast<const BoxCollider&>(*collider_);
        const glm::vec3 h = box.getHalfExtents() * scale;
        inertia.x = (1.0f / 3.0f) * mass * (h.y * h.y + h.z * h.z);
        inertia.y = (1.0f / 3.0f) * mass * (h.x * h.x + h.z * h.z);
        inertia.z = (1.0f / 3.0f) * mass * (h.x * h.x + h.y * h.y);
    } else if (collider_->getType() == ColliderType::Sphere) {
        const auto& sphere = static_cast<const SphereCollider&>(*collider_);
        const float radius = sphere.getRadius() * std::max({scale.x, scale.y, scale.z});
        const float moment = (2.0f / 5.0f) * mass * radius * radius;
        inertia = glm::vec3(moment);
    }

    inverseInertiaTensor_ = glm::mat3(1.0f);
    inverseInertiaTensor_[0][0] = 1.0f / std::max(inertia.x, 1e-6f);
    inverseInertiaTensor_[1][1] = 1.0f / std::max(inertia.y, 1e-6f);
    inverseInertiaTensor_[2][2] = 1.0f / std::max(inertia.z, 1e-6f);

    const glm::mat3 rotation = glm::mat3_cast(state_.rotation);
    inverseInertiaTensor_ = rotation * inverseInertiaTensor_ * glm::transpose(rotation);
    inverseInertiaTensorDirty_ = false;
}

glm::mat4 RigidBody::getInterpolatedTransform(float alpha) const {
    const glm::vec3 position = glm::mix(state_.prevPosition, state_.position, alpha);
    const glm::quat rotation = glm::slerp(state_.prevRotation, state_.rotation, alpha);

    glm::mat4 transform = glm::mat4_cast(rotation);
    transform[3] = glm::vec4(position, 1.0f);

    const glm::vec3 scale = getLocalScale();
    transform[0] *= scale.x;
    transform[1] *= scale.y;
    transform[2] *= scale.z;
    return transform;
}

void RigidBody::resetForces() {
    state_.totalForce = glm::vec3(0.0f);
    state_.totalTorque = glm::vec3(0.0f);
    state_.acceleration = glm::vec3(0.0f);
}

void RigidBody::applyDamping(glm::vec3& velocity, float damping, float deltaTime) {
    if (damping <= 0.0f) {
        return;
    }
    velocity *= std::exp(-damping * deltaTime);
}

void RigidBody::applyCombinedDamping(float deltaTime) {
    applyDamping(state_.velocity, props_.linearDamping, deltaTime);
    applyDamping(state_.angularVelocity, props_.angularDamping, deltaTime);

    if (glm::length2(state_.velocity) < 1e-6f) {
        state_.velocity = glm::vec3(0.0f);
    }
    if (glm::length2(state_.angularVelocity) < 1e-6f) {
        state_.angularVelocity = glm::vec3(0.0f);
    }
}

void RigidBody::integrateVelocity(float deltaTime) {
    if (props_.isKinematic) {
        return;
    }

    state_.prevPosition = state_.position;
    state_.prevRotation = state_.rotation;
    updateInertiaTensor();

    if (props_.mass > 0.0f) {
        state_.acceleration = state_.totalForce / props_.mass;
        state_.velocity += state_.acceleration * deltaTime;
    }

    applyCombinedDamping(deltaTime);

    if (props_.mass > 0.0f) {
        const glm::vec3 angularAcceleration = inverseInertiaTensor_ * state_.totalTorque;
        state_.angularVelocity += angularAcceleration * deltaTime;
    }

    if (glm::length2(state_.angularVelocity) > 1e-8f) {
        const glm::quat spin(0.0f, state_.angularVelocity);
        const glm::quat delta = state_.rotation * spin * (0.5f * deltaTime);
        state_.rotation = glm::normalize(state_.rotation + delta);
        inverseInertiaTensorDirty_ = true;
    }

    resetForces();
}

void RigidBody::integratePosition(float deltaTime) {
    if (props_.isKinematic) {
        return;
    }

    state_.position += state_.velocity * deltaTime;
    updateTransform();
}

void RigidBody::movePosition(const glm::vec3& delta) {
    if (props_.isKinematic) {
        return;
    }

    state_.position += delta;
    updateTransform();
}

} // namespace ge
