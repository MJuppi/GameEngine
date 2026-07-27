#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include "engine/physics/RigidBody.h"
#include "engine/physics/Collider.h"
#include "engine/physics/BoxCollider.h"
#include "engine/physics/SphereCollider.h"
#include <glm/gtc/quaternion.hpp>
#include <algorithm>

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
      inverseInertiaTensorDirty_(true)
{
    state_.position = glm::vec3(transform[3]);
    state_.prevPosition = state_.position;
    state_.rotation = extractRotation(transform);
    state_.prevRotation = state_.rotation;
    updateTransform();
}

void RigidBody::setTransform(const glm::mat4& transform) {
    baseTransform_ = transform;
    state_.position = glm::vec3(transform[3]);
    state_.prevPosition = state_.position;
    state_.rotation = extractRotation(transform);
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
    worldTransform_ = glm::mat4_cast(state_.rotation);
    worldTransform_[3] = glm::vec4(state_.position, 1.0f);

    const glm::vec3 scale = getLocalScale();
    if (scale != glm::vec3(1.0f)) {
        worldTransform_ = glm::scale(worldTransform_, scale);
    }
}

void RigidBody::updateInertiaTensor() const {
    if (!inverseInertiaTensorDirty_) return;

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
        const float r = sphere.getRadius() * std::max({scale.x, scale.y, scale.z});
        const float val = (2.0f / 5.0f) * mass * r * r;
        inertia = glm::vec3(val);
    }

    inverseInertiaTensor_ = glm::mat3(1.0f);
    inverseInertiaTensor_[0][0] = 1.0f / inertia.x;
    inverseInertiaTensor_[1][1] = 1.0f / inertia.y;
    inverseInertiaTensor_[2][2] = 1.0f / inertia.z;

    const glm::mat3 rotation = glm::mat3_cast(state_.rotation);
    inverseInertiaTensor_ = rotation * inverseInertiaTensor_ * glm::transpose(rotation);
    inverseInertiaTensorDirty_ = false;
}

glm::mat4 RigidBody::getInterpolatedTransform(float alpha) const {
    const glm::vec3 interpolatedPosition = glm::mix(state_.prevPosition, state_.position, alpha);
    const glm::quat interpolatedRotation = glm::slerp(state_.prevRotation, state_.rotation, alpha);

    glm::mat4 interpolatedTransform = glm::mat4_cast(interpolatedRotation);
    interpolatedTransform[3] = glm::vec4(interpolatedPosition, 1.0f);

    const glm::vec3 scale = getLocalScale();
    if (scale != glm::vec3(1.0f)) {
        interpolatedTransform = glm::scale(interpolatedTransform, scale);
    }

    return interpolatedTransform;
}

void RigidBody::resetForces() {
    state_.totalForce = glm::vec3(0.0f);
    state_.totalTorque = glm::vec3(0.0f);
    state_.acceleration = glm::vec3(0.0f);
}

void RigidBody::applyDamping(glm::vec3& velocity, float damping, float deltaTime) {
    velocity *= glm::pow(1.0f - damping, deltaTime);
}

void RigidBody::integrateVelocity(float deltaTime) {
    if (props_.isKinematic) return;

    state_.prevPosition = state_.position;
    state_.prevRotation = state_.rotation;

    updateInertiaTensor();

    if (props_.mass > 0.0f) {
        state_.acceleration = state_.totalForce / props_.mass;
    }

    state_.velocity += state_.acceleration * deltaTime;
    applyDamping(state_.velocity, props_.linearDamping, deltaTime);

    if (glm::length2(state_.velocity) < 0.001f) state_.velocity = glm::vec3(0.0f);

    if (props_.mass > 0.0f) {
        const glm::vec3 angularAcceleration = inverseInertiaTensor_ * state_.totalTorque;
        state_.angularVelocity += angularAcceleration * deltaTime;
    }

    applyDamping(state_.angularVelocity, props_.angularDamping, deltaTime);

    if (glm::length2(state_.angularVelocity) < 0.001f) state_.angularVelocity = glm::vec3(0.0f);

    if (glm::length2(state_.angularVelocity) > 0.001f) {
        const glm::quat angularVelocityQuat(0.0f, state_.angularVelocity);
        const glm::quat rotationDelta = state_.rotation * angularVelocityQuat * (deltaTime * 0.5f);
        state_.rotation = glm::normalize(state_.rotation + rotationDelta);
        inverseInertiaTensorDirty_ = true;
    }

    resetForces();
}

void RigidBody::integratePosition(float deltaTime) {
    if (props_.isKinematic) return;

    state_.position += state_.velocity * deltaTime;
    updateTransform();
}

void RigidBody::movePosition(const glm::vec3& delta) {
    if (props_.isKinematic) return;

    state_.position += delta;
    updateTransform();
}

} // namespace ge
