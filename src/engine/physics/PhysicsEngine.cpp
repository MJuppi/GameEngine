#include "engine/physics/PhysicsEngine.h"
#include "glm/geometric.hpp"
#include <algorithm>
#include <cmath>
#include <string>

namespace ge {
namespace {

constexpr float kEpsilon = 1.0e-4f;

void applyDamping(glm::vec3& value, float damping, float deltaTime) {
    if (damping > 0.0f) {
        const float factor = std::max(0.0f, 1.0f - damping * deltaTime);
        value *= factor;
    }
}

void resolveContact(Contact& contact) {
    RigidBody* bodyA = contact.bodyA;
    RigidBody* bodyB = contact.bodyB;
    if (bodyA == nullptr || bodyB == nullptr) {
        return;
    }

    const RigidBodyProps& propsA = bodyA->getProps();
    const RigidBodyProps& propsB = bodyB->getProps();

    const float invMassA = (propsA.isKinematic || propsA.mass <= 0.0f) ? 0.0f : 1.0f / propsA.mass;
    const float invMassB = (propsB.isKinematic || propsB.mass <= 0.0f) ? 0.0f : 1.0f / propsB.mass;
    const float totalInvMass = invMassA + invMassB;

    if (totalInvMass <= 0.0f) {
        return;
    }

    const glm::vec3 relativeVelocity = bodyB->getVelocity() - bodyA->getVelocity();
    const float velocityAlongNormal = glm::dot(relativeVelocity, contact.normal);

    if (velocityAlongNormal > 0.0f) {
        return;
    }

    const float restitution = std::min(propsA.restitution, propsB.restitution);
    const float impulse = -(1.0f + restitution) * velocityAlongNormal / totalInvMass;

    bodyA->setVelocity(bodyA->getVelocity() - contact.normal * impulse * invMassA);
    bodyB->setVelocity(bodyB->getVelocity() + contact.normal * impulse * invMassB);

    const float friction = std::min(propsA.friction, propsB.friction);
    glm::vec3 tangent = relativeVelocity - contact.normal * velocityAlongNormal;
    if (glm::length(tangent) > kEpsilon) {
        tangent = glm::normalize(tangent);
        float frictionImpulse = -glm::dot(relativeVelocity, tangent) / totalInvMass;
        const float maxFriction = impulse * friction;
        frictionImpulse = std::clamp(frictionImpulse, -maxFriction, maxFriction);

        bodyA->setVelocity(bodyA->getVelocity() - tangent * frictionImpulse * invMassA);
        bodyB->setVelocity(bodyB->getVelocity() + tangent * frictionImpulse * invMassB);
    }

    const float correctionFactor = 0.8f;
    const float correctionMargin = 0.001f;
    if (contact.depth > correctionMargin) {
        const glm::vec3 correction =
            contact.normal * ((contact.depth - correctionMargin) * correctionFactor / totalInvMass);

        if (invMassA > 0.0f) {
            bodyA->setPosition(bodyA->getPosition() - correction * invMassA);
            bodyA->updateTransform();
        }

        if (invMassB > 0.0f) {
            bodyB->setPosition(bodyB->getPosition() + correction * invMassB);
            bodyB->updateTransform();
        }
    }
}

} // namespace

// =============================================================================
// Collider base class implementation
// =============================================================================

void Collider::getWorldBounds(glm::vec3& min, glm::vec3& max, const glm::mat4& transform) const {
    glm::vec3 localMin, localMax;
    getLocalBounds(localMin, localMax);

    // Transform all 8 corners of the bounding box
    min = max = glm::vec3(transform * glm::vec4(localMin, 1.0f));

    glm::vec4 corners[] = {
        {localMin.x, localMin.y, localMin.z, 1.0f},
        {localMax.x, localMin.y, localMin.z, 1.0f},
        {localMin.x, localMax.y, localMin.z, 1.0f},
        {localMax.x, localMax.y, localMin.z, 1.0f},
        {localMin.x, localMin.y, localMax.z, 1.0f},
        {localMax.x, localMin.y, localMax.z, 1.0f},
        {localMin.x, localMax.y, localMax.z, 1.0f},
        {localMax.x, localMax.y, localMax.z, 1.0f}
    };

    for (const auto& corner : corners) {
        glm::vec3 transformed = glm::vec3(transform * corner);
        min = glm::min(min, transformed);
        max = glm::max(max, transformed);
    }
}

// =============================================================================
// BoxCollider implementation
// =============================================================================

BoxCollider::BoxCollider(const glm::vec3& halfExtents)
    : halfExtents_(halfExtents) {
}

void BoxCollider::getLocalBounds(glm::vec3& min, glm::vec3& max) const {
    min = -halfExtents_;
    max = halfExtents_;
}

CollisionResult BoxCollider::checkCollision(
    const Collider& other,
    const glm::mat4& transformA,
    const glm::mat4& transformB
) const {
    CollisionResult result;

    // Check if the other collider is a BoxCollider
    if (other.getType() == std::string("Box")) {
        const BoxCollider& boxB = static_cast<const BoxCollider&>(other);

        // Extract OBB information from both boxes
        glm::vec3 aMin, aMax;
        getWorldBounds(aMin, aMax, transformA);

        glm::vec3 bMin, bMax;
        boxB.getWorldBounds(bMin, bMax, transformB);

        // Simple AABB collision detection for now
        // TODO: Implement proper OBB collision detection
        bool colliding = true;
        colliding &= (aMin.x <= bMax.x && aMax.x >= bMin.x);
        colliding &= (aMin.y <= bMax.y && aMax.y >= bMin.y);
        colliding &= (aMin.z <= bMax.z && aMax.z >= bMin.z);

        if (colliding) {
            result.isColliding = true;

            const float overlapX1 = aMax.x - bMin.x;
            const float overlapX2 = bMax.x - aMin.x;
            const float overlapY1 = aMax.y - bMin.y;
            const float overlapY2 = bMax.y - aMin.y;
            const float overlapZ1 = aMax.z - bMin.z;
            const float overlapZ2 = bMax.z - aMin.z;

            float penetration = overlapX1;
            glm::vec3 normal{1.0f, 0.0f, 0.0f};

            if (overlapX2 < penetration) { penetration = overlapX2; normal = {-1.0f, 0.0f, 0.0f}; }
            if (overlapY1 < penetration) { penetration = overlapY1; normal = {0.0f, 1.0f, 0.0f}; }
            if (overlapY2 < penetration) { penetration = overlapY2; normal = {0.0f, -1.0f, 0.0f}; }
            if (overlapZ1 < penetration) { penetration = overlapZ1; normal = {0.0f, 0.0f, 1.0f}; }
            if (overlapZ2 < penetration) { penetration = overlapZ2; normal = {0.0f, 0.0f, -1.0f}; }

            const glm::vec3 centerA = (aMin + aMax) * 0.5f;
            const glm::vec3 centerB = (bMin + bMax) * 0.5f;
            if (glm::dot(normal, centerB - centerA) < 0.0f) {
                normal = -normal;
            }

            Contact contact;
            contact.depth = penetration;
            contact.normal = normal;
            contact.point = {
                (std::max(aMin.x, bMin.x) + std::min(aMax.x, bMax.x)) * 0.5f,
                (std::max(aMin.y, bMin.y) + std::min(aMax.y, bMax.y)) * 0.5f,
                (std::max(aMin.z, bMin.z) + std::min(aMax.z, bMax.z)) * 0.5f
            };

            result.contacts.push_back(contact);
        }

    } else if (other.getType() == std::string("Sphere")) {
        const SphereCollider& sphereB = static_cast<const SphereCollider&>(other);

        // Box-Sphere collision detection
        glm::vec3 aMin, aMax;
        getWorldBounds(aMin, aMax, transformA);

        glm::vec3 sphereCenter = glm::vec3(transformB[3]);
        float sphereRadius = sphereB.getRadius();

        // Find closest point on box to sphere center
        glm::vec3 closestPoint = sphereCenter;
        closestPoint.x = std::clamp(closestPoint.x, aMin.x, aMax.x);
        closestPoint.y = std::clamp(closestPoint.y, aMin.y, aMax.y);
        closestPoint.z = std::clamp(closestPoint.z, aMin.z, aMax.z);

        const float distance = glm::distance(sphereCenter, closestPoint);

        if (distance <= sphereRadius) {
            result.isColliding = true;

            Contact contact;
            contact.depth = sphereRadius - distance;

            if (distance > 0.001f) {
                contact.normal = glm::normalize(sphereCenter - closestPoint);
            } else {
                // Sphere center is inside the box, use arbitrary normal
                contact.normal = { 0, 1, 0 };
            }

            contact.point = closestPoint;
            result.contacts.push_back(contact);
        }
    }

    return result;
}

// =============================================================================
// SphereCollider implementation
// =============================================================================

SphereCollider::SphereCollider(float radius)
    : radius_(radius) {
}

void SphereCollider::getLocalBounds(glm::vec3& min, glm::vec3& max) const {
    min = {-radius_, -radius_, -radius_};
    max = {radius_, radius_, radius_};
}

CollisionResult SphereCollider::checkCollision(
    const Collider& other,
    const glm::mat4& transformA,
    const glm::mat4& transformB
) const {
    CollisionResult result;

    if (other.getType() == std::string("Sphere")) {
        const SphereCollider& sphereB = static_cast<const SphereCollider&>(other);

        glm::vec3 centerA = glm::vec3(transformA[3]);
        glm::vec3 centerB = glm::vec3(transformB[3]);

        const float distance = glm::distance(centerA, centerB);
        const float radiusSum = radius_ + sphereB.getRadius();

        if (distance <= radiusSum) {
            result.isColliding = true;

            Contact contact;
            contact.depth = radiusSum - distance;

            if (distance > 0.001f) {
                contact.normal = glm::normalize(centerB - centerA);
            } else {
                // Spheres are overlapping completely, use arbitrary normal
                contact.normal = { 0, 1, 0 };
            }

            contact.point = centerA + contact.normal * radius_;
            result.contacts.push_back(contact);
        }

    } else if (other.getType() == std::string("Box")) {
        // Sphere-Box collision (same as Box-Sphere but reversed)
        const BoxCollider& boxB = static_cast<const BoxCollider&>(other);

        glm::vec3 sphereCenter = glm::vec3(transformA[3]);

        glm::vec3 bMin, bMax;
        boxB.getWorldBounds(bMin, bMax, transformB);

        // Find closest point on box to sphere center
        glm::vec3 closestPoint = sphereCenter;
        closestPoint.x = std::clamp(closestPoint.x, bMin.x, bMax.x);
        closestPoint.y = std::clamp(closestPoint.y, bMin.y, bMax.y);
        closestPoint.z = std::clamp(closestPoint.z, bMin.z, bMax.z);

        const float distance = glm::distance(sphereCenter, closestPoint);

        if (distance <= radius_) {
            result.isColliding = true;

            Contact contact;
            contact.depth = radius_ - distance;

            if (distance > 0.001f) {
                contact.normal = glm::normalize(closestPoint - sphereCenter);
            } else {
                contact.normal = { 0, 1, 0 };
            }

            contact.point = closestPoint;
            result.contacts.push_back(contact);
        }
    }

    return result;
}

// =============================================================================
// RigidBody implementation
// =============================================================================

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

RigidBody::~RigidBody() = default;

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

void RigidBody::addForce(const PhysicsVec3& force) {
    totalForce_ += force;
}

void RigidBody::addTorque(const PhysicsVec3& torque) {
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

void RigidBody::updateInertiaTensor() {
    if (!inverseInertiaTensorDirty_) return;

    // For now, use a simple spherical inertia tensor approximation
    // TODO: Implement proper inertia tensor calculation for different shapes
    float mass = props_.mass;
    if (mass <= 0.0f) {
        inverseInertiaTensor_ = glm::mat3(0.0f);
        inverseInertiaTensorDirty_ = false;
        return;
    }

    // Approximate as a sphere with radius 1.0
    float radius = 1.0f;
    float inertia = 0.4f * mass * radius * radius; // 2/5 * m * r^2 for sphere

    inverseInertiaTensor_ = glm::mat3(inertia > 0.0f ? 1.0f / inertia : 0.0f);
    inverseInertiaTensorDirty_ = false;
}

void RigidBody::resetForces() {
    totalForce_ = glm::vec3(0.0f);
    totalTorque_ = glm::vec3(0.0f);
    acceleration_ = glm::vec3(0.0f);
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
        // angularAcceleration = inverseInertiaTensor * totalTorque
        glm::vec3 angularAcceleration = inverseInertiaTensor_ * totalTorque_;
        angularVelocity_ += angularAcceleration * deltaTime;
    }

    applyDamping(angularVelocity_, props_.angularDamping, deltaTime);

    // Update rotation (using quaternions)
    if (glm::length(angularVelocity_) > 0.001f) {
        glm::quat angularVelocityQuat(0.0f, angularVelocity_);
        glm::quat rotationDelta = angularVelocityQuat * (deltaTime * 0.5f);
        rotation_ = glm::normalize(rotation_ + rotationDelta * rotation_);
    }

    // Update the world transform
    updateTransform();

    // Reset forces for next frame
    resetForces();
}

// =============================================================================
// PhysicsWorld implementation
// =============================================================================

PhysicsWorld::PhysicsWorld() {
}

PhysicsWorld::~PhysicsWorld() {
    clearBodies();
}

void PhysicsWorld::setGravity(const PhysicsVec3& gravity) {
    gravity_ = gravity;
}

RigidBody* PhysicsWorld::addBody(std::unique_ptr<RigidBody> body) {
    bodies_.push_back(std::move(body));
    return bodies_.back().get();
}

void PhysicsWorld::removeBody(RigidBody* body) {
    auto it = std::find_if(bodies_.begin(), bodies_.end(),
        [body](const std::unique_ptr<RigidBody>& ptr) { return ptr.get() == body; });

    if (it != bodies_.end()) {
        bodies_.erase(it);
    }
}

void PhysicsWorld::clearBodies() {
    bodies_.clear();
}

void PhysicsWorld::applyGravity() {
    for (auto& body : bodies_) {
        if (body->getProps().useGravity && !body->getProps().isKinematic) {
            body->addForce(gravity_ * body->getProps().mass);
        }
    }
}

void PhysicsWorld::detectCollisions(std::vector<Contact>& contacts) {
    contacts.clear();

    const std::size_t bodyCount = bodies_.size();
    if (bodyCount < 2) {
        return;
    }

    for (std::size_t i = 0; i < bodyCount; ++i) {
        for (std::size_t j = i + 1; j < bodyCount; ++j) {
            RigidBody* bodyA = bodies_[i].get();
            RigidBody* bodyB = bodies_[j].get();

            CollisionResult collision = bodyA->getCollider().checkCollision(
                bodyB->getCollider(),
                bodyA->getWorldTransform(),
                bodyB->getWorldTransform()
            );

            if (collision.isColliding) {
                for (auto& contact : collision.contacts) {
                    contact.bodyA = bodyA;
                    contact.bodyB = bodyB;
                    contacts.push_back(contact);
                }
            }
        }
    }
}

void PhysicsWorld::resolveCollisions(std::vector<Contact>& contacts) {
    for (auto& contact : contacts) {
        resolveContact(contact);
    }
}

void PhysicsWorld::step(float deltaTime, int /*maxSubSteps*/) {
    if (deltaTime <= 0.0f) {
        return;
    }

    applyGravity();

    for (auto& body : bodies_) {
        body->integrate(deltaTime);
    }

    std::vector<Contact> contacts;
    for (int iteration = 0; iteration < solverIterations_; ++iteration) {
        detectCollisions(contacts);
        if (contacts.empty()) {
            break;
        }
        resolveCollisions(contacts);
    }
}

// =============================================================================
// PhysicsEngine implementation
// =============================================================================

PhysicsEngine::PhysicsEngine() = default;

PhysicsEngine::~PhysicsEngine() = default;

RigidBody* PhysicsEngine::createBoxBody(const glm::vec3& halfExtents,
                                       const glm::mat4& transform,
                                       const RigidBodyProps& props) {
    auto collider = std::make_unique<BoxCollider>(halfExtents);
    auto body = std::make_unique<RigidBody>(std::move(collider), transform, props);
    return world_.addBody(std::move(body));
}

RigidBody* PhysicsEngine::createSphereBody(float radius,
                                         const glm::mat4& transform,
                                         const RigidBodyProps& props) {
    auto collider = std::make_unique<SphereCollider>(radius);
    auto body = std::make_unique<RigidBody>(std::move(collider), transform, props);
    return world_.addBody(std::move(body));
}

void PhysicsEngine::destroyBody(RigidBody* body) {
    world_.removeBody(body);
}

void PhysicsEngine::setGravity(const glm::vec3& gravity) {
    world_.setGravity(gravity);
}

void PhysicsEngine::update(float deltaTime) {
    if (paused_ || deltaTime <= 0.0f) {
        return;
    }

    if (fixedTimeStep_ > 0.0f) {
        // Fixed time stepping
        accumulator_ += deltaTime;

        while (accumulator_ >= fixedTimeStep_) {
            world_.step(fixedTimeStep_, maxSubSteps_);
            accumulator_ -= fixedTimeStep_;
        }
    } else {
        // Variable time stepping
        world_.step(deltaTime, maxSubSteps_);
    }
}

void PhysicsEngine::clear() {
    world_.clearBodies();
}

} // namespace ge
