#include "engine/physics/PhysicsWorld.h"

#include "engine/physics/BoxCollider.h"
#include "engine/physics/RigidBody.h"
#include "engine/physics/SphereCollider.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <limits>
#include <unordered_set>
#include <utility>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace ge {
namespace {

bool differs(const glm::vec3& a, const glm::vec3& b, float eps = 1e-6f) {
    const glm::vec3 d = a - b;
    return glm::dot(d, d) > eps * eps;
}

uint64_t makeBodyPairKey(uint32_t bodyIdA, uint32_t bodyIdB) {
    const uint32_t lo = std::min(bodyIdA, bodyIdB);
    const uint32_t hi = std::max(bodyIdA, bodyIdB);
    return (static_cast<uint64_t>(lo) << 32) | static_cast<uint64_t>(hi);
}

int64_t quantizeComponent(float value) {
    return static_cast<int64_t>(std::llround(value * 10000.0f));
}

uint64_t hashContactSignature(uint32_t bodyIdA,
                              uint32_t bodyIdB,
                              const glm::vec3& point,
                              const glm::vec3& normal) {
    uint64_t hash = makeBodyPairKey(bodyIdA, bodyIdB);
    const int64_t px = quantizeComponent(point.x);
    const int64_t py = quantizeComponent(point.y);
    const int64_t pz = quantizeComponent(point.z);
    const int64_t nx = quantizeComponent(normal.x);
    const int64_t ny = quantizeComponent(normal.y);
    const int64_t nz = quantizeComponent(normal.z);

    auto mix = [&hash](int64_t value) {
        hash ^= static_cast<uint64_t>(value) + 0x9e3779b97f4a7c15ull + (hash << 6) + (hash >> 2);
    };

    mix(px);
    mix(py);
    mix(pz);
    mix(nx);
    mix(ny);
    mix(nz);
    return hash;
}

glm::vec3 extractWorldScale(const glm::mat4& transform) {
    return glm::vec3(
        glm::length(glm::vec3(transform[0])),
        glm::length(glm::vec3(transform[1])),
        glm::length(glm::vec3(transform[2]))
    );
}

bool raycastBox(const BoxCollider& box,
                const glm::mat4& transform,
                const glm::vec3& origin,
                const glm::vec3& direction,
                float maxDistance,
                float& outDistance,
                glm::vec3& outNormal) {
    const glm::mat4 invTransform = glm::inverse(transform);
    const glm::vec3 localOrigin = glm::vec3(invTransform * glm::vec4(origin, 1.0f));
    const glm::vec3 localDirection = glm::vec3(invTransform * glm::vec4(direction, 0.0f));
    const float directionLength = glm::length(localDirection);
    if (directionLength < 1e-6f) {
        return false;
    }

    const glm::vec3 localDir = localDirection / directionLength;
    const glm::vec3 halfExtents = box.getHalfExtents();

    float tMin = 0.0f;
    float tMax = maxDistance * directionLength;
    int hitAxis = -1;
    float hitSign = 1.0f;

    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(localDir[axis]) < 1e-6f) {
            if (localOrigin[axis] < -halfExtents[axis] || localOrigin[axis] > halfExtents[axis]) {
                return false;
            }
            continue;
        }

        const float invD = 1.0f / localDir[axis];
        float t0 = (-halfExtents[axis] - localOrigin[axis]) * invD;
        float t1 = (halfExtents[axis] - localOrigin[axis]) * invD;
        float sign = 1.0f;
        if (invD < 0.0f) {
            std::swap(t0, t1);
            sign = -1.0f;
        }

        if (t0 > tMin) {
            tMin = t0;
            hitAxis = axis;
            hitSign = sign;
        }
        tMax = std::min(tMax, t1);
        if (tMax <= tMin) {
            return false;
        }
    }

    if (tMax <= tMin || tMin <= 0.0f || tMin > maxDistance * directionLength) {
        return false;
    }

    outDistance = tMin / directionLength;
    glm::vec3 localNormal(0.0f);
    if (hitAxis >= 0) {
        localNormal[hitAxis] = hitSign;
    } else {
        localNormal = glm::vec3(0.0f, 1.0f, 0.0f);
    }

    outNormal = glm::normalize(glm::vec3(transform * glm::vec4(localNormal, 0.0f)));
    return true;
}

bool raycastSphere(const SphereCollider& sphere,
                   const glm::mat4& transform,
                   const glm::vec3& origin,
                   const glm::vec3& direction,
                   float maxDistance,
                   float& outDistance,
                   glm::vec3& outNormal) {
    const glm::vec3 center = glm::vec3(transform[3]);
    const glm::vec3 scale = extractWorldScale(transform);
    const float radius = sphere.getRadius() * std::max({scale.x, scale.y, scale.z});

    const glm::vec3 oc = origin - center;
    const float a = glm::dot(direction, direction);
    const float b = 2.0f * glm::dot(oc, direction);
    const float c = glm::dot(oc, oc) - radius * radius;
    const float discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0.0f) {
        return false;
    }

    const float sqrtDisc = std::sqrt(discriminant);
    float t = (-b - sqrtDisc) / (2.0f * a);
    if (t <= 0.0f) {
        t = (-b + sqrtDisc) / (2.0f * a);
    }
    if (t <= 0.0f || t > maxDistance) {
        return false;
    }

    outDistance = t;
    const glm::vec3 hitPoint = origin + direction * t;
    outNormal = glm::normalize(hitPoint - center);
    return true;
}

} // namespace

PhysicsWorld::PhysicsWorld() {
    setGravity(gravity_);
    setSolverIterations(solverIterations_);
}

PhysicsWorld::~PhysicsWorld() {
    clearBodies();
}

cannon::Vec3 PhysicsWorld::toCannon(const glm::vec3& value) {
    return cannon::Vec3(value.x, value.y, value.z);
}

glm::vec3 PhysicsWorld::toGlm(const cannon::Vec3& value) {
    return glm::vec3(value.x, value.y, value.z);
}

void PhysicsWorld::setGravity(const glm::vec3& gravity) {
    gravity_ = gravity;
    cannonWorld_.gravity = toCannon(gravity_);
}

cannon::Body* PhysicsWorld::createCannonBodyFromRigidBody(const RigidBody& rigidBody) {
    auto cannonBody = std::make_unique<cannon::Body>(rigidBody.getProps(), rigidBody.getWorldTransform());

    const Collider& collider = rigidBody.getCollider();
    if (collider.getType() == ColliderType::Box) {
        const auto& box = static_cast<const BoxCollider&>(collider);
        cannonBody->addShape(std::make_unique<BoxCollider>(box.getHalfExtents()));
    } else if (collider.getType() == ColliderType::Sphere) {
        const auto& sphere = static_cast<const SphereCollider&>(collider);
        cannonBody->addShape(std::make_unique<SphereCollider>(sphere.getRadius()));
    }

    cannonBody->type = rigidBody.getProps().isKinematic ? cannon::BodyType::KINEMATIC
                    : (rigidBody.getProps().mass > 0.0f ? cannon::BodyType::DYNAMIC : cannon::BodyType::STATIC);

    syncRigidToCannon(rigidBody, *cannonBody);

    cannon::Body* raw = cannonBody.get();
    cannonBindings_.push_back({const_cast<RigidBody*>(&rigidBody), std::move(cannonBody)});
    return raw;
}

void PhysicsWorld::syncRigidToCannon(const RigidBody& rigidBody, cannon::Body& cannonBody) {
    const glm::vec3 rbPosition = rigidBody.getPosition();
    cannonBody.position = toCannon(rbPosition);
    cannonBody.velocity = toCannon(rigidBody.getVelocity());
    cannonBody.angularVelocity = toCannon(rigidBody.getAngularVelocity());

    const glm::mat4 transform = rigidBody.getWorldTransform();
    const glm::quat rotation = glm::quat_cast(transform);
    cannonBody.quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);

    cannonBody.mass = rigidBody.getProps().mass;
    cannonBody.invMass = (rigidBody.getProps().isKinematic || rigidBody.getProps().mass <= 0.0f) ? 0.0f : (1.0f / rigidBody.getProps().mass);
    cannonBody.invMassSolve = cannonBody.invMass;
    cannonBody.linearDamping = rigidBody.getProps().linearDamping;
    cannonBody.angularDamping = rigidBody.getProps().angularDamping;
    cannonBody.isTrigger = rigidBody.getProps().isTrigger;
    cannonBody.useGravity = rigidBody.getProps().useGravity;
    cannonBody.collisionLayer = rigidBody.getProps().collisionLayer;
    cannonBody.collisionMask = rigidBody.getProps().collisionMask;
    cannonBody.type = rigidBody.getProps().isKinematic ? cannon::BodyType::KINEMATIC
                    : (rigidBody.getProps().mass > 0.0f ? cannon::BodyType::DYNAMIC : cannon::BodyType::STATIC);
    cannonBody.updateMassProperties();
}

void PhysicsWorld::syncCannonToRigid(const cannon::Body& cannonBody, RigidBody& rigidBody) {
    const glm::quat rotation(cannonBody.quaternion.w,
                             cannonBody.quaternion.x,
                             cannonBody.quaternion.y,
                             cannonBody.quaternion.z);
    glm::mat4 transform = glm::mat4_cast(rotation);
    const glm::vec3 scale = rigidBody.getLocalScale();
    transform[0] *= scale.x;
    transform[1] *= scale.y;
    transform[2] *= scale.z;
    const glm::vec3 centerOfMassOffset = rigidBody.getProps().centerOfMassOffset;
    const glm::vec3 comWorld(cannonBody.position.x, cannonBody.position.y, cannonBody.position.z);
    const glm::vec3 shapeOrigin = comWorld - (rotation * centerOfMassOffset);
    transform[3] = glm::vec4(shapeOrigin, 1.0f);

    rigidBody.setTransform(transform);
    rigidBody.setVelocity(toGlm(cannonBody.velocity));
    rigidBody.setAngularVelocity(toGlm(cannonBody.angularVelocity));
}

RigidBody* PhysicsWorld::addBody(std::unique_ptr<RigidBody> body) {
    if (!body) {
        return nullptr;
    }

    RigidBody* rigidBody = body.get();
    const uint32_t bodyId = nextBodyId_++;
    bodies_.push_back(std::move(body));

    cannon::Body* cannonBody = createCannonBodyFromRigidBody(*rigidBody);
    cannonBody->id = static_cast<int>(bodyId);
    cannonWorld_.addBody(cannonBody);
    rigidToCannon_[rigidBody] = cannonBody;
    rigidToId_[rigidBody] = bodyId;
    idToRigid_[bodyId] = rigidBody;
    idToCannon_[bodyId] = cannonBody;

    return rigidBody;
}

void PhysicsWorld::removeBody(RigidBody* body) {
    if (!body) {
        return;
    }

    auto cannonIt = rigidToCannon_.find(body);
    cannon::Body* removedCannonBody = nullptr;
    if (cannonIt != rigidToCannon_.end()) {
        removedCannonBody = cannonIt->second;
    }

    if (removedCannonBody) {
        for (size_t i = 0; i < constraints_.size();) {
            cannon::Constraint* c = constraints_[i].get();
            if (c && (c->bodyA == removedCannonBody || c->bodyB == removedCannonBody)) {
                cannonWorld_.removeConstraint(c);
                constraints_.erase(constraints_.begin() + i);
                continue;
            }
            ++i;
        }
    }

    if (cannonIt != rigidToCannon_.end()) {
        cannonWorld_.removeBody(cannonIt->second);
        rigidToCannon_.erase(cannonIt);
    }

    auto idIt = rigidToId_.find(body);
    if (idIt != rigidToId_.end()) {
        const uint32_t bodyId = idIt->second;
        idToRigid_.erase(bodyId);
        idToCannon_.erase(bodyId);
        rigidToId_.erase(idIt);
    }

    cannonBindings_.erase(std::remove_if(cannonBindings_.begin(), cannonBindings_.end(),
        [body](const CannonBinding& binding) { return binding.rigidBody == body; }), cannonBindings_.end());

    bodies_.erase(std::remove_if(bodies_.begin(), bodies_.end(),
        [body](const std::unique_ptr<RigidBody>& ptr) { return ptr.get() == body; }), bodies_.end());
}

void PhysicsWorld::clearBodies() {
    clearConstraints();

    for (const auto& binding : cannonBindings_) {
        cannonWorld_.removeBody(binding.cannonBody.get());
    }
    rigidToCannon_.clear();
    rigidToId_.clear();
    idToRigid_.clear();
    idToCannon_.clear();
    cannonBindings_.clear();
    bodies_.clear();
}

cannon::PointToPointConstraint* PhysicsWorld::addPointToPointConstraint(RigidBody* bodyA,
                                                                        const glm::vec3& pivotA,
                                                                        RigidBody* bodyB,
                                                                        const glm::vec3& pivotB,
                                                                        float maxForce,
                                                                        bool collideConnected) {
    if (!bodyA || !bodyB) {
        return nullptr;
    }

    auto itA = rigidToCannon_.find(bodyA);
    auto itB = rigidToCannon_.find(bodyB);
    if (itA == rigidToCannon_.end() || itB == rigidToCannon_.end()) {
        return nullptr;
    }

    auto constraint = std::make_unique<cannon::PointToPointConstraint>(
        itA->second,
        toCannon(pivotA),
        itB->second,
        toCannon(pivotB),
        maxForce,
        collideConnected);

    auto* raw = constraint.get();
    constraints_.push_back(std::move(constraint));
    cannonWorld_.addConstraint(raw);
    return raw;
}

cannon::DistanceConstraint* PhysicsWorld::addDistanceConstraint(RigidBody* bodyA,
                                                                RigidBody* bodyB,
                                                                float distance,
                                                                float maxForce,
                                                                bool collideConnected) {
    if (!bodyA || !bodyB || distance < 0.0f) {
        return nullptr;
    }

    auto itA = rigidToCannon_.find(bodyA);
    auto itB = rigidToCannon_.find(bodyB);
    if (itA == rigidToCannon_.end() || itB == rigidToCannon_.end()) {
        return nullptr;
    }

    auto constraint = std::make_unique<cannon::DistanceConstraint>(
        itA->second,
        itB->second,
        distance,
        maxForce,
        collideConnected);

    auto* raw = constraint.get();
    constraints_.push_back(std::move(constraint));
    cannonWorld_.addConstraint(raw);
    return raw;
}

cannon::LockConstraint* PhysicsWorld::addLockConstraint(RigidBody* bodyA,
                                                        RigidBody* bodyB,
                                                        float maxForce,
                                                        bool collideConnected) {
    if (!bodyA || !bodyB) {
        return nullptr;
    }

    auto itA = rigidToCannon_.find(bodyA);
    auto itB = rigidToCannon_.find(bodyB);
    if (itA == rigidToCannon_.end() || itB == rigidToCannon_.end()) {
        return nullptr;
    }

    auto constraint = std::make_unique<cannon::LockConstraint>(
        itA->second,
        itB->second,
        maxForce,
        collideConnected);

    auto* raw = constraint.get();
    constraints_.push_back(std::move(constraint));
    cannonWorld_.addConstraint(raw);
    return raw;
}

cannon::HingeConstraint* PhysicsWorld::addHingeConstraint(RigidBody* bodyA,
                                                          RigidBody* bodyB,
                                                          const glm::vec3& pivotA,
                                                          const glm::vec3& pivotB,
                                                          const glm::vec3& axisA,
                                                          const glm::vec3& axisB,
                                                          float maxForce,
                                                          bool collideConnected) {
    if (!bodyA || !bodyB) {
        return nullptr;
    }

    auto itA = rigidToCannon_.find(bodyA);
    auto itB = rigidToCannon_.find(bodyB);
    if (itA == rigidToCannon_.end() || itB == rigidToCannon_.end()) {
        return nullptr;
    }

    auto constraint = std::make_unique<cannon::HingeConstraint>(
        itA->second,
        itB->second,
        toCannon(pivotA),
        toCannon(pivotB),
        toCannon(axisA),
        toCannon(axisB),
        maxForce,
        collideConnected);

    auto* raw = constraint.get();
    constraints_.push_back(std::move(constraint));
    cannonWorld_.addConstraint(raw);
    return raw;
}

cannon::ConeTwistConstraint* PhysicsWorld::addConeTwistConstraint(RigidBody* bodyA,
                                                                  RigidBody* bodyB,
                                                                  const glm::vec3& pivotA,
                                                                  const glm::vec3& pivotB,
                                                                  const glm::vec3& axisA,
                                                                  const glm::vec3& axisB,
                                                                  float angle,
                                                                  float twistAngle,
                                                                  float maxForce,
                                                                  bool collideConnected) {
    if (!bodyA || !bodyB) {
        return nullptr;
    }

    auto itA = rigidToCannon_.find(bodyA);
    auto itB = rigidToCannon_.find(bodyB);
    if (itA == rigidToCannon_.end() || itB == rigidToCannon_.end()) {
        return nullptr;
    }

    auto constraint = std::make_unique<cannon::ConeTwistConstraint>(
        itA->second,
        itB->second,
        toCannon(pivotA),
        toCannon(pivotB),
        toCannon(axisA),
        toCannon(axisB),
        angle,
        twistAngle,
        maxForce,
        collideConnected);

    auto* raw = constraint.get();
    constraints_.push_back(std::move(constraint));
    cannonWorld_.addConstraint(raw);
    return raw;
}

void PhysicsWorld::removeConstraint(cannon::Constraint* constraint) {
    if (!constraint) {
        return;
    }

    cannonWorld_.removeConstraint(constraint);
    constraints_.erase(std::remove_if(constraints_.begin(), constraints_.end(),
                                      [constraint](const std::unique_ptr<cannon::Constraint>& c) {
                                          return c.get() == constraint;
                                      }),
                       constraints_.end());
}

void PhysicsWorld::clearConstraints() {
    for (const auto& constraint : constraints_) {
        cannonWorld_.removeConstraint(constraint.get());
    }
    constraints_.clear();
}

void PhysicsWorld::rebuildContactManifolds() {
    contactManifolds_.clear();

    std::map<uint64_t, ContactManifold> manifoldsByPair;
    for (const cannon::ContactEquation* contact : cannonWorld_.contacts) {
        if (!contact || !contact->bi || !contact->bj) {
            continue;
        }

        auto itA = idToRigid_.find(static_cast<uint32_t>(contact->bi->id));
        auto itB = idToRigid_.find(static_cast<uint32_t>(contact->bj->id));
        if (itA == idToRigid_.end() || itB == idToRigid_.end()) {
            continue;
        }

        const uint32_t bodyIdA = static_cast<uint32_t>(contact->bi->id);
        const uint32_t bodyIdB = static_cast<uint32_t>(contact->bj->id);
        const uint64_t key = makeBodyPairKey(bodyIdA, bodyIdB);

        ContactManifold& manifold = manifoldsByPair[key];
        if (!manifold.bodyA || !manifold.bodyB) {
            manifold.bodyA = itA->second;
            manifold.bodyB = itB->second;
        }

        const glm::vec3 pointA(contact->bi->position.x + contact->ri.x,
                               contact->bi->position.y + contact->ri.y,
                               contact->bi->position.z + contact->ri.z);
        const glm::vec3 pointB(contact->bj->position.x + contact->rj.x,
                               contact->bj->position.y + contact->rj.y,
                               contact->bj->position.z + contact->rj.z);
        const glm::vec3 normal = glm::normalize(glm::vec3(contact->ni.x, contact->ni.y, contact->ni.z));
        const glm::vec3 point = 0.5f * (pointA + pointB);
        const float depth = std::max(0.0f, -glm::dot(normal, pointB - pointA));

        Contact contactPoint;
        contactPoint.bodyA = manifold.bodyA;
        contactPoint.bodyB = manifold.bodyB;
        contactPoint.normal = normal;
        contactPoint.depth = depth;
        contactPoint.point = point;
        contactPoint.persistentId = static_cast<uint32_t>(hashContactSignature(bodyIdA, bodyIdB, point, normal));
        manifold.contacts.push_back(contactPoint);
        manifold.normal += normal;
        manifold.isColliding = true;
    }

    contactManifolds_.reserve(manifoldsByPair.size());
    for (auto& entry : manifoldsByPair) {
        ContactManifold& manifold = entry.second;
        if (!manifold.contacts.empty()) {
            manifold.normal /= static_cast<float>(manifold.contacts.size());
            const float normalLengthSq = glm::dot(manifold.normal, manifold.normal);
            if (normalLengthSq > 1e-12f) {
                manifold.normal = glm::normalize(manifold.normal);
            } else {
                manifold.normal = manifold.contacts.front().normal;
            }
        }
        contactManifolds_.push_back(std::move(manifold));
    }
}

void PhysicsWorld::setSolverIterations(int iterations) {
    solverIterations_ = std::max(1, iterations);
    if (cannonWorld_.solver) {
        cannonWorld_.solver->iterations = solverIterations_;
    }
}

int PhysicsWorld::getSolverIterations() const {
    return solverIterations_;
}

void PhysicsWorld::setCollisionCallback(CollisionCallback callback) {
    collisionCallback_ = std::move(callback);
}

void PhysicsWorld::setContactManifoldCallback(ContactManifoldCallback callback) {
    contactManifoldCallback_ = std::move(callback);
}

void PhysicsWorld::step(float deltaTime, int maxSubSteps) {
    if (deltaTime <= 0.0f) {
        return;
    }

    for (const auto& binding : cannonBindings_) {
        const RigidBodyProps& props = binding.rigidBody->getProps();
        if (props.isKinematic || props.mass <= 0.0f) {
            syncRigidToCannon(*binding.rigidBody, *binding.cannonBody);
            continue;
        }

        // Dynamic bodies are simulation-owned, but gameplay code may inject
        // launch/spin updates through RigidBody setters between frames.
        const glm::vec3 rbVelocity = binding.rigidBody->getVelocity();
        const glm::vec3 rbAngularVelocity = binding.rigidBody->getAngularVelocity();
        const glm::vec3 cbVelocity = toGlm(binding.cannonBody->velocity);
        const glm::vec3 cbAngularVelocity = toGlm(binding.cannonBody->angularVelocity);

        if (differs(rbVelocity, cbVelocity) || differs(rbAngularVelocity, cbAngularVelocity)) {
            syncRigidToCannon(*binding.rigidBody, *binding.cannonBody);
            binding.cannonBody->wakeUp();
        }
    }

    const int subSteps = std::max(1, maxSubSteps);
    const float subDeltaTime = deltaTime / static_cast<float>(subSteps);
    for (int i = 0; i < subSteps; ++i) {
        cannonWorld_.step(subDeltaTime, -1.0f, 1);
    }

    rebuildContactManifolds();

    if (contactManifoldCallback_) {
        for (const ContactManifold& manifold : contactManifolds_) {
            contactManifoldCallback_(manifold);
        }
    }

    for (const auto& binding : cannonBindings_) {
        const RigidBodyProps& props = binding.rigidBody->getProps();
        if (!props.isKinematic && props.mass > 0.0f) {
            syncCannonToRigid(*binding.cannonBody, *binding.rigidBody);
        }
    }

    if (collisionCallback_) {
        std::vector<std::pair<int, int>> began;
        std::vector<std::pair<int, int>> ended;
        std::vector<std::pair<int, int>> current;
        cannonWorld_.getBodyOverlapDeltas(began, ended);
        cannonWorld_.getCurrentBodyOverlaps(current);

        std::unordered_set<uint64_t> beganKeys;
        beganKeys.reserve(began.size());
        for (const auto& pair : began) {
            const int a = std::min(pair.first, pair.second);
            const int b = std::max(pair.first, pair.second);
            const uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) |
                                 static_cast<uint64_t>(static_cast<uint32_t>(b));
            beganKeys.insert(key);

            auto itA = idToRigid_.find(static_cast<uint32_t>(a));
            auto itB = idToRigid_.find(static_cast<uint32_t>(b));
            if (itA != idToRigid_.end() && itB != idToRigid_.end()) {
                collisionCallback_(CollisionEvent{CollisionPhase::Begin, itA->second, itB->second});
            }
        }

        for (const auto& pair : current) {
            const int a = std::min(pair.first, pair.second);
            const int b = std::max(pair.first, pair.second);
            const uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) |
                                 static_cast<uint64_t>(static_cast<uint32_t>(b));
            if (beganKeys.find(key) != beganKeys.end()) {
                continue;
            }

            auto itA = idToRigid_.find(static_cast<uint32_t>(a));
            auto itB = idToRigid_.find(static_cast<uint32_t>(b));
            if (itA != idToRigid_.end() && itB != idToRigid_.end()) {
                collisionCallback_(CollisionEvent{CollisionPhase::Stay, itA->second, itB->second});
            }
        }

        for (const auto& pair : ended) {
            const int a = std::min(pair.first, pair.second);
            const int b = std::max(pair.first, pair.second);
            auto itA = idToRigid_.find(static_cast<uint32_t>(a));
            auto itB = idToRigid_.find(static_cast<uint32_t>(b));
            if (itA != idToRigid_.end() && itB != idToRigid_.end()) {
                collisionCallback_(CollisionEvent{CollisionPhase::End, itA->second, itB->second});
            }
        }
    }
}

PhysicsWorld::RaycastResult PhysicsWorld::raycast(const glm::vec3& origin, const glm::vec3& direction, float maxDistance) {
    RaycastResult result;
    result.fraction = 1.0f;

    const float directionLength = glm::length(direction);
    if (directionLength < 1e-6f || maxDistance <= 0.0f) {
        return result;
    }

    const glm::vec3 dir = direction / directionLength;
    float closestDistance = maxDistance;

    for (const auto& body : bodies_) {
        const auto& collider = body->getCollider();
        const glm::mat4& transform = body->getWorldTransform();
        float hitDistance = 0.0f;
        glm::vec3 hitNormal(0.0f);
        bool hit = false;

        if (collider.getType() == ColliderType::Box) {
            hit = raycastBox(static_cast<const BoxCollider&>(collider), transform, origin, dir, maxDistance, hitDistance, hitNormal);
        } else if (collider.getType() == ColliderType::Sphere) {
            hit = raycastSphere(static_cast<const SphereCollider&>(collider), transform, origin, dir, maxDistance, hitDistance, hitNormal);
        }

        if (hit && hitDistance > 0.0f && hitDistance < closestDistance) {
            closestDistance = hitDistance;
            result.hit = true;
            result.body = body.get();
            result.fraction = hitDistance / maxDistance;
            result.point = origin + dir * hitDistance;
            result.normal = hitNormal;
        }
    }

    return result;
}

bool PhysicsWorld::raycastAny(const glm::vec3& origin,
                             const glm::vec3& direction,
                             float maxDistance,
                             RaycastResult& result) {
    result = RaycastResult{};

    const float directionLength = glm::length(direction);
    if (directionLength < 1e-6f || maxDistance <= 0.0f) {
        return false;
    }

    const glm::vec3 dir = direction / directionLength;

    for (const auto& body : bodies_) {
        const auto& collider = body->getCollider();
        const glm::mat4& transform = body->getWorldTransform();
        float hitDistance = 0.0f;
        glm::vec3 hitNormal(0.0f);
        bool hit = false;

        if (collider.getType() == ColliderType::Box) {
            hit = raycastBox(static_cast<const BoxCollider&>(collider), transform, origin, dir, maxDistance, hitDistance, hitNormal);
        } else if (collider.getType() == ColliderType::Sphere) {
            hit = raycastSphere(static_cast<const SphereCollider&>(collider), transform, origin, dir, maxDistance, hitDistance, hitNormal);
        }

        if (hit && hitDistance > 0.0f && hitDistance <= maxDistance) {
            result.hit = true;
            result.body = body.get();
            result.fraction = hitDistance / maxDistance;
            result.point = origin + dir * hitDistance;
            result.normal = hitNormal;
            return true;
        }
    }

    return false;
}

void PhysicsWorld::raycastAll(const glm::vec3& origin,
                              const glm::vec3& direction,
                              float maxDistance,
                              const std::function<bool(const RaycastResult&)>& callback) {
    if (!callback) {
        return;
    }

    const float directionLength = glm::length(direction);
    if (directionLength < 1e-6f || maxDistance <= 0.0f) {
        return;
    }

    const glm::vec3 dir = direction / directionLength;

    for (const auto& body : bodies_) {
        const auto& collider = body->getCollider();
        const glm::mat4& transform = body->getWorldTransform();
        float hitDistance = 0.0f;
        glm::vec3 hitNormal(0.0f);
        bool hit = false;

        if (collider.getType() == ColliderType::Box) {
            hit = raycastBox(static_cast<const BoxCollider&>(collider), transform, origin, dir, maxDistance, hitDistance, hitNormal);
        } else if (collider.getType() == ColliderType::Sphere) {
            hit = raycastSphere(static_cast<const SphereCollider&>(collider), transform, origin, dir, maxDistance, hitDistance, hitNormal);
        }

        if (hit && hitDistance > 0.0f && hitDistance <= maxDistance) {
            RaycastResult result;
            result.hit = true;
            result.body = body.get();
            result.fraction = hitDistance / maxDistance;
            result.point = origin + dir * hitDistance;
            result.normal = hitNormal;

            if (!callback(result)) {
                return;
            }
        }
    }
}

} // namespace ge
