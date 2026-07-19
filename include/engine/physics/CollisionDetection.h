#pragma once

#include <memory>
#include <vector>

#include "engine/physics/Collider.h"
#include "engine/physics/RigidBody.h"

#include "engine/physics/BroadPhase.h"

namespace ge {

class CollisionDetection {
public:
    static ContactManifold checkCollision(
        const Collider& colliderA, const glm::mat4& transformA,
        const Collider& colliderB, const glm::mat4& transformB);

    static void detectCollisions(
        const std::vector<PotentialPair>& pairs,
        std::vector<ContactManifold>& manifolds);

    static void resolveCollisions(std::vector<ContactManifold>& manifolds, int iterations);
    static void resolveManifold(ContactManifold& manifold);
};

} // namespace ge
