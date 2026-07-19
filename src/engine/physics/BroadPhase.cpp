#include "engine/physics/BroadPhase.h"
#include "engine/physics/RigidBody.h"
#include "engine/physics/Collider.h"

namespace ge {

std::vector<PotentialPair> BroadPhase::findPairs(const std::vector<std::unique_ptr<RigidBody>>& bodies) {
    std::vector<PotentialPair> pairs;
    const size_t count = bodies.size();
    if (count < 2) return pairs;

    std::vector<AABB> aabbs(count);
    for (size_t i = 0; i < count; ++i) {
        bodies[i]->getCollider().getWorldBounds(aabbs[i].min, aabbs[i].max, bodies[i]->getWorldTransform());
    }

    for (size_t i = 0; i < count; ++i) {
        for (size_t j = i + 1; j < count; ++j) {
            const auto& propsA = bodies[i]->getProps();
            const auto& propsB = bodies[j]->getProps();

            if (!(propsA.collisionMask & propsB.collisionLayer) ||
                !(propsB.collisionMask & propsA.collisionLayer)) {
                continue;
            }

            if (intersects(aabbs[i], aabbs[j])) {
                pairs.push_back({bodies[i].get(), bodies[j].get()});
            }
        }
    }

    return pairs;
}

bool BroadPhase::intersects(const AABB& a, const AABB& b) {
    return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
           (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
           (a.min.z <= b.max.z && a.max.z >= b.min.z);
}

} // namespace ge
