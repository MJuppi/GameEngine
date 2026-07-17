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
        glm::vec3 min, max;
        bodies[i]->getCollider().getWorldBounds(min, max, bodies[i]->getWorldTransform());
        aabbs[i].min[0] = min.x; aabbs[i].min[1] = min.y; aabbs[i].min[2] = min.z;
        aabbs[i].max[0] = max.x; aabbs[i].max[1] = max.y; aabbs[i].max[2] = max.z;
    }

    for (size_t i = 0; i < count; ++i) {
        for (size_t j = i + 1; j < count; ++j) {
            // Collision Filtering
            const auto& propsA = bodies[i]->getProps();
            const auto& propsB = bodies[j]->getProps();

            if (!(propsA.collisionMask & propsB.collisionLayer) ||
                !(propsB.collisionMask & propsA.collisionLayer)) {
                continue;
            }

            // AABB Check
            if (intersects(aabbs[i], aabbs[j])) {
                pairs.push_back({bodies[i].get(), bodies[j].get()});
            }
        }
    }

    return pairs;
}

bool BroadPhase::intersects(const AABB& a, const AABB& b) {
    if (a.max[0] < b.min[0] || a.min[0] > b.max[0]) return false;
    if (a.max[1] < b.min[1] || a.min[1] > b.max[1]) return false;
    if (a.max[2] < b.min[2] || a.min[2] > b.max[2]) return false;
    return true;
}

} // namespace ge
