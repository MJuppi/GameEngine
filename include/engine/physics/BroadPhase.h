#pragma once

#include <vector>
#include <memory>
#include <utility>

namespace ge {

class RigidBody;

struct PotentialPair {
    RigidBody* bodyA;
    RigidBody* bodyB;
};

class BroadPhase {
public:
    static std::vector<PotentialPair> findPairs(const std::vector<std::unique_ptr<RigidBody>>& bodies);

private:
    struct AABB {
        float min[3];
        float max[3];
    };

    static bool intersects(const AABB& a, const AABB& b);
};

} // namespace ge
