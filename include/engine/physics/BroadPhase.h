#pragma once

#include <vector>
#include <memory>
#include <utility>
#include <glm/glm.hpp>

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
        glm::vec3 min;
        glm::vec3 max;
    };

    static bool intersects(const AABB& a, const AABB& b);
};

} // namespace ge
