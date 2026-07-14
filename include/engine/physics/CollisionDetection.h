#pragma once

#include <memory>
#include <vector>

#include "engine/physics/Collider.h"
#include "engine/physics/RigidBody.h"

namespace ge {

class CollisionDetection {
public:
    static void detectCollisions(
        const std::vector<std::unique_ptr<RigidBody>>& bodies,
        std::vector<Contact>& contacts);

    static void resolveCollisions(std::vector<Contact>& contacts);
    static void resolveContact(Contact& contact);
};

} // namespace ge
