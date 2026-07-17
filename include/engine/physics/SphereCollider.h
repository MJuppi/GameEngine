#pragma once

#include "engine/physics/Collider.h"
#include <glm/glm.hpp>

namespace ge {

class SphereCollider : public Collider {
public:
    SphereCollider(float radius);

    std::string getType() const override;
    void getLocalBounds(glm::vec3& min, glm::vec3& max) const override;
    ContactManifold checkCollision(
        const Collider& other,
        const glm::mat4& transformA,
        const glm::mat4& transformB
    ) const override;
    void getWorldBounds(glm::vec3& min, glm::vec3& max, const glm::mat4& transform) const override;

    float getRadius() const { return radius_; }

private:
    float radius_;
};

} // namespace ge
