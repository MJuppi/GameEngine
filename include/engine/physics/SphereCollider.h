#pragma once

#include "engine/physics/Collider.h"
#include <glm/glm.hpp>

namespace ge {

class SphereCollider : public Collider {
public:
    explicit SphereCollider(float radius);

    void getLocalBounds(glm::vec3& min, glm::vec3& max) const override;
    void getWorldBounds(glm::vec3& min, glm::vec3& max, const glm::mat4& transform) const override;

    float getRadius() const { return radius_; }

private:
    float radius_;
};

} // namespace ge
