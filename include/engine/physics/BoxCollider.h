#pragma once

#include "engine/physics/Collider.h"
#include <glm/glm.hpp>

namespace ge {

class BoxCollider : public Collider {
public:
    explicit BoxCollider(const glm::vec3& halfExtents);

    void getLocalBounds(glm::vec3& min, glm::vec3& max) const override;
    void getWorldBounds(glm::vec3& min, glm::vec3& max, const glm::mat4& transform) const override;

    const glm::vec3& getHalfExtents() const { return halfExtents_; }

private:
    glm::vec3 halfExtents_;
};

} // namespace ge
