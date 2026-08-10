#pragma once

#include <memory>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <glm/glm.hpp>

#include "engine/physics/Collider.h"
#include "engine/physics/cannon/World.h"

namespace ge {

class RigidBody;

class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();

    void setGravity(const glm::vec3& gravity);
    RigidBody* addBody(std::unique_ptr<RigidBody> body);
    void removeBody(RigidBody* body);
    void clearBodies();

    void step(float deltaTime, int maxSubSteps = 1);

    struct RaycastResult {
        bool hit = false;
        RigidBody* body = nullptr;
        glm::vec3 point{0.0f};
        glm::vec3 normal{0.0f};
        float fraction = 1.0f;
    };

    RaycastResult raycast(const glm::vec3& origin, const glm::vec3& direction, float maxDistance);

    void setSolverIterations(int iterations);
    int getSolverIterations() const;

    const std::vector<std::unique_ptr<RigidBody>>& getBodies() const { return bodies_; }

private:
    struct CannonBinding {
        RigidBody* rigidBody = nullptr;
        std::unique_ptr<cannon::Body> cannonBody;
    };

    cannon::Body* createCannonBodyFromRigidBody(const RigidBody& rigidBody);
    void syncRigidToCannon(const RigidBody& rigidBody, cannon::Body& cannonBody);
    void syncCannonToRigid(const cannon::Body& cannonBody, RigidBody& rigidBody);
    static cannon::Vec3 toCannon(const glm::vec3& value);
    static glm::vec3 toGlm(const cannon::Vec3& value);

    std::vector<std::unique_ptr<RigidBody>> bodies_;
    std::vector<CannonBinding> cannonBindings_;
    std::unordered_map<uint32_t, RigidBody*> idToRigid_;
    std::unordered_map<uint32_t, cannon::Body*> idToCannon_;
    std::unordered_map<RigidBody*, cannon::Body*> rigidToCannon_;
    std::unordered_map<RigidBody*, uint32_t> rigidToId_;
    uint32_t nextBodyId_ = 1;
    cannon::World cannonWorld_;

    glm::vec3 gravity_{0.0f, -9.81f, 0.0f};
    int solverIterations_ = 8;
};

} // namespace ge
