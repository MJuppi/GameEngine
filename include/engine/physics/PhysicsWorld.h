#pragma once

#include <memory>
#include <cstdint>
#include <unordered_map>
#include <vector>
#include <functional>

#include <glm/glm.hpp>

#include "engine/physics/Collider.h"
#include "engine/physics/cannon/Constraint.h"
#include "engine/physics/cannon/World.h"

namespace ge {

class RigidBody;

class PhysicsWorld {
public:
    enum class CollisionPhase {
        Begin,
        Stay,
        End
    };

    struct CollisionEvent {
        CollisionPhase phase = CollisionPhase::Begin;
        RigidBody* bodyA = nullptr;
        RigidBody* bodyB = nullptr;
    };

    using CollisionCallback = std::function<void(const CollisionEvent&)>;
    using ContactManifoldCallback = std::function<void(const ContactManifold&)>;

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
    bool raycastAny(const glm::vec3& origin, const glm::vec3& direction, float maxDistance, RaycastResult& result);
    void raycastAll(const glm::vec3& origin,
                    const glm::vec3& direction,
                    float maxDistance,
                    const std::function<bool(const RaycastResult&)>& callback);

    void setCollisionCallback(CollisionCallback callback);
    void setContactManifoldCallback(ContactManifoldCallback callback);

    const std::vector<ContactManifold>& getContactManifolds() const { return contactManifolds_; }

    cannon::PointToPointConstraint* addPointToPointConstraint(RigidBody* bodyA,
                                                               const glm::vec3& pivotA,
                                                               RigidBody* bodyB,
                                                               const glm::vec3& pivotB,
                                                               float maxForce = 1e6f,
                                                               bool collideConnected = true);
    cannon::DistanceConstraint* addDistanceConstraint(RigidBody* bodyA,
                                                      RigidBody* bodyB,
                                                      float distance,
                                                      float maxForce = 1e6f,
                                                      bool collideConnected = true);
    cannon::LockConstraint* addLockConstraint(RigidBody* bodyA,
                                              RigidBody* bodyB,
                                              float maxForce = 1e6f,
                                              bool collideConnected = true);
    cannon::HingeConstraint* addHingeConstraint(RigidBody* bodyA,
                                                RigidBody* bodyB,
                                                const glm::vec3& pivotA,
                                                const glm::vec3& pivotB,
                                                const glm::vec3& axisA,
                                                const glm::vec3& axisB,
                                                float maxForce = 1e6f,
                                                bool collideConnected = true);
    cannon::ConeTwistConstraint* addConeTwistConstraint(RigidBody* bodyA,
                                                         RigidBody* bodyB,
                                                         const glm::vec3& pivotA,
                                                         const glm::vec3& pivotB,
                                                         const glm::vec3& axisA,
                                                         const glm::vec3& axisB,
                                                         float angle = 0.0f,
                                                         float twistAngle = 0.0f,
                                                         float maxForce = 1e6f,
                                                         bool collideConnected = true);
    void removeConstraint(cannon::Constraint* constraint);
    void clearConstraints();

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
    void rebuildContactManifolds();
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
    std::vector<std::unique_ptr<cannon::Constraint>> constraints_;
    std::vector<ContactManifold> contactManifolds_;

    glm::vec3 gravity_{0.0f, -9.81f, 0.0f};
    int solverIterations_ = 8;
    CollisionCallback collisionCallback_;
    ContactManifoldCallback contactManifoldCallback_;
};

} // namespace ge
