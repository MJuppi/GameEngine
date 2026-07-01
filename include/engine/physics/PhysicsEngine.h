#pragma once

// =============================================================================
// PhysicsEngine.h — Simple physics simulation for rigid bodies
// =============================================================================
// Provides basic physics simulation with collision detection and response.
// =============================================================================

#include <cstddef>
#include <memory>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace ge {

// Forward declarations
class RigidBody;
class Collider;
class PhysicsWorld;

/// @brief Physics vector type for consistency
using PhysicsVec3 = glm::vec3;

/// @brief Physics scalar type
using PhysicsScalar = float;

/// @brief Collision contact information
struct Contact {
    PhysicsVec3 normal{};      // Contact normal (from A to B)
    PhysicsVec3 point{};       // Contact point in world space
    PhysicsScalar depth = 0.0f; // Penetration depth
    RigidBody* bodyA = nullptr; // First body in collision
    RigidBody* bodyB = nullptr; // Second body in collision
};

/// @brief Collision result containing all contacts
struct CollisionResult {
    std::vector<Contact> contacts;
    bool isColliding = false;
};

/// @brief Base collider interface
class Collider {
public:
    virtual ~Collider() = default;

    /// @brief Get the type of collider
    virtual const char* getType() const = 0;

    /// @brief Check collision with another collider
    /// @param other The other collider to check against
    /// @param transformA Transform of this collider
    /// @param transformB Transform of the other collider
    /// @return Collision result
    virtual CollisionResult checkCollision(
        const Collider& other,
        const glm::mat4& transformA,
        const glm::mat4& transformB
    ) const = 0;

    /// @brief Get the bounding box of this collider in local space
    virtual void getLocalBounds(glm::vec3& min, glm::vec3& max) const = 0;

    /// @brief Get the bounding box of this collider in world space
    virtual void getWorldBounds(glm::vec3& min, glm::vec3& max, const glm::mat4& transform) const;
};

/// @brief Box collider (AABB)
class BoxCollider : public Collider {
public:
    /// @brief Construct a box collider
    /// @param halfExtents Half extents of the box (x, y, z)
    explicit BoxCollider(const glm::vec3& halfExtents);

    const char* getType() const override { return "Box"; }
    CollisionResult checkCollision(
        const Collider& other,
        const glm::mat4& transformA,
        const glm::mat4& transformB
    ) const override;

    void getLocalBounds(glm::vec3& min, glm::vec3& max) const override;

    const glm::vec3& getHalfExtents() const { return halfExtents_; }
    void setHalfExtents(const glm::vec3& extents) { halfExtents_ = extents; }

private:
    glm::vec3 halfExtents_;
};

/// @brief Sphere collider
class SphereCollider : public Collider {
public:
    /// @brief Construct a sphere collider
    /// @param radius Radius of the sphere
    explicit SphereCollider(float radius);

    const char* getType() const override { return "Sphere"; }
    CollisionResult checkCollision(
        const Collider& other,
        const glm::mat4& transformA,
        const glm::mat4& transformB
    ) const override;

    void getLocalBounds(glm::vec3& min, glm::vec3& max) const override;

    float getRadius() const { return radius_; }
    void setRadius(float radius) { radius_ = radius; }

private:
    float radius_;
};

/// @brief Physical properties of a rigid body
struct RigidBodyProps {
    float mass = 1.0f;                // Mass of the body
    float friction = 0.2f;           // Coefficient of friction
    float restitution = 0.5f;        // Bounciness (0-1)
    float linearDamping = 0.01f;     // Linear velocity damping
    float angularDamping = 0.01f;   // Angular velocity damping
    bool isKinematic = false;        // If true, body is not affected by physics
    bool useGravity = true;          // If true, body is affected by gravity
};

/// @brief Rigid body for physics simulation
class RigidBody {
public:
    /// @brief Construct a rigid body
    /// @param collider The collider for this body
    /// @param transform Initial transform (position, orientation, scale)
    /// @param props Physical properties
    RigidBody(std::unique_ptr<Collider> collider,
             const glm::mat4& transform = glm::mat4(1.0f),
             const RigidBodyProps& props = RigidBodyProps());

    ~RigidBody();

    RigidBody(const RigidBody&) = delete;
    RigidBody& operator=(const RigidBody&) = delete;

    // Getters
    Collider& getCollider() { return *collider_; }
    const Collider& getCollider() const { return *collider_; }
    const glm::mat4& getTransform() const { return transform_; }
    const glm::mat4& getWorldTransform() const { return worldTransform_; }
    const PhysicsVec3& getPosition() const { return position_; }
    const PhysicsVec3& getVelocity() const { return velocity_; }
    const PhysicsVec3& getAngularVelocity() const { return angularVelocity_; }
    const RigidBodyProps& getProps() const { return props_; }

    // Setters
    void setTransform(const glm::mat4& transform);
    void setPosition(const glm::vec3& position);
    void setVelocity(const glm::vec3& velocity);
    void setAngularVelocity(const glm::vec3& angularVelocity);
    void setProps(const RigidBodyProps& props);

    /// @brief Add force to the body
    /// @param force Force vector in world space
    void addForce(const PhysicsVec3& force);

    /// @brief Add torque to the body
    /// @param torque Torque vector in world space
    void addTorque(const PhysicsVec3& torque);

    /// @brief Update the body's transform from its position and orientation
    void updateTransform();

    /// @brief Set if the body is kinematic
    void setKinematic(bool kinematic) { props_.isKinematic = kinematic; }

    /// @brief Set if the body uses gravity
    void setUseGravity(bool useGravity) { props_.useGravity = useGravity; }

    /// @brief Set the mass
    void setMass(float mass) { props_.mass = mass; }

    // Internal physics state (accessed by PhysicsEngine)
    PhysicsVec3 getAcceleration() const { return acceleration_; }
    PhysicsVec3 getTotalForce() const { return totalForce_; }
    PhysicsVec3 getTotalTorque() const { return totalTorque_; }

    void resetForces();
    void integrate(float deltaTime);

private:
    friend class PhysicsEngine;

    std::unique_ptr<Collider> collider_;
    glm::mat4 transform_;          // Local transform
    glm::mat4 worldTransform_;     // World transform (updated each frame)
    PhysicsVec3 position_;

    RigidBodyProps props_;

    // Physics state
    PhysicsVec3 velocity_;
    PhysicsVec3 angularVelocity_;
    PhysicsVec3 acceleration_;
    PhysicsVec3 totalForce_;
    PhysicsVec3 totalTorque_;

    // Orientation
    glm::quat rotation_;

    // Derived properties
    glm::mat3 inverseInertiaTensor_;
    bool inverseInertiaTensorDirty_ = true;

    void updateInertiaTensor();
};

/// @brief Physics world containing all bodies and simulation settings
class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();

    /// @brief Set the gravity vector
    /// @param gravity Gravity vector (default: (0, -9.81, 0))
    void setGravity(const PhysicsVec3& gravity);

    /// @brief Get the gravity vector
    const PhysicsVec3& getGravity() const { return gravity_; }

    /// @brief Add a rigid body to the world
    /// @param body The body to add
    /// @return Pointer to the added body
    RigidBody* addBody(std::unique_ptr<RigidBody> body);

    /// @brief Remove a rigid body from the world
    /// @param body The body to remove
    void removeBody(RigidBody* body);

    /// @brief Remove all bodies from the world
    void clearBodies();

    /// @brief Get all rigid bodies
    const std::vector<std::unique_ptr<RigidBody>>& getBodies() const { return bodies_; }

    [[nodiscard]] bool empty() const noexcept { return bodies_.empty(); }
    [[nodiscard]] std::size_t bodyCount() const noexcept { return bodies_.size(); }

    /// @brief Step the physics simulation
    /// @param deltaTime Time step in seconds
    /// @param maxSubSteps Maximum number of sub-steps (for stability)
    void step(float deltaTime, int maxSubSteps = 1);

    /// @brief Set the number of solver iterations
    /// @param iterations Number of iterations for constraint solver
    void setSolverIterations(int iterations) { solverIterations_ = iterations; }

    /// @brief Set if continuous collision detection is enabled
    /// @param enabled Whether to enable CCD
    void setContinuousCollisionDetection(bool enabled) {
        continuousCdEnabled_ = enabled;
    }

private:
    friend class PhysicsEngine;

    PhysicsVec3 gravity_ = {0.0f, -9.81f, 0.0f};
    std::vector<std::unique_ptr<RigidBody>> bodies_;
    int solverIterations_ = 10;
    bool continuousCdEnabled_ = false;

    // Internal methods
    void applyGravity();
    void detectCollisions(std::vector<Contact>& contacts);
    void resolveCollisions(std::vector<Contact>& contacts);
};

/// @brief Main physics engine class
class PhysicsEngine {
public:
    PhysicsEngine();
    ~PhysicsEngine();

    PhysicsEngine(const PhysicsEngine&) = delete;
    PhysicsEngine& operator=(const PhysicsEngine&) = delete;

    /// @brief Get the physics world
    PhysicsWorld& getWorld() { return world_; }
    const PhysicsWorld& getWorld() const { return world_; }

    /// @brief Create a rigid body with a box collider
    /// @param halfExtents Half extents of the box
    /// @param transform Initial transform
    /// @param props Physical properties
    /// @return Pointer to the created body
    RigidBody* createBoxBody(const glm::vec3& halfExtents,
                           const glm::mat4& transform = glm::mat4(1.0f),
                           const RigidBodyProps& props = RigidBodyProps());

    /// @brief Create a rigid body with a sphere collider
    /// @param radius Radius of the sphere
    /// @param transform Initial transform
    /// @param props Physical properties
    /// @return Pointer to the created body
    RigidBody* createSphereBody(float radius,
                              const glm::mat4& transform = glm::mat4(1.0f),
                              const RigidBodyProps& props = RigidBodyProps());

    /// @brief Remove a rigid body
    /// @param body The body to remove
    void destroyBody(RigidBody* body);

    /// @brief Update the physics simulation
    /// @param deltaTime Time since last frame in seconds
    void update(float deltaTime);

    /// @brief Set the physics world's gravity
    /// @param gravity Gravity vector
    void setGravity(const glm::vec3& gravity);

    /// @brief Set fixed time step for simulation
    /// @param timeStep Fixed time step (0 for variable time step)
    void setFixedTimeStep(float timeStep) { fixedTimeStep_ = timeStep; }

    /// @brief Set maximum sub-steps for fixed time stepping
    /// @param maxSubSteps Maximum number of sub-steps
    void setMaxSubSteps(int maxSubSteps) { maxSubSteps_ = maxSubSteps; }

    /// @brief Set if physics simulation is paused
    /// @param paused Whether to pause simulation
    void setPaused(bool paused) { paused_ = paused; }

    /// @brief Check if physics simulation is paused
    bool isPaused() const { return paused_; }

    /// @brief Clear all bodies from the physics world
    void clear();

private:
    PhysicsWorld world_;
    float fixedTimeStep_ = 0.0f;      // 0 means use deltaTime directly
    int maxSubSteps_ = 5;
    bool paused_ = false;
    float accumulator_ = 0.0f;        // For fixed time stepping
};

} // namespace ge
