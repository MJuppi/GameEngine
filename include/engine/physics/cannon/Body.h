#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <vector>
#include <optional>
#include <string>
#include "engine/physics/cannon/Vec3.h"
#include "engine/physics/cannon/Mat3.h"
#include "engine/physics/cannon/Quaternion.h"
#include "engine/physics/cannon/Material.h"
#include "engine/physics/Collider.h"
#include "engine/physics/RigidBody.h"

namespace ge {
namespace cannon {

enum BodyType {
    DYNAMIC = 1,
    STATIC = 2,
    KINEMATIC = 4
};

class Body {
public:
    Body();
    explicit Body(const RigidBodyProps& props, const glm::mat4& transform);

    int id;
    int type;
    uint32_t collisionLayer;
    uint32_t collisionMask;
    bool useGravity;
    Vec3 position;
    Vec3 velocity;
    Vec3 force;
    Vec3 torque;
    Quaternion quaternion;
    Vec3 angularVelocity;
    float mass;
    float invMass;
    float invMassSolve;
    Vec3 invInertia;
    Mat3 invInertiaWorld;
    Mat3 invInertiaWorldSolve;
    Vec3 vlambda;
    Vec3 wlambda;
    Vec3 initPosition;
    Quaternion initQuaternion;
    Vec3 initVelocity;
    Vec3 initAngularVelocity;
    bool allowSleep;
    int sleepState;
    float sleepSpeedLimit;
    float sleepTimeLimit;
    float timeLastSleepy;
    bool wakeUpAfterNarrowphase;
    bool isTrigger;
    float linearDamping;
    float angularDamping;
    const Material* material;
    ColliderType shapeType;
    std::unique_ptr<Collider> collider;

    void addShape(std::unique_ptr<Collider> shape);
    void updateMassProperties();
    void applyForce(const Vec3& f);
    void integrate(float dt, bool quatNormalize);
    void wakeUp();
    void sleep();
    void sleepTick(float time);
    void updateSolveMassProperties();
};

} // namespace cannon
} // namespace ge
