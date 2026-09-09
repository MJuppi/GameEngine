#include "engine/physics/BoxCollider.h"
#include "engine/physics/PhysicsEngine.h"
#include "engine/physics/PhysicsWorld.h"
#include "engine/physics/RigidBody.h"
#include "engine/physics/SphereCollider.h"
#include "engine/physics/cannon/World.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>

namespace {

constexpr float kFixedTimeStep = 1.0f / 60.0f;

void requireNear(float actual, float expected, float tolerance, const char* message) {
    if (std::fabs(actual - expected) > tolerance) {
        throw std::runtime_error(std::string(message) + ": expected " +
                                 std::to_string(expected) + ", got " +
                                 std::to_string(actual));
    }
}

std::unique_ptr<ge::RigidBody> makeBoxBody(const ge::RigidBodyProps& props,
                                           const glm::mat4& transform = glm::mat4(1.0f),
                                           const glm::vec3& halfExtents = glm::vec3(0.5f)) {
    return std::make_unique<ge::RigidBody>(
        std::make_unique<ge::BoxCollider>(halfExtents),
        transform,
        props);
}

void testAccumulatorAdvancesOneFixedStep() {
    ge::PhysicsWorld world;
    ge::RigidBodyProps props;
    props.linearDamping = 0.0f;
    props.angularDamping = 0.0f;

    ge::RigidBody* body = world.addBody(makeBoxBody(props));
    world.step(kFixedTimeStep, 4);

    const float expectedY = -9.81f * kFixedTimeStep * kFixedTimeStep;
    requireNear(body->getPosition().y, expectedY, 1e-5f,
                "A fixed elapsed tick must produce one Cannon simulation step");
}

void testPhysicsEngineUpdateUsesElapsedTimeParameter() {
    ge::PhysicsEngine engine;
    ge::RigidBodyProps props;
    props.linearDamping = 0.0f;
    props.angularDamping = 0.0f;

    ge::RigidBody* body = engine.createBoxBody(glm::vec3(0.5f), glm::mat4(1.0f), props);
    engine.update(kFixedTimeStep, 4);

    const float expectedY = -9.81f * kFixedTimeStep * kFixedTimeStep;
    requireNear(body->getPosition().y, expectedY, 1e-5f,
                "PhysicsEngine update must pass elapsed time separately from max substeps");
}

void testCannonAccumulatorRemainsBoundedAfterSubstepLimit() {
    ge::cannon::World world;
    world.step(kFixedTimeStep, 1.0f, 1);

    if (world.accumulator < 0.0f || world.accumulator >= kFixedTimeStep) {
        throw std::runtime_error("Cannon accumulator must remain below one fixed timestep");
    }
}

void testRotatedBodyUpdatesWorldInverseInertia() {
    ge::RigidBodyProps props;
    props.useGravity = false;
    props.mass = 1.0f;

    ge::cannon::Body body(props, glm::mat4(1.0f));
    body.addShape(std::make_unique<ge::BoxCollider>(glm::vec3(1.0f, 2.0f, 3.0f)));
    body.quaternion.setFromAxisAngle(ge::cannon::Vec3(0.0f, 0.0f, 1.0f), glm::radians(90.0f));
    body.updateMassProperties();

    requireNear(body.invInertiaWorld.elements[0], body.invInertia.y, 1e-5f,
                "Rotating a body must rotate its inverse inertia tensor");
    requireNear(body.invInertiaWorld.elements[4], body.invInertia.x, 1e-5f,
                "Rotating a body must swap the rotated box inertia axes");
}

void testCenterOfMassOffsetKeepsCannonShapeAtShapeOrigin() {
    ge::PhysicsWorld world;
    ge::RigidBodyProps dynamicProps;
    dynamicProps.useGravity = false;
    dynamicProps.centerOfMassOffset = glm::vec3(0.5f, 0.0f, 0.0f);

    ge::RigidBodyProps staticProps = dynamicProps;
    staticProps.mass = 0.0f;
    staticProps.centerOfMassOffset = glm::vec3(0.0f);

    world.addBody(makeBoxBody(dynamicProps));
    world.addBody(makeBoxBody(staticProps,
                              glm::translate(glm::mat4(1.0f), glm::vec3(1.2f, 0.0f, 0.0f))));

    int beginCount = 0;
    world.setCollisionCallback([&beginCount](const ge::PhysicsWorld::CollisionEvent& event) {
        if (event.phase == ge::PhysicsWorld::CollisionPhase::Begin) {
            ++beginCount;
        }
    });
    world.step(kFixedTimeStep, 4);

    if (beginCount != 0) {
        throw std::runtime_error("A center-of-mass offset must not move the Cannon shape origin");
    }
}

void testZeroRadiusSphereHasFiniteInverseInertia() {
    ge::RigidBodyProps props;
    props.useGravity = false;
    ge::cannon::Body body(props, glm::mat4(1.0f));
    body.addShape(std::make_unique<ge::SphereCollider>(0.0f));

    if (body.invInertia.x != 0.0f || body.invInertia.y != 0.0f || body.invInertia.z != 0.0f) {
        throw std::runtime_error("A zero-radius sphere must have zero inverse inertia");
    }
}

void testKinematicBodySynchronizesVelocityMotion() {
    ge::PhysicsWorld world;
    ge::RigidBodyProps props;
    props.isKinematic = true;
    props.useGravity = false;

    ge::RigidBody* body = world.addBody(makeBoxBody(props));
    body->setVelocity(glm::vec3(3.0f, 0.0f, 0.0f));
    world.step(kFixedTimeStep, 4);

    requireNear(body->getPosition().x, 3.0f * kFixedTimeStep, 1e-5f,
                "Kinematic Cannon motion must synchronize to RigidBody");
}

void testContactManifoldsRemainAvailableAfterStep() {
    ge::PhysicsWorld world;
    ge::RigidBodyProps dynamicProps;
    dynamicProps.useGravity = false;
    dynamicProps.linearDamping = 0.0f;
    dynamicProps.angularDamping = 0.0f;

    ge::RigidBodyProps staticProps = dynamicProps;
    staticProps.mass = 0.0f;

    world.addBody(makeBoxBody(dynamicProps));
    world.addBody(makeBoxBody(staticProps));

    int callbackCount = 0;
    world.setContactManifoldCallback([&callbackCount](const ge::ContactManifold& manifold) {
        if (manifold.isColliding && !manifold.contacts.empty()) {
            ++callbackCount;
        }
    });

    world.step(kFixedTimeStep, 4);

    if (world.getContactManifolds().empty()) {
        throw std::runtime_error("An overlapping pair must produce a contact manifold after stepping");
    }
    if (callbackCount != 1) {
        throw std::runtime_error("The contact manifold callback must run once for an overlapping pair");
    }
}

void testTriggerOverlapEmitsCollisionEvent() {
    ge::PhysicsWorld world;
    ge::RigidBodyProps dynamicProps;
    dynamicProps.useGravity = false;

    ge::RigidBodyProps triggerProps;
    triggerProps.mass = 0.0f;
    triggerProps.isTrigger = true;

    world.addBody(makeBoxBody(dynamicProps));
    world.addBody(makeBoxBody(triggerProps));

    int beginCount = 0;
    world.setCollisionCallback([&beginCount](const ge::PhysicsWorld::CollisionEvent& event) {
        if (event.phase == ge::PhysicsWorld::CollisionPhase::Begin) {
            ++beginCount;
        }
    });

    world.step(kFixedTimeStep, 4);

    if (beginCount != 1) {
        throw std::runtime_error("An overlapping trigger must emit one collision begin event");
    }
}

void testRotatedBoxesSeparatedOnEdgeAxisDoNotCollide() {
    ge::PhysicsWorld world;
    ge::RigidBodyProps dynamicProps;
    dynamicProps.useGravity = false;

    ge::RigidBodyProps staticProps = dynamicProps;
    staticProps.mass = 0.0f;

    const glm::vec3 edgeAxis(0.577350269f, -0.577350269f, -0.577350269f);
    const glm::mat4 firstTransform = glm::rotate(glm::mat4(1.0f), glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    glm::mat4 secondTransform = glm::rotate(glm::mat4(1.0f), glm::radians(-45.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    secondTransform = glm::translate(secondTransform, edgeAxis * 0.5f);

    world.addBody(makeBoxBody(dynamicProps, firstTransform, glm::vec3(1.0f, 0.1f, 0.1f)));
    world.addBody(makeBoxBody(staticProps, secondTransform, glm::vec3(1.0f, 0.1f, 0.1f)));

    int beginCount = 0;
    world.setCollisionCallback([&beginCount](const ge::PhysicsWorld::CollisionEvent& event) {
        if (event.phase == ge::PhysicsWorld::CollisionPhase::Begin) {
            ++beginCount;
        }
    });

    world.step(kFixedTimeStep, 4);

    if (beginCount != 0) {
        throw std::runtime_error("Boxes separated on an edge-cross-edge axis must not collide");
    }
}

void testRaycastFromInsideBoxFindsExitFace() {
    ge::PhysicsWorld world;
    ge::RigidBodyProps staticProps;
    staticProps.mass = 0.0f;
    world.addBody(makeBoxBody(staticProps));

    const ge::PhysicsWorld::RaycastResult result = world.raycast(
        glm::vec3(0.0f), glm::vec3(1.0f, 0.0f, 0.0f), 2.0f);

    if (!result.hit) {
        throw std::runtime_error("A ray starting inside a box must hit its exit face");
    }
    requireNear(result.fraction, 0.25f, 1e-5f,
                "The inside-box raycast must report the forward exit distance");
}

void testInterpolatedTransformUsesPreviousPhysicsState() {
    ge::PhysicsWorld world;
    ge::RigidBodyProps props;
    props.linearDamping = 0.0f;
    props.angularDamping = 0.0f;

    ge::RigidBody* body = world.addBody(makeBoxBody(props));
    world.step(kFixedTimeStep, 4);

    const float expectedY = -0.5f * 9.81f * kFixedTimeStep * kFixedTimeStep;
    requireNear(glm::vec3(body->getInterpolatedTransform(0.5f)[3]).y, expectedY, 1e-5f,
                "Interpolated transforms must blend the previous and current physics positions");
}

void testInterpolatedTransformPreservesCenterOfMassOffset() {
    ge::PhysicsWorld world;
    ge::RigidBodyProps props;
    props.useGravity = false;
    props.centerOfMassOffset = glm::vec3(0.5f, 0.0f, 0.0f);

    ge::RigidBody* body = world.addBody(makeBoxBody(props));
    world.step(kFixedTimeStep, 4);

    requireNear(glm::vec3(body->getInterpolatedTransform(1.0f)[3]).x, 0.0f, 1e-5f,
                "Interpolated transforms must use the shape origin instead of the center of mass");
}

void testRaycastUsesPhysicalColliderSizeWhenTransformIsScaled() {
    ge::PhysicsWorld world;
    ge::RigidBodyProps staticProps;
    staticProps.mass = 0.0f;

    const glm::mat4 visualTransform = glm::scale(glm::mat4(1.0f), glm::vec3(2.0f));
    world.addBody(makeBoxBody(staticProps, visualTransform));

    const ge::PhysicsWorld::RaycastResult result = world.raycast(
        glm::vec3(-2.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), 4.0f);

    if (!result.hit) {
        throw std::runtime_error("A scaled visual transform must not disable the physical box raycast");
    }
    requireNear(result.fraction, 0.375f, 1e-5f,
                "Raycasts must use the collider dimensions used by simulation, not visual scale");
}

void testSphereRaycastUsesPhysicalRadiusWhenTransformIsScaled() {
    ge::PhysicsWorld world;
    ge::RigidBodyProps staticProps;
    staticProps.mass = 0.0f;

    const glm::mat4 visualTransform = glm::scale(glm::mat4(1.0f), glm::vec3(3.0f));
    auto sphere = std::make_unique<ge::RigidBody>(
        std::make_unique<ge::SphereCollider>(0.5f), visualTransform, staticProps);
    world.addBody(std::move(sphere));

    const ge::PhysicsWorld::RaycastResult result = world.raycast(
        glm::vec3(-2.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), 4.0f);

    if (!result.hit) {
        throw std::runtime_error("A scaled visual transform must not enlarge the physical sphere raycast");
    }
    requireNear(result.fraction, 0.375f, 1e-5f,
                "Sphere raycasts must use the collider radius used by simulation");
}

void testScaledVisualTransformDoesNotEnlargeBoxCollision() {
    ge::PhysicsWorld world;
    ge::RigidBodyProps dynamicProps;
    dynamicProps.useGravity = false;
    dynamicProps.linearDamping = 0.0f;
    dynamicProps.angularDamping = 0.0f;

    ge::RigidBodyProps staticProps = dynamicProps;
    staticProps.mass = 0.0f;

    const glm::mat4 scaledTransform = glm::scale(glm::mat4(1.0f), glm::vec3(4.0f));
    world.addBody(makeBoxBody(dynamicProps, scaledTransform));
    world.addBody(makeBoxBody(staticProps,
                              glm::translate(glm::mat4(1.0f), glm::vec3(1.2f, 0.0f, 0.0f))));

    int beginCount = 0;
    world.setCollisionCallback([&beginCount](const ge::PhysicsWorld::CollisionEvent& event) {
        if (event.phase == ge::PhysicsWorld::CollisionPhase::Begin) {
            ++beginCount;
        }
    });

    world.step(kFixedTimeStep, 4);

    if (beginCount != 0) {
        throw std::runtime_error("A visual transform scale must not enlarge the simulated box collider");
    }
}

void testHeadOnCubeCollisionDoesNotAmplifyVelocity() {
    ge::PhysicsWorld world;
    ge::RigidBodyProps props;
    props.useGravity = false;
    props.linearDamping = 0.0f;
    props.angularDamping = 0.0f;
    props.restitution = 0.0f;

    const glm::mat4 leftTransform = glm::translate(glm::mat4(1.0f), glm::vec3(-0.9f, 0.0f, 0.0f));
    const glm::mat4 rightTransform = glm::translate(glm::mat4(1.0f), glm::vec3(0.9f, 0.0f, 0.0f));
    ge::RigidBody* left = world.addBody(makeBoxBody(props, leftTransform));
    ge::RigidBody* right = world.addBody(makeBoxBody(props, rightTransform));

    world.step(kFixedTimeStep, 4);

    const float leftSpeed = glm::length(left->getVelocity());
    const float rightSpeed = glm::length(right->getVelocity());
    if (leftSpeed > 0.5f || rightSpeed > 0.5f) {
        throw std::runtime_error("A resting-restitution cube collision created excessive speed: left=" +
                                 std::to_string(leftSpeed) + ", right=" + std::to_string(rightSpeed));
    }
}

void testHeadOnCubeCollisionRespectsRestitution() {
    ge::PhysicsWorld world;
    ge::RigidBodyProps props;
    props.useGravity = false;
    props.linearDamping = 0.0f;
    props.angularDamping = 0.0f;
    props.restitution = 0.5f;

    const glm::mat4 leftTransform = glm::translate(glm::mat4(1.0f), glm::vec3(-0.9f, 0.0f, 0.0f));
    const glm::mat4 rightTransform = glm::translate(glm::mat4(1.0f), glm::vec3(0.9f, 0.0f, 0.0f));
    ge::RigidBody* left = world.addBody(makeBoxBody(props, leftTransform));
    ge::RigidBody* right = world.addBody(makeBoxBody(props, rightTransform));
    left->setVelocity(glm::vec3(1.0f, 0.0f, 0.0f));
    right->setVelocity(glm::vec3(-1.0f, 0.0f, 0.0f));

    world.step(kFixedTimeStep, 4);

    if (left->getVelocity().x < -1.5f || right->getVelocity().x > 1.5f) {
        throw std::runtime_error("Cube collision response must remain bounded by restitution");
    }
}

void testHighSpeedCubeCollisionDoesNotCreateImpulseSpike() {
    ge::PhysicsWorld world;
    ge::RigidBodyProps props;
    props.useGravity = false;
    props.linearDamping = 0.0f;
    props.angularDamping = 0.0f;
    props.restitution = 0.1f;

    const glm::mat4 leftTransform = glm::translate(glm::mat4(1.0f), glm::vec3(-0.9f, 0.0f, 0.0f));
    const glm::mat4 rightTransform = glm::translate(glm::mat4(1.0f), glm::vec3(0.9f, 0.0f, 0.0f));
    ge::RigidBody* left = world.addBody(makeBoxBody(props, leftTransform));
    ge::RigidBody* right = world.addBody(makeBoxBody(props, rightTransform));
    left->setVelocity(glm::vec3(10.0f, 0.0f, 0.0f));
    right->setVelocity(glm::vec3(-10.0f, 0.0f, 0.0f));

    world.step(kFixedTimeStep, 4);

    if (glm::length(left->getVelocity()) > 12.0f || glm::length(right->getVelocity()) > 12.0f) {
        throw std::runtime_error("A high-speed cube collision must not produce an impulse spike");
    }
}

void testRepeatedRestingContactDoesNotReuseStaleImpulse() {
    ge::PhysicsWorld world;
    ge::RigidBodyProps props;
    props.useGravity = false;
    props.linearDamping = 0.0f;
    props.angularDamping = 0.0f;
    props.restitution = 0.0f;

    const glm::mat4 leftTransform = glm::translate(glm::mat4(1.0f), glm::vec3(-0.9f, 0.0f, 0.0f));
    const glm::mat4 rightTransform = glm::translate(glm::mat4(1.0f), glm::vec3(0.9f, 0.0f, 0.0f));
    ge::RigidBody* left = world.addBody(makeBoxBody(props, leftTransform));
    ge::RigidBody* right = world.addBody(makeBoxBody(props, rightTransform));

    world.step(kFixedTimeStep, 4);
    world.step(kFixedTimeStep, 4);

    if (glm::length(left->getVelocity()) > 0.5f || glm::length(right->getVelocity()) > 0.5f) {
        throw std::runtime_error("Repeated resting contacts must not reuse a stale collision impulse");
    }
}

} // namespace

int main() {
    try {
        testAccumulatorAdvancesOneFixedStep();
        testPhysicsEngineUpdateUsesElapsedTimeParameter();
        testCannonAccumulatorRemainsBoundedAfterSubstepLimit();
        testRotatedBodyUpdatesWorldInverseInertia();
        testCenterOfMassOffsetKeepsCannonShapeAtShapeOrigin();
        testZeroRadiusSphereHasFiniteInverseInertia();
        testKinematicBodySynchronizesVelocityMotion();
        testContactManifoldsRemainAvailableAfterStep();
        testTriggerOverlapEmitsCollisionEvent();
        testRotatedBoxesSeparatedOnEdgeAxisDoNotCollide();
        testRaycastFromInsideBoxFindsExitFace();
        testInterpolatedTransformUsesPreviousPhysicsState();
        testInterpolatedTransformPreservesCenterOfMassOffset();
        testRaycastUsesPhysicalColliderSizeWhenTransformIsScaled();
        testSphereRaycastUsesPhysicalRadiusWhenTransformIsScaled();
        testScaledVisualTransformDoesNotEnlargeBoxCollision();
        testHeadOnCubeCollisionDoesNotAmplifyVelocity();
        testHeadOnCubeCollisionRespectsRestitution();
        testHighSpeedCubeCollisionDoesNotCreateImpulseSpike();
        testRepeatedRestingContactDoesNotReuseStaleImpulse();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}