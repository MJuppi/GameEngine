#include "engine/physics/BoxCollider.h"
#include "engine/physics/PhysicsWorld.h"
#include "engine/physics/RigidBody.h"

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
        testKinematicBodySynchronizesVelocityMotion();
        testContactManifoldsRemainAvailableAfterStep();
        testTriggerOverlapEmitsCollisionEvent();
        testRotatedBoxesSeparatedOnEdgeAxisDoNotCollide();
        testRaycastFromInsideBoxFindsExitFace();
        testInterpolatedTransformUsesPreviousPhysicsState();
        testInterpolatedTransformPreservesCenterOfMassOffset();
        testRaycastUsesPhysicalColliderSizeWhenTransformIsScaled();
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