#include "engine/physics/BoxCollider.h"
#include "engine/physics/PhysicsWorld.h"
#include "engine/physics/RigidBody.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {

constexpr float kFixedTimeStep = 1.0f / 60.0f;

void requireNear(float actual, float expected, float tolerance, const char* message) {
    if (std::fabs(actual - expected) > tolerance) {
        throw std::runtime_error(std::string(message) + ": expected " +
                                 std::to_string(expected) + ", got " +
                                 std::to_string(actual));
    }
}

std::unique_ptr<ge::RigidBody> makeBoxBody(const ge::RigidBodyProps& props) {
    return std::make_unique<ge::RigidBody>(
        std::make_unique<ge::BoxCollider>(glm::vec3(0.5f)),
        glm::mat4(1.0f),
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

} // namespace

int main() {
    try {
        testAccumulatorAdvancesOneFixedStep();
        testKinematicBodySynchronizesVelocityMotion();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }

    return 0;
}