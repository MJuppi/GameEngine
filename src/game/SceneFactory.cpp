#include "game/SceneFactory.h"
#include "engine/Engine.h"
#include "engine/physics/PhysicsEngine.h"
#include "engine/ui/UIManager.h"
#include "engine/ui/Label.h"
#include "engine/FrameTimer.h"

#include <glm/gtc/matrix_transform.hpp>
#include <iomanip>
#include <sstream>
#include <vector>

namespace ge {

namespace {

RigidBody* findBodyByName(Engine& engine, const std::string& name) {
    const auto& bodies = engine.getPhysicsEngine().getWorld().getBodies();
    for (const auto& body : bodies) {
        if (body && body->getName() == name) {
            return body.get();
        }
    }
    return nullptr;
}

} // namespace

void SceneFactory::configureTestLevel(Level& level) {
    // Configure lighting
    auto& lights = level.getSceneLights();

    // Ambient
    lights.ambient.color = {1.0f, 1.0f, 1.0f, 1.0f};
    lights.ambient.intensity = 0.1f;

    // Directional
    lights.directional.direction = glm::normalize(glm::vec4(0.5f, 1.0f, 0.5f, 0.0f));
    lights.directional.color = {1.0f, 0.95f, 0.8f, 1.0f};
    lights.directional.intensity = 0.5f;

    // Point Lights
    lights.pointLightCount = 2;

    // Light 1: Warm orange
    lights.pointLights[0].position = {0.0f, 4.0f, 0.0f, 1.0f};
    lights.pointLights[0].color = {1.0f, 0.6f, 0.2f, 1.0f};
    lights.pointLights[0].parameters = {1.0f, 0.09f, 0.032f, 2.0f};

    // Light 2: Cool blue
    lights.pointLights[1].position = {5.0f, 2.0f, 5.0f, 1.0f};
    lights.pointLights[1].color = {0.2f, 0.4f, 1.0f, 1.0f};
    lights.pointLights[1].parameters = {1.0f, 0.09f, 0.032f, 1.5f};

    // Stacking test for stability
    for (int i = 0; i < 5; ++i) {
        level.add("test_cube").name("Stack_" + std::to_string(i))
             .at(0.0f, 2.0f + i * 1.5f, 0.0f).extents({1.0f, 1.0f, 1.0f}).mass(1.0f).asActive();
    }

    // Trigger test: A ghost cube that doesn't block
    RigidBodyProps triggerProps;
    triggerProps.isTrigger = true;
    triggerProps.isKinematic = true;
    level.add("test_cube").name("GhostCube").at(5.0f, 2.0f, 0.0f)
         .props(triggerProps).asActive();

    // Example of using the new material refactor
    Material redMat = makeDefaultMaterial("RedMaterial");
    redMat.diffuse = {1.0f, 0.0f, 0.0f};
    redMat.shininess = 64.0f;
    level.add("suomi").name("Suomi").at(2.0f, 6.0f, 0.0f).extents({5.0f, 5.0f, 5.0f}).asVisual();

    // Add static ground
    level.add("").name("Ground").at(0.0f, -4.0f, 0.0f).extents({50.0f, 2.0f, 50.0f}).asStatic();
}

void SceneFactory::configureConstraintParityLevel(Level& level) {
    auto& lights = level.getSceneLights();

    lights.ambient.color = {1.0f, 1.0f, 1.0f, 1.0f};
    lights.ambient.intensity = 0.14f;
    lights.directional.direction = glm::normalize(glm::vec4(0.4f, 1.0f, 0.3f, 0.0f));
    lights.directional.color = {1.0f, 0.96f, 0.9f, 1.0f};
    lights.directional.intensity = 0.55f;
    lights.pointLightCount = 2;

    lights.pointLights[0].position = {-6.0f, 6.0f, 4.0f, 1.0f};
    lights.pointLights[0].color = {0.95f, 0.7f, 0.25f, 1.0f};
    lights.pointLights[0].parameters = {1.0f, 0.09f, 0.032f, 2.0f};

    lights.pointLights[1].position = {5.0f, 5.0f, -3.0f, 1.0f};
    lights.pointLights[1].color = {0.25f, 0.45f, 1.0f, 1.0f};
    lights.pointLights[1].parameters = {1.0f, 0.09f, 0.032f, 1.5f};

    level.add("test_cube").name("Ground").at(0.0f, -4.0f, 0.0f).extents({50.0f, 2.0f, 50.0f}).asStatic();

    level.add("test_cube").name("HingeAnchor").at(-4.0f, 1.0f, 0.0f).extents({0.5f, 2.0f, 0.5f}).asStatic();
    level.add("test_cube").name("HingeDoor").at(-2.0f, 1.0f, 0.0f).extents({1.0f, 2.0f, 0.25f}).mass(2.0f).asActive();

    level.add("test_cube").name("ConeRoot").at(4.0f, 3.0f, 0.0f).extents({0.75f, 0.75f, 0.75f}).mass(2.0f).asActive();
    level.add("test_cube").name("ConeTip").at(4.0f, 1.0f, 0.0f).extents({0.75f, 0.75f, 0.75f}).mass(1.5f).asActive();

    level.add("test_cube").name("BalanceBlock").at(0.0f, 6.0f, 0.0f).extents({0.75f, 0.75f, 0.75f}).mass(1.0f).asActive();
}

void SceneFactory::configurePairOrderingLevel(Level& level) {
    auto& lights = level.getSceneLights();

    lights.ambient.color = {1.0f, 1.0f, 1.0f, 1.0f};
    lights.ambient.intensity = 0.12f;
    lights.directional.direction = glm::normalize(glm::vec4(0.35f, 1.0f, 0.25f, 0.0f));
    lights.directional.color = {1.0f, 0.95f, 0.88f, 1.0f};
    lights.directional.intensity = 0.5f;
    lights.pointLightCount = 1;
    lights.pointLights[0].position = {0.0f, 5.0f, 5.0f, 1.0f};
    lights.pointLights[0].color = {0.4f, 0.7f, 1.0f, 1.0f};
    lights.pointLights[0].parameters = {1.0f, 0.09f, 0.032f, 2.0f};

    level.add("test_cube").name("PairAnchorA").at(-4.0f, 0.0f, 0.0f).extents({1.0f, 1.0f, 1.0f}).asStatic();
    level.add("test_cube").name("PairBlockA").at(-4.0f, 0.0f, 0.0f).extents({1.0f, 1.0f, 1.0f}).mass(1.0f).asActive();
    level.add("test_cube").name("PairAnchorB").at(4.0f, 0.0f, 0.0f).extents({1.0f, 1.0f, 1.0f}).asStatic();
    level.add("test_cube").name("PairBlockB").at(4.0f, 0.0f, 0.0f).extents({1.0f, 1.0f, 1.0f}).mass(1.0f).asActive();
}

void SceneFactory::setupConstraintParityPhysics(Engine& engine) {
    auto* hingeAnchor = findBodyByName(engine, "HingeAnchor");
    auto* hingeDoor = findBodyByName(engine, "HingeDoor");
    auto* coneRoot = findBodyByName(engine, "ConeRoot");
    auto* coneTip = findBodyByName(engine, "ConeTip");

    if (hingeAnchor && hingeDoor) {
        auto* hinge = engine.getPhysicsEngine().addHingeConstraint(
            hingeAnchor,
            hingeDoor,
            {1.0f, 0.0f, 0.0f},
            {-1.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            20.0f,
            false);

        if (hinge) {
            hinge->enableMotor();
            hinge->setMotorSpeed(0.35f);
            hinge->setMotorMaxForce(18.0f);
        }
    }

    if (coneRoot && coneTip) {
        engine.getPhysicsEngine().addConeTwistConstraint(
            coneRoot,
            coneTip,
            {0.0f, -0.75f, 0.0f},
            {0.0f, 0.75f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            0.55f,
            0.25f,
            40.0f,
            false);
    }
}

void SceneFactory::setupPairOrderingPhysics(Engine& engine) {
    struct PairOrderingState {
        std::vector<std::string> observed;
        std::vector<std::string> expected;
        bool validated = false;
        bool passed = false;
        std::string message = "Pair ordering: waiting for manifolds...";
    };

    auto state = std::make_shared<PairOrderingState>();
    state->expected = {
        "PairAnchorA <-> PairBlockA",
        "PairAnchorB <-> PairBlockB",
    };

    auto label = std::make_shared<Label>();
    label->setPosition({0.02f, 0.05f});
    label->setSize({0.9f, 0.1f});
    label->setColor({0.9f, 0.9f, 0.15f, 1.0f});
    label->setFontSize(1.15f);
    label->setText(state->message);
    label->setOnUpdate([state](Label& outLabel, float /*deltaTime*/) {
        std::ostringstream ss;
        ss << state->message;
        if (!state->observed.empty()) {
            ss << "\nobserved:";
            for (const auto& pair : state->observed) {
                ss << "\n- " << pair;
            }
        }
        outLabel.setText(ss.str());
    });
    engine.getUIManager().addElement(label);

    engine.getPhysicsEngine().getWorld().setContactManifoldCallback(
        [state](const ContactManifold& manifold) {
            if (state->validated || !manifold.bodyA || !manifold.bodyB) {
                return;
            }

            const std::string bodyNameA = manifold.bodyA->getName().empty() ? std::string("BodyA") : manifold.bodyA->getName();
            const std::string bodyNameB = manifold.bodyB->getName().empty() ? std::string("BodyB") : manifold.bodyB->getName();
            state->observed.push_back(bodyNameA + " <-> " + bodyNameB);

            if (state->observed.size() != state->expected.size()) {
                return;
            }

            state->passed = (state->observed == state->expected);
            state->validated = true;
            std::ostringstream ss;
            ss << "Pair ordering: " << (state->passed ? "PASS" : "FAIL");
            if (!state->passed) {
                ss << "\nexpected:";
                for (const auto& pair : state->expected) {
                    ss << "\n- " << pair;
                }
            }
            state->message = ss.str();
            std::cout << state->message << '\n';
        });
}

void SceneFactory::setupUI(Engine& engine) {
    auto fpsLabel = std::make_shared<Label>();
    fpsLabel->setPosition({0.85f, 0.05f});
    fpsLabel->setSize({0.1f, 0.03f});
    fpsLabel->setColor({1.0f, 1.0f, 0.0f, 1.0f}); // Yellow
    fpsLabel->setFontSize(1.5f);
    fpsLabel->setText("FPS: 0");

    // Capture engine by reference if it lives long enough (it does, it's the main engine)
    // Or better, capture the FrameTimer if possible.
    // Engine has no direct getter for FrameTimer, but we can capture engine reference.
    fpsLabel->setOnUpdate([&engine](Label& label, float /*deltaTime*/) {
        std::stringstream ss;
        ss << "FPS: " << std::fixed << std::setprecision(1) << engine.getFPS();
        label.setText(ss.str());
    });

    engine.getUIManager().addElement(fpsLabel);

    // Debug label for the active regression bodies, with a fallback to the old stack test.
    auto cubeDebugLabel = std::make_shared<Label>();
    cubeDebugLabel->setPosition({0.02f, 0.05f});
    cubeDebugLabel->setSize({0.8f, 0.08f});
    cubeDebugLabel->setColor({0.0f, 1.0f, 1.0f, 1.0f});
    cubeDebugLabel->setFontSize(1.2f);
    cubeDebugLabel->setText("Constraint parity: searching...");

    cubeDebugLabel->setOnUpdate([&engine](Label& label, float /*deltaTime*/) {
        const RigidBody* hingeDoor = findBodyByName(engine, "HingeDoor");
        const RigidBody* coneTip = findBodyByName(engine, "ConeTip");
        const RigidBody* fallback = findBodyByName(engine, "Stack_0");

        if (!hingeDoor && !coneTip && !fallback) {
            label.setText("Constraint parity: not found");
            return;
        }

        std::ostringstream ss;
        auto appendBody = [&ss](const char* name, const RigidBody* body) {
            if (!body) {
                return;
            }

            const glm::vec3 pos = body->getPosition();
            const glm::vec3 vel = body->getVelocity();
            const glm::vec3 ang = body->getAngularVelocity();

            ss << name;
            ss << "\npos=" << std::fixed << std::setprecision(2)
               << pos.x << "," << pos.y << "," << pos.z;
            ss << "\nvel=" << vel.x << "," << vel.y << "," << vel.z;
            ss << "\nang=" << ang.x << "," << ang.y << "," << ang.z << "\n";
        };

        appendBody("HingeDoor", hingeDoor);
        appendBody("ConeTip", coneTip);
        if (!hingeDoor && !coneTip) {
            appendBody("Stack_0", fallback);
        }
        label.setText(ss.str());
    });
    engine.getUIManager().addElement(cubeDebugLabel);
}

RigidBodyProps SceneFactory::makeDynamicBoxProps(float mass, float friction, float restitution) {
    RigidBodyProps props;
    props.mass = mass;
    props.friction = friction; // dynamic friction
    props.staticFriction = std::min(1.0f, friction + 0.2f);
    props.rollingFriction = 0.5f;
    props.restitution = restitution;
    props.linearDamping = 0.5f;
    props.angularDamping = 0.8f;
    return props;
}

RigidBodyProps SceneFactory::makeGroundProps() {
    RigidBodyProps props;
    props.mass = 0.0f;
    props.isKinematic = true;
    props.useGravity = false;
    return props;
}

RigidBodyProps SceneFactory::makeProjectileProps() {
    RigidBodyProps props;
    props.mass = 1.0f;
    return props;
}

/// @brief Spawns a projectile in the physics engine with the specified properties. The projectile is created as a box-shaped rigid body with the given half extents, spawn position, and initial velocity based on the fire direction and optional velocity offset. The function returns a pointer to the created RigidBody, or nullptr if the half extents are invalid (non-positive).
/// @param engine 
/// @param spawnPosition 
/// @param fireDirection 
/// @param velocityOffset 
/// @param halfExtents 
/// @return 
RigidBody* SceneFactory::spawnProjectile(Engine& engine, const glm::vec3& spawnPosition, const glm::vec3& fireDirection, const glm::vec3& velocityOffset, const glm::vec3& halfExtents) {
    if (halfExtents.x <= 0.0f || halfExtents.y <= 0.0f || halfExtents.z <= 0.0f) {
        return nullptr;
    }

    const glm::mat4 spawnTransform = glm::translate(glm::mat4(1.0f), spawnPosition);
    RigidBody* projectile = engine.getPhysicsEngine().createActiveBoxBody(halfExtents, spawnTransform, makeProjectileProps());

    if (projectile) {
        projectile->setVelocity(fireDirection * 15.0f + velocityOffset);
        projectile->setAngularVelocity(glm::vec3(0.5f, 1.0f, 0.2f));
    }

    return projectile;
}

} // namespace ge
