#include "game/SceneFactory.h"
#include "engine/Engine.h"
#include "engine/physics/PhysicsEngine.h"
#include "engine/ui/UIManager.h"
#include "engine/ui/Label.h"
#include "engine/ui/Button.h"
#include "engine/FrameTimer.h"

#include <glm/gtc/matrix_transform.hpp>
#include <iomanip>
#include <sstream>
#include <utility>
#include <vector>
#include <memory>

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

void addBoxFromHalfExtents(Level& level,
                           std::string name,
                           const glm::vec3& location,
                           const glm::vec3& halfExtents,
                           ObjectType type,
                           const RigidBodyProps& props = {},
                           Material material = {}) {
    level.addBox(std::move(name), halfExtents * 2.0f, location, type, props, std::move(material));
}

RigidBodyProps boxPropsWithMass(float mass = 1.0f) {
    RigidBodyProps props;
    props.mass = mass;
    return props;
}

} // namespace

void SceneFactory::configureCombatMission(Level& level) {
    auto& lights = level.getSceneLights();
    lights.ambient.color = {0.42f, 0.52f, 0.68f, 1.0f};
    lights.ambient.intensity = 0.28f;
    lights.directional.direction = glm::normalize(glm::vec4(-0.35f, 1.0f, 0.25f, 0.0f));
    lights.directional.color = {1.0f, 0.86f, 0.68f, 1.0f};
    lights.directional.intensity = 0.85f;
    lights.pointLightCount = 1;
    lights.pointLights[0].position = {0.0f, 40.0f, 0.0f, 1.0f};
    lights.pointLights[0].color = {0.35f, 0.55f, 1.0f, 1.0f};
    lights.pointLights[0].parameters = {1.0f, 0.03f, 0.01f, 2.0f};

    RigidBodyProps aircraft;
    aircraft.mass = 1.0f;
    aircraft.useGravity = false;
    aircraft.linearDamping = 0.02f;
    aircraft.angularDamping = 0.9f;

    addBoxFromHalfExtents(level, "Terrain", {0.0f, -1.0f, 0.0f},
                          {150.0f, 1.0f, 150.0f}, ObjectType::Static);
    addBoxFromHalfExtents(level, "PlayerJet", {0.0f, 30.0f, 20.0f},
                          {1.8f, 0.45f, 3.2f}, ObjectType::Active, aircraft);
    addBoxFromHalfExtents(level, "EnemyJet0", {-34.0f, 34.0f, -75.0f},
                          {1.6f, 0.4f, 2.8f}, ObjectType::Active, aircraft);
    addBoxFromHalfExtents(level, "EnemyJet1", {38.0f, 42.0f, -120.0f},
                          {1.6f, 0.4f, 2.8f}, ObjectType::Active, aircraft);
    addBoxFromHalfExtents(level, "EnemyJet2", {-62.0f, 27.0f, -155.0f},
                          {1.6f, 0.4f, 2.8f}, ObjectType::Active, aircraft);
    addBoxFromHalfExtents(level, "EnemyJet3", {74.0f, 50.0f, -190.0f},
                          {1.6f, 0.4f, 2.8f}, ObjectType::Active, aircraft);

    for (int i = 0; i < 12; ++i) {
        const float x = static_cast<float>((i % 4) * 38 - 57);
        const float z = static_cast<float>((i / 4) * -55 - 35);
        addBoxFromHalfExtents(level, "TerrainMarker" + std::to_string(i), {x, 0.0f, z},
                              {1.5f, 4.0f, 1.5f}, ObjectType::Static);
    }
}

void SceneFactory::configurePhysicsShowcaseLevel(Level& level) {
    auto& lights = level.getSceneLights();
    lights.ambient.color = {0.72f, 0.82f, 1.0f, 1.0f};
    lights.ambient.intensity = 0.24f;
    lights.directional.direction = glm::normalize(glm::vec4(-0.4f, 1.0f, 0.3f, 0.0f));
    lights.directional.color = {1.0f, 0.94f, 0.82f, 1.0f};
    lights.directional.intensity = 0.7f;
    lights.pointLightCount = 2;
    lights.pointLights[0].position = {-8.0f, 10.0f, 8.0f, 1.0f};
    lights.pointLights[0].color = {0.2f, 0.55f, 1.0f, 1.0f};
    lights.pointLights[0].parameters = {1.0f, 0.03f, 0.01f, 2.0f};
    lights.pointLights[1].position = {10.0f, 12.0f, -8.0f, 1.0f};
    lights.pointLights[1].color = {1.0f, 0.35f, 0.16f, 1.0f};
    lights.pointLights[1].parameters = {1.0f, 0.03f, 0.01f, 2.0f};

    addBoxFromHalfExtents(level, "ShowcaseGround", {0.0f, -1.0f, 0.0f},
                          {30.0f, 1.0f, 25.0f}, ObjectType::Static);

    for (int i = 0; i < 5; ++i) {
        addBoxFromHalfExtents(level, "Stack_" + std::to_string(i),
            {-12.0f, 0.8f + static_cast<float>(i) * 1.62f, 0.0f},
            {0.8f, 0.8f, 0.8f}, ObjectType::Active);
    }

    RigidBodyProps lowFriction = makeDynamicBoxProps(1.0f, 0.03f, 0.0f);
    lowFriction.staticFriction = 0.04f;
    lowFriction.linearDamping = 0.0f;
    Material lowFrictionMaterial = makeDefaultMaterial("ShowcaseLowFriction");
    lowFrictionMaterial.diffuse = {0.15f, 0.45f, 1.0f};
    RigidBodyProps highFriction = makeDynamicBoxProps(1.0f, 0.9f, 0.0f);
    highFriction.staticFriction = 1.0f;
    highFriction.linearDamping = 0.0f;
    Material highFrictionMaterial = makeDefaultMaterial("ShowcaseHighFriction");
    highFrictionMaterial.diffuse = {1.0f, 0.2f, 0.15f};
    addBoxFromHalfExtents(level, "LowFrictionSlider", {-13.0f, 0.55f, -10.0f},
                          {0.55f, 0.55f, 0.55f}, ObjectType::Active, lowFriction, lowFrictionMaterial);
    addBoxFromHalfExtents(level, "HighFrictionSlider", {-13.0f, 0.55f, -6.0f},
                          {0.55f, 0.55f, 0.55f}, ObjectType::Active, highFriction, highFrictionMaterial);

    RigidBodyProps lowBounce = makeDynamicBoxProps(1.0f, 0.4f, 0.05f);
    lowBounce.linearDamping = 0.0f;
    Material lowBounceMaterial = makeDefaultMaterial("ShowcaseLowBounce");
    lowBounceMaterial.diffuse = {1.0f, 0.48f, 0.12f};
    RigidBodyProps highBounce = makeDynamicBoxProps(1.0f, 0.4f, 0.9f);
    highBounce.linearDamping = 0.0f;
    Material highBounceMaterial = makeDefaultMaterial("ShowcaseHighBounce");
    highBounceMaterial.diffuse = {0.2f, 0.9f, 0.35f};
    addBoxFromHalfExtents(level, "LowBounceBlock", {6.0f, 9.0f, -5.0f},
                          {0.7f, 0.7f, 0.7f}, ObjectType::Active, lowBounce, lowBounceMaterial);
    addBoxFromHalfExtents(level, "HighBounceBlock", {11.0f, 9.0f, -5.0f},
                          {0.7f, 0.7f, 0.7f}, ObjectType::Active, highBounce, highBounceMaterial);

    RigidBodyProps triggerProps;
    triggerProps.mass = 0.1f;
    triggerProps.useGravity = false;
    triggerProps.isTrigger = true;
    triggerProps.friction = 0.0f;
    triggerProps.staticFriction = 0.0f;
    Material triggerMaterial = makeDefaultMaterial("ShowcaseTrigger");
    triggerMaterial.diffuse = {0.1f, 0.85f, 0.95f};
    addBoxFromHalfExtents(level, "SensorGate", {-3.0f, 0.6f, -10.0f},
                          {0.15f, 0.6f, 1.2f}, ObjectType::Active, triggerProps, triggerMaterial);

    RigidBodyProps kinematicProps;
    kinematicProps.mass = 1.0f;
    kinematicProps.isKinematic = true;
    kinematicProps.useGravity = false;
    kinematicProps.linearDamping = 0.0f;
    Material platformMaterial = makeDefaultMaterial("ShowcaseKinematic");
    platformMaterial.diffuse = {0.55f, 0.3f, 0.95f};
    addBoxFromHalfExtents(level, "KinematicPlatform", {0.0f, 0.0f, 6.0f},
                          {2.5f, 0.25f, 1.5f}, ObjectType::Active, kinematicProps, platformMaterial);
    addBoxFromHalfExtents(level, "PlatformRider", {0.0f, 1.0f, 6.0f},
                          {0.6f, 0.6f, 0.6f}, ObjectType::Active);

    addBoxFromHalfExtents(level, "HingePost", {-10.0f, 2.5f, 7.0f},
                          {0.25f, 2.5f, 0.25f}, ObjectType::Static);
    Material hingeDoorMaterial = makeDefaultMaterial("ShowcaseHingeDoor");
    hingeDoorMaterial.diffuse = {0.95f, 0.75f, 0.2f};
    RigidBodyProps hingeDoorProps = makeDynamicBoxProps(2.0f);
    addBoxFromHalfExtents(level, "HingeDoor", {-8.0f, 2.5f, 7.0f},
                          {1.75f, 2.0f, 0.25f}, ObjectType::Active, hingeDoorProps, hingeDoorMaterial);

    addBoxFromHalfExtents(level, "DistanceAnchor", {13.0f, 10.0f, 5.0f},
                          {0.35f, 0.35f, 0.35f}, ObjectType::Static);
    addBoxFromHalfExtents(level, "DistanceWeight", {13.0f, 6.0f, 5.0f},
                          {0.55f, 0.55f, 0.55f}, ObjectType::Active, boxPropsWithMass(1.5f));
}

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
        addBoxFromHalfExtents(level, "Stack_" + std::to_string(i),
            {0.0f, 2.0f + i * 2.1f, 0.0f}, {1.0f, 1.0f, 1.0f},
            ObjectType::Active);
    }

    // Trigger test: A ghost cube that doesn't block
    RigidBodyProps triggerProps;
    triggerProps.isTrigger = true;
    triggerProps.isKinematic = true;
    addBoxFromHalfExtents(level, "GhostCube", {5.0f, 2.0f, 0.0f},
                          {0.5f, 0.5f, 0.5f}, ObjectType::Active, triggerProps);

    // Example of using the new material refactor
    Material redMat = makeDefaultMaterial("RedMaterial");
    redMat.diffuse = {1.0f, 0.0f, 0.0f};
    redMat.shininess = 64.0f;
    level.add("suomi").name("Suomi").at(2.0f, 6.0f, 0.0f).extents({5.0f, 5.0f, 5.0f}).asVisual();

    // Add static ground
    addBoxFromHalfExtents(level, "Ground", {0.0f, -4.0f, 0.0f},
                          {50.0f, 2.0f, 50.0f}, ObjectType::Static);
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

    addBoxFromHalfExtents(level, "Ground", {0.0f, -4.0f, 0.0f},
                          {50.0f, 2.0f, 50.0f}, ObjectType::Static);
    addBoxFromHalfExtents(level, "HingeAnchor", {-4.0f, 1.0f, 0.0f},
                          {0.5f, 2.0f, 0.5f}, ObjectType::Static);
    addBoxFromHalfExtents(level, "HingeDoor", {-2.0f, 1.0f, 0.0f},
                          {1.0f, 2.0f, 0.25f}, ObjectType::Active, boxPropsWithMass(2.0f));
    addBoxFromHalfExtents(level, "ConeRoot", {4.0f, 3.0f, 0.0f},
                          {0.75f, 0.75f, 0.75f}, ObjectType::Active, boxPropsWithMass(2.0f));
    addBoxFromHalfExtents(level, "ConeTip", {4.0f, 1.0f, 0.0f},
                          {0.75f, 0.75f, 0.75f}, ObjectType::Active, boxPropsWithMass(1.5f));
    addBoxFromHalfExtents(level, "BalanceBlock", {0.0f, 6.0f, 0.0f},
                          {0.75f, 0.75f, 0.75f}, ObjectType::Active);
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

    addBoxFromHalfExtents(level, "PairAnchorA", {-4.0f, 0.0f, 0.0f},
                          {1.0f, 1.0f, 1.0f}, ObjectType::Static);
    addBoxFromHalfExtents(level, "PairBlockA", {-4.0f, 0.0f, 0.0f},
                          {1.0f, 1.0f, 1.0f}, ObjectType::Active);
    addBoxFromHalfExtents(level, "PairAnchorB", {4.0f, 0.0f, 0.0f},
                          {1.0f, 1.0f, 1.0f}, ObjectType::Static);
    addBoxFromHalfExtents(level, "PairBlockB", {4.0f, 0.0f, 0.0f},
                          {1.0f, 1.0f, 1.0f}, ObjectType::Active);
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

void SceneFactory::setupPhysicsShowcasePhysics(Engine& engine) {
    auto& physics = engine.getPhysicsEngine();
    if (RigidBody* lowFriction = findBodyByName(engine, "LowFrictionSlider")) {
        lowFriction->setVelocity({8.0f, 0.0f, 0.0f});
    }
    if (RigidBody* highFriction = findBodyByName(engine, "HighFrictionSlider")) {
        highFriction->setVelocity({8.0f, 0.0f, 0.0f});
    }
    if (RigidBody* lowBounce = findBodyByName(engine, "LowBounceBlock")) {
        lowBounce->setVelocity({0.0f, -2.0f, 0.0f});
    }
    if (RigidBody* highBounce = findBodyByName(engine, "HighBounceBlock")) {
        highBounce->setVelocity({0.0f, -2.0f, 0.0f});
    }
    if (RigidBody* platform = findBodyByName(engine, "KinematicPlatform")) {
        platform->setVelocity({1.5f, 0.0f, 0.0f});
    }

    if (RigidBody* post = findBodyByName(engine, "HingePost")) {
        if (RigidBody* door = findBodyByName(engine, "HingeDoor")) {
            if (auto* hinge = physics.addHingeConstraint(
                    post, door,
                    {0.25f, 0.0f, 0.0f}, {-1.75f, 0.0f, 0.0f},
                    {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
                    24.0f, false)) {
                hinge->enableMotor();
                hinge->setMotorSpeed(0.4f);
                hinge->setMotorMaxForce(16.0f);
            }
        }
    }

    if (RigidBody* anchor = findBodyByName(engine, "DistanceAnchor")) {
        if (RigidBody* weight = findBodyByName(engine, "DistanceWeight")) {
            physics.addDistanceConstraint(anchor, weight, 4.0f, 60.0f, false);
        }
    }
}

SceneFactory::PauseMenuBindings SceneFactory::setupUI(Engine& engine,
                                                      const LevelManager& levelManager,
                                                      size_t currentLevelIndex,
                                                      std::function<void(size_t)> onLevelSelected) {
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

    const Level* currentLevel = levelManager.getLevel(currentLevelIndex);
    const bool showPhysicsDebug = currentLevel
        && currentLevel->getName() != "PhysicsShowcase"
        && currentLevel->getName() != "CombatMission";
    auto cubeDebugLabel = std::make_shared<Label>();
    cubeDebugLabel->setPosition({0.02f, 0.05f});
    cubeDebugLabel->setSize({0.8f, 0.08f});
    cubeDebugLabel->setColor({0.0f, 1.0f, 1.0f, 1.0f});
    cubeDebugLabel->setFontSize(1.2f);
    cubeDebugLabel->setText("Constraint parity: searching...");
    cubeDebugLabel->setVisible(showPhysicsDebug);

    cubeDebugLabel->setOnUpdate([&engine, showPhysicsDebug](Label& label, float /*deltaTime*/) {
        if (!showPhysicsDebug) {
            return;
        }
        const RigidBody* hingeDoor = findBodyByName(engine, "HingeDoor");
        const RigidBody* coneTip = findBodyByName(engine, "ConeTip");
        const RigidBody* stack0 = findBodyByName(engine, "Stack_0");

        if (!hingeDoor && !coneTip && !stack0) {
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
        if (!hingeDoor && !coneTip && stack0) {
            appendBody("Stack_0", stack0);
            const glm::vec3 pos = stack0->getPosition();
            const glm::vec3 vel = stack0->getVelocity();
            std::cout << "[Stack_0] loc=(" << std::fixed << std::setprecision(3)
                      << pos.x << ", " << pos.y << ", " << pos.z
                      << ") vel=(" << vel.x << ", " << vel.y << ", " << vel.z << ")\n";

            size_t stackManifoldCount = 0;
            const ContactManifold* firstStackManifold = nullptr;
            const auto& manifolds = engine.getPhysicsEngine().getWorld().getContactManifolds();
            for (const auto& manifold : manifolds) {
                if (!manifold.bodyA || !manifold.bodyB) {
                    continue;
                }

                const bool involvesStack0 = manifold.bodyA->getName() == "Stack_0" || manifold.bodyB->getName() == "Stack_0";
                if (!involvesStack0) {
                    continue;
                }

                ++stackManifoldCount;
                if (!firstStackManifold) {
                    firstStackManifold = &manifold;
                }
            }

            std::cout << "[StackProbe] manifolds=" << stackManifoldCount;
            if (firstStackManifold) {
                const std::string manifoldNameA = firstStackManifold->bodyA->getName().empty() ? std::string("BodyA") : firstStackManifold->bodyA->getName();
                const std::string manifoldNameB = firstStackManifold->bodyB->getName().empty() ? std::string("BodyB") : firstStackManifold->bodyB->getName();
                std::cout << " pair=" << manifoldNameA << "<->" << manifoldNameB
                          << " contacts=" << firstStackManifold->contacts.size()
                          << " normal=(" << firstStackManifold->normal.x << ", " << firstStackManifold->normal.y << ", " << firstStackManifold->normal.z << ")";

                if (!firstStackManifold->contacts.empty()) {
                    const Contact& c = firstStackManifold->contacts.front();
                    std::cout << " point=(" << c.point.x << ", " << c.point.y << ", " << c.point.z << ")"
                              << " depth=" << c.depth
                              << " nImpulse=" << c.normalImpulse
                              << " tImpulse=" << c.tangentImpulse;
                }
            }
            std::cout << '\n';
        }
        label.setText(ss.str());
    });
    engine.getUIManager().addElement(cubeDebugLabel);

    auto menuVisible = std::make_shared<bool>(false);
    auto menuElements = std::make_shared<std::vector<std::shared_ptr<UIElement>>>();

    auto backdrop = std::make_shared<Button>();
    backdrop->setPosition({0.0f, 0.0f});
    backdrop->setSize({1.0f, 1.0f});
    backdrop->setColor({0.0f, 0.0f, 0.0f, 0.65f});
    backdrop->setVisible(false);
    backdrop->setOnClick([]() {});
    engine.getUIManager().addElement(backdrop);
    menuElements->push_back(backdrop);

    auto panel = std::make_shared<Button>();
    panel->setPosition({0.22f, 0.15f});
    panel->setSize({0.56f, 0.7f});
    panel->setColor({0.1f, 0.12f, 0.17f, 0.95f});
    panel->setVisible(false);
    panel->setOnClick([]() {});
    engine.getUIManager().addElement(panel);
    menuElements->push_back(panel);

    auto title = std::make_shared<Label>();
    title->setPosition({0.26f, 0.2f});
    title->setSize({0.48f, 0.07f});
    title->setColor({1.0f, 0.92f, 0.3f, 1.0f});
    title->setFontSize(1.4f);
    title->setText("Pause Menu");
    title->setVisible(false);
    engine.getUIManager().addElement(title);
    menuElements->push_back(title);

    auto subtitle = std::make_shared<Label>();
    subtitle->setPosition({0.26f, 0.27f});
    subtitle->setSize({0.48f, 0.05f});
    subtitle->setColor({0.8f, 0.85f, 0.95f, 1.0f});
    subtitle->setFontSize(1.0f);
    subtitle->setText("Select level (ESC to resume)");
    subtitle->setVisible(false);
    engine.getUIManager().addElement(subtitle);
    menuElements->push_back(subtitle);

    for (size_t i = 0; i < levelManager.getLevelCount(); ++i) {
        const Level* level = levelManager.getLevel(i);
        if (!level) {
            continue;
        }

        const float y = 0.35f + static_cast<float>(i) * 0.09f;
        const bool isCurrent = i == currentLevelIndex;

        auto levelButton = std::make_shared<Button>();
        levelButton->setPosition({0.28f, y});
        levelButton->setSize({0.44f, 0.07f});
        levelButton->setColor(isCurrent ? glm::vec4{0.2f, 0.38f, 0.68f, 1.0f}
                                        : glm::vec4{0.18f, 0.2f, 0.26f, 1.0f});
        levelButton->setVisible(false);
        levelButton->setOnClick([onLevelSelected, i]() {
            if (onLevelSelected) {
                onLevelSelected(i);
            }
        });
        engine.getUIManager().addElement(levelButton);
        menuElements->push_back(levelButton);

        auto levelLabel = std::make_shared<Label>();
        levelLabel->setPosition({0.30f, y + 0.015f});
        levelLabel->setSize({0.40f, 0.04f});
        levelLabel->setColor({1.0f, 1.0f, 1.0f, 1.0f});
        levelLabel->setFontSize(1.0f);
        levelLabel->setText((isCurrent ? "* " : "") + level->getName());
        levelLabel->setVisible(false);
        engine.getUIManager().addElement(levelLabel);
        menuElements->push_back(levelLabel);
    }

    PauseMenuBindings bindings;
    bindings.setVisible = [menuVisible, menuElements](bool visible) {
        *menuVisible = visible;
        for (const auto& element : *menuElements) {
            if (element) {
                element->setVisible(visible);
            }
        }
    };
    bindings.isVisible = [menuVisible]() {
        return *menuVisible;
    };

    return bindings;
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
