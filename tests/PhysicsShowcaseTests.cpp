#include "game/Level.h"
#include "game/SceneFactory.h"
#include "engine/asset/AssetManager.h"
#include "engine/physics/BoxCollider.h"

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

const ge::PhysicsMeshObject& findObject(const ge::Level& level, const char* name) {
    const auto& objects = level.getObjects();
    const auto it = std::find_if(objects.begin(), objects.end(), [&name](const auto& object) {
        return object.name == name;
    });
    if (it == objects.end()) {
        throw std::runtime_error(std::string("Showcase is missing object: ") + name);
    }
    return *it;
}

void testPhysicsShowcaseDemonstratesDistinctPhysicsFeatures() {
    ge::Level level("PhysicsShowcase");
    ge::SceneFactory::configurePhysicsShowcaseLevel(level);

    const auto& lowFriction = findObject(level, "LowFrictionSlider");
    const auto& highFriction = findObject(level, "HighFrictionSlider");
    if (lowFriction.physicsProps.friction >= highFriction.physicsProps.friction) {
        throw std::runtime_error("The showcase must compare low- and high-friction bodies");
    }

    const auto& lowBounce = findObject(level, "LowBounceBlock");
    const auto& highBounce = findObject(level, "HighBounceBlock");
    if (lowBounce.physicsProps.restitution >= highBounce.physicsProps.restitution) {
        throw std::runtime_error("The showcase must compare low- and high-restitution bodies");
    }

    const auto& platform = findObject(level, "KinematicPlatform");
    if (!platform.physicsProps.isKinematic) {
        throw std::runtime_error("The showcase must include a kinematic platform");
    }

    const auto& sensor = findObject(level, "SensorGate");
    if (!sensor.physicsProps.isTrigger) {
        throw std::runtime_error("The showcase must include a non-blocking trigger");
    }

    findObject(level, "HingeDoor");
    findObject(level, "DistanceWeight");
    const auto& stackBox = findObject(level, "Stack_0");
    const ge::RigidBodyProps defaultProps;
    if (stackBox.physicsProps.mass != defaultProps.mass ||
        stackBox.physicsProps.friction != defaultProps.friction ||
        stackBox.physicsProps.restitution != defaultProps.restitution) {
        throw std::runtime_error("Procedural scene migration must preserve default stack physics properties");
    }

    if (level.getObjects().size() < 15) {
        throw std::runtime_error("The showcase needs enough physical props to demonstrate the feature set");
    }
}

void testProceduralBoxesUseRequestedDimensionsAndMaterial() {
    ge::Material material;
    material.name = "procedural-box";
    material.diffuse = {0.2f, 0.4f, 0.8f};
    ge::RigidBodyProps props;
    props.useGravity = false;

    ge::PhysicsMeshObject box = ge::ObjectBuilder::createBox(
        "box", {2.0f, 4.0f, 6.0f}, {3.0f, 5.0f, 7.0f},
        ge::ObjectType::Active, props, material);

    if (!box.meshPath.empty()) {
        throw std::runtime_error("Procedural boxes must not require an external mesh path");
    }
    if (box.mesh.vertices.size() != 24 || box.mesh.indices.size() != 36) {
        throw std::runtime_error("A procedural box must contain six independently-normaled faces");
    }
    const ge::MeshBounds bounds = ge::computeMeshBounds(box.mesh);
    if (glm::length(bounds.getHalfExtents() - glm::vec3(1.0f, 2.0f, 3.0f)) > 1e-5f) {
        throw std::runtime_error("Procedural box mesh dimensions must match requested size");
    }
    if (box.halfExtents != glm::vec3(1.0f, 2.0f, 3.0f)) {
        throw std::runtime_error("Procedural box collider extents must match mesh half-extents");
    }
    if (box.mesh.materials.size() != 1 ||
        box.mesh.materials[0].diffuse != material.diffuse) {
        throw std::runtime_error("Procedural boxes must preserve the requested material");
    }

    ge::PhysicsEngine physics;
    ge::RigidBody* body = ge::ObjectBuilder::attachPhysics(physics, box);
    if (!body || body->getMesh().has_value() == false) {
        throw std::runtime_error("Active procedural boxes must attach their mesh to the body");
    }
    const auto& collider = static_cast<const ge::BoxCollider&>(body->getCollider());
    if (glm::length(collider.getHalfExtents() - glm::vec3(1.0f, 2.0f, 3.0f)) > 1e-5f) {
        throw std::runtime_error("Procedural box collider must use the requested dimensions");
    }
    if (body->getMaterial()->diffuse != material.diffuse) {
        throw std::runtime_error("Attached procedural box must preserve its material");
    }

    ge::Level level("Procedural");
    level.addBox("level-box", {2.0f, 4.0f, 6.0f}, {3.0f, 5.0f, 7.0f},
                 ge::ObjectType::Visual, {}, material);
    ge::AssetManager assets;
    level.load(assets);
    const ge::MeshBounds combinedBounds = ge::computeMeshBounds(level.getMesh());
    if (glm::length(combinedBounds.getHalfExtents() - glm::vec3(1.0f, 2.0f, 3.0f)) > 1e-5f) {
        throw std::runtime_error("Loading a procedural level box must preserve its dimensions");
    }
    if (glm::length(combinedBounds.getCenter() - glm::vec3(3.0f, 5.0f, 7.0f)) > 1e-5f) {
        throw std::runtime_error("Procedural level boxes must preserve their world position");
    }

    ge::Level defaultMaterials("DefaultMaterials");
    defaultMaterials.addBox("first", {1.0f, 1.0f, 1.0f});
    defaultMaterials.addBox("second", {1.0f, 1.0f, 1.0f});
    if (defaultMaterials.buildCombinedMesh().materials.size() != 1) {
        throw std::runtime_error("Default procedural boxes should share one material");
    }

    bool rejectedInvalidSize = false;
    try {
        (void)ge::ObjectBuilder::createBox("invalid", {1.0f, 0.0f, 1.0f});
    } catch (const std::invalid_argument&) {
        rejectedInvalidSize = true;
    }
    if (!rejectedInvalidSize) {
        throw std::runtime_error("Procedural boxes must reject zero-sized dimensions");
    }
}

void testSceneBoxesUseProceduralGeometry() {
    const auto verifyBoxes = [](ge::Level& level, const std::string& allowedExternalMesh = {}) {
        for (const auto& object : level.getObjects()) {
            if (!object.meshPath.empty()) {
                if (object.meshPath == allowedExternalMesh) {
                    continue;
                }
                throw std::runtime_error(
                    std::string("Scene box still depends on external mesh: ") + object.name);
            }
            if (object.mesh.vertices.empty() || object.mesh.indices.empty()) {
                throw std::runtime_error(
                    std::string("Scene box has no procedural geometry: ") + object.name);
            }

            const ge::MeshBounds bounds = ge::computeMeshBounds(object.mesh);
            const glm::vec3 expectedHalfExtents = object.halfExtents;
            if (glm::length(bounds.getHalfExtents() - expectedHalfExtents) > 1e-5f) {
                throw std::runtime_error(
                    std::string("Scene box visual dimensions do not match collider: ") + object.name);
            }
        }
    };

    ge::Level showcase("PhysicsShowcase");
    ge::SceneFactory::configurePhysicsShowcaseLevel(showcase);
    verifyBoxes(showcase);

    ge::Level mission("CombatMission");
    ge::SceneFactory::configureCombatMission(mission);
    verifyBoxes(mission);

    ge::Level testLevel("TestLevel");
    ge::SceneFactory::configureTestLevel(testLevel);
    verifyBoxes(testLevel, "suomi");

    ge::Level constraintParity("ConstraintParity");
    ge::SceneFactory::configureConstraintParityLevel(constraintParity);
    verifyBoxes(constraintParity);

    ge::Level pairOrdering("PairOrdering");
    ge::SceneFactory::configurePairOrderingLevel(pairOrdering);
    verifyBoxes(pairOrdering);
}

} // namespace

int main() {
    try {
        testPhysicsShowcaseDemonstratesDistinctPhysicsFeatures();
        testProceduralBoxesUseRequestedDimensionsAndMaterial();
        testSceneBoxesUseProceduralGeometry();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
