#include "game/Level.h"
#include "game/SceneFactory.h"

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
    findObject(level, "Stack_0");

    if (level.getObjects().size() < 15) {
        throw std::runtime_error("The showcase needs enough physical props to demonstrate the feature set");
    }
}

} // namespace

int main() {
    try {
        testPhysicsShowcaseDemonstratesDistinctPhysicsFeatures();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
