#pragma once

#include <vector>
#include <memory>
#include "engine/physics/cannon/Body.h"
#include "engine/physics/cannon/Solver.h"
#include "engine/physics/cannon/Narrowphase.h"
#include "engine/physics/cannon/Material.h"
#include "engine/physics/cannon/AABB.h"

namespace ge {
namespace cannon {

class World {
public:
    World();
    ~World();

    std::vector<Body*> bodies;
    Vec3 gravity;
    float dt;
    float time;
    float accumulator;
    int stepnumber;
    bool allowSleep;
    Solver* solver;
    Narrowphase* narrowphase;
    std::vector<ContactEquation*> contacts;
    std::vector<ContactEquation*> frictionEquations;
    Material defaultMaterial;
    ContactMaterial defaultContactMaterial;

    void addBody(Body* body);
    void removeBody(Body* body);
    void step(float dt, float timeSinceLastCalled = -1.0f, int maxSubSteps = 10);
    void internalStep(float dt);
    void clearForces();
    void fixedStep(float dt = 1.0f/60.0f, int maxSubSteps = 10);
};

} // namespace cannon
} // namespace ge
