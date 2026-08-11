#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include "engine/physics/cannon/Body.h"
#include "engine/physics/cannon/Solver.h"
#include "engine/physics/cannon/Narrowphase.h"
#include "engine/physics/cannon/Material.h"
#include "engine/physics/cannon/AABB.h"
#include "engine/physics/cannon/Constraint.h"

namespace ge {
namespace cannon {

enum class BroadphaseType {
    Naive,
    SAP_X
};

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
    std::vector<FrictionEquation*> frictionEquations;
    std::vector<Constraint*> constraints;
    Material defaultMaterial;
    ContactMaterial defaultContactMaterial;
    float frictionGravity;
    BroadphaseType broadphaseType;
    std::unordered_map<uint64_t, ContactMaterial> contactMaterialTable;
    std::unordered_set<uint64_t> bodyOverlapsCurrent;
    std::unordered_set<uint64_t> bodyOverlapsPrevious;

    void addBody(Body* body);
    void removeBody(Body* body);
    void addConstraint(Constraint* constraint);
    void removeConstraint(Constraint* constraint);
    void setBroadphase(BroadphaseType type);
    BroadphaseType getBroadphase() const;
    bool isBodyPairOverlapping(int bodyIdA, int bodyIdB) const;
    void getCurrentBodyOverlaps(std::vector<std::pair<int, int>>& overlaps) const;
    void getBodyOverlapDeltas(std::vector<std::pair<int, int>>& began,
                              std::vector<std::pair<int, int>>& ended) const;
    void addContactMaterial(const ContactMaterial& contactMaterial);
    const ContactMaterial& getContactMaterial(const Material* matA, const Material* matB) const;
    void step(float dt, float timeSinceLastCalled = -1.0f, int maxSubSteps = 10);
    void internalStep(float dt);
    void clearForces();
    void fixedStep(float dt = 1.0f/60.0f, int maxSubSteps = 10);
};

} // namespace cannon
} // namespace ge
