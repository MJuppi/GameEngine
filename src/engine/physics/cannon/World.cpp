#include "engine/physics/cannon/World.h"
#include "engine/physics/BoxCollider.h"
#include "engine/physics/SphereCollider.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <vector>

namespace ge {
namespace cannon {

namespace {

uint64_t makeMaterialPairKey(const Material* matA, const Material* matB) {
    if (!matA || !matB) {
        return 0ull;
    }

    const uint32_t a = static_cast<uint32_t>(matA->id);
    const uint32_t b = static_cast<uint32_t>(matB->id);
    const uint32_t lo = std::min(a, b);
    const uint32_t hi = std::max(a, b);
    return (static_cast<uint64_t>(lo) << 32) | static_cast<uint64_t>(hi);
}

uint64_t makeBodyPairKey(int bodyIdA, int bodyIdB) {
    const uint32_t a = static_cast<uint32_t>(bodyIdA);
    const uint32_t b = static_cast<uint32_t>(bodyIdB);
    const uint32_t lo = std::min(a, b);
    const uint32_t hi = std::max(a, b);
    return (static_cast<uint64_t>(lo) << 32) | static_cast<uint64_t>(hi);
}

std::pair<int, int> decodeBodyPairKey(uint64_t key) {
    const int a = static_cast<int>((key >> 32) & 0xFFFFFFFFull);
    const int b = static_cast<int>(key & 0xFFFFFFFFull);
    return {a, b};
}

bool pairLess(const std::pair<Body*, Body*>& a, const std::pair<Body*, Body*>& b) {
    const int a0 = a.first ? a.first->id : 0;
    const int a1 = a.second ? a.second->id : 0;
    const int b0 = b.first ? b.first->id : 0;
    const int b1 = b.second ? b.second->id : 0;
    if (a0 != b0) {
        return a0 < b0;
    }
    return a1 < b1;
}

void canonicalizePair(std::pair<Body*, Body*>& pair) {
    if (!pair.first || !pair.second) {
        return;
    }

    if (pair.second->id < pair.first->id) {
        std::swap(pair.first, pair.second);
    }
}

void getBoxAxes(const Quaternion& q, Vec3& ax, Vec3& ay, Vec3& az) {
    ax = q.vmult(Vec3(1.0f, 0.0f, 0.0f));
    ay = q.vmult(Vec3(0.0f, 1.0f, 0.0f));
    az = q.vmult(Vec3(0.0f, 0.0f, 1.0f));
}

void getShapeWorldTransform(const Body* body, Vec3& worldPosition, Quaternion& worldOrientation) {
    worldOrientation = body->quaternion.mult(body->shapeOrientation);
    const Vec3 worldOffset = body->quaternion.vmult(body->shapeOffset);
    worldPosition = body->position;
    worldPosition.vadd(worldOffset);
}

bool computeBodyAABB(const Body* body, AABB& out) {
    if (!body || !body->collider) {
        return false;
    }

    Vec3 center;
    Quaternion orientation;
    getShapeWorldTransform(body, center, orientation);

    if (body->collider->getType() == ColliderType::Sphere) {
        const auto* sphere = static_cast<const SphereCollider*>(body->collider.get());
        const float r = sphere->getRadius();
        out.lowerBound.set(center.x - r, center.y - r, center.z - r);
        out.upperBound.set(center.x + r, center.y + r, center.z + r);
        return true;
    }

    if (body->collider->getType() == ColliderType::Box) {
        const auto* box = static_cast<const BoxCollider*>(body->collider.get());
        const glm::vec3 he = box->getHalfExtents();

        Vec3 ax, ay, az;
        getBoxAxes(orientation, ax, ay, az);

        const float rx = std::fabs(ax.x) * he.x + std::fabs(ay.x) * he.y + std::fabs(az.x) * he.z;
        const float ry = std::fabs(ax.y) * he.x + std::fabs(ay.y) * he.y + std::fabs(az.y) * he.z;
        const float rz = std::fabs(ax.z) * he.x + std::fabs(ay.z) * he.y + std::fabs(az.z) * he.z;

        out.lowerBound.set(center.x - rx, center.y - ry, center.z - rz);
        out.upperBound.set(center.x + rx, center.y + ry, center.z + rz);
        return true;
    }

    return false;
}

bool aabbOverlapYZ(const AABB& a, const AABB& b) {
    return a.lowerBound.y <= b.upperBound.y && a.upperBound.y >= b.lowerBound.y &&
           a.lowerBound.z <= b.upperBound.z && a.upperBound.z >= b.lowerBound.z;
}

struct BodyIslandUnionFind {
    std::vector<size_t> parent;
    std::vector<size_t> rank;

    explicit BodyIslandUnionFind(size_t count)
        : parent(count), rank(count, 0) {
        std::iota(parent.begin(), parent.end(), 0);
    }

    size_t find(size_t index) {
        if (parent[index] == index) {
            return index;
        }
        parent[index] = find(parent[index]);
        return parent[index];
    }

    void unite(size_t a, size_t b) {
        a = find(a);
        b = find(b);
        if (a == b) {
            return;
        }

        if (rank[a] < rank[b]) {
            std::swap(a, b);
        }

        parent[b] = a;
        if (rank[a] == rank[b]) {
            ++rank[a];
        }
    }
};

struct SolverIsland {
    std::vector<Body*> bodies;
    std::vector<Equation*> equations;
    int minBodyId = std::numeric_limits<int>::max();
};

std::vector<SolverIsland> buildSolverIslands(const std::vector<Body*>& worldBodies,
                                             const std::vector<Equation*>& equations) {
    std::unordered_map<Body*, size_t> bodyIndex;
    bodyIndex.reserve(worldBodies.size());
    for (size_t i = 0; i < worldBodies.size(); ++i) {
        if (worldBodies[i]) {
            bodyIndex[worldBodies[i]] = i;
        }
    }

    BodyIslandUnionFind uf(worldBodies.size());
    std::vector<bool> usedBody(worldBodies.size(), false);

    for (Equation* equation : equations) {
        if (!equation || !equation->bi || !equation->bj) {
            continue;
        }

        auto itA = bodyIndex.find(equation->bi);
        auto itB = bodyIndex.find(equation->bj);
        if (itA == bodyIndex.end() || itB == bodyIndex.end()) {
            continue;
        }

        usedBody[itA->second] = true;
        usedBody[itB->second] = true;
        uf.unite(itA->second, itB->second);
    }

    std::unordered_map<size_t, size_t> rootToIsland;
    std::vector<SolverIsland> islands;
    islands.reserve(equations.size());

    auto ensureIsland = [&](size_t root) -> SolverIsland& {
        auto it = rootToIsland.find(root);
        if (it != rootToIsland.end()) {
            return islands[it->second];
        }

        const size_t index = islands.size();
        rootToIsland[root] = index;
        islands.push_back(SolverIsland{});
        return islands.back();
    };

    for (size_t i = 0; i < worldBodies.size(); ++i) {
        if (!usedBody[i] || !worldBodies[i]) {
            continue;
        }

        Body* body = worldBodies[i];
        SolverIsland& island = ensureIsland(uf.find(i));
        island.bodies.push_back(body);
        island.minBodyId = std::min(island.minBodyId, body->id);
    }

    for (Equation* equation : equations) {
        if (!equation || !equation->bi || !equation->bj) {
            continue;
        }

        auto itA = bodyIndex.find(equation->bi);
        auto itB = bodyIndex.find(equation->bj);
        if (itA == bodyIndex.end() || itB == bodyIndex.end()) {
            continue;
        }

        SolverIsland& island = ensureIsland(uf.find(itA->second));
        island.equations.push_back(equation);
    }

    std::sort(islands.begin(), islands.end(), [](const SolverIsland& a, const SolverIsland& b) {
        return a.minBodyId < b.minBodyId;
    });

    for (SolverIsland& island : islands) {
        std::sort(island.bodies.begin(), island.bodies.end(), [](const Body* a, const Body* b) {
            const int idA = a ? a->id : 0;
            const int idB = b ? b->id : 0;
            return idA < idB;
        });
    }

    return islands;
}

} // namespace

World::World()
    : gravity(), dt(1.0f / 60.0f), time(0.0f), accumulator(0.0f), stepnumber(0), allowSleep(true), solver(new GSSolver()), narrowphase(new Narrowphase(this)), defaultMaterial("default"), defaultContactMaterial(defaultMaterial, defaultMaterial, 0.3f, 0.0f), frictionGravity(0.0f), broadphaseType(BroadphaseType::Naive), bodyOverlapsCurrent(), bodyOverlapsPrevious() {
}

World::~World() {
    delete solver;
    delete narrowphase;
}

void World::addBody(Body* body) {
    if (!body) {
        return;
    }

    if (std::find(bodies.begin(), bodies.end(), body) != bodies.end()) {
        return;
    }

    bodies.push_back(body);
}

void World::removeBody(Body* body) {
    if (!body) {
        return;
    }

    bodies.erase(std::remove(bodies.begin(), bodies.end(), body), bodies.end());
}

void World::addConstraint(Constraint* constraint) {
    if (!constraint) {
        return;
    }

    if (std::find(constraints.begin(), constraints.end(), constraint) != constraints.end()) {
        return;
    }

    constraints.push_back(constraint);
}

void World::removeConstraint(Constraint* constraint) {
    if (!constraint) {
        return;
    }

    constraints.erase(std::remove(constraints.begin(), constraints.end(), constraint), constraints.end());
}

void World::setBroadphase(BroadphaseType type) {
    broadphaseType = type;
}

BroadphaseType World::getBroadphase() const {
    return broadphaseType;
}

bool World::isBodyPairOverlapping(int bodyIdA, int bodyIdB) const {
    const uint64_t key = makeBodyPairKey(bodyIdA, bodyIdB);
    return bodyOverlapsCurrent.find(key) != bodyOverlapsCurrent.end();
}

void World::getCurrentBodyOverlaps(std::vector<std::pair<int, int>>& overlaps) const {
    overlaps.clear();
    overlaps.reserve(bodyOverlapsCurrent.size());
    for (uint64_t key : bodyOverlapsCurrent) {
        overlaps.push_back(decodeBodyPairKey(key));
    }
}

void World::getBodyOverlapDeltas(std::vector<std::pair<int, int>>& began,
                                 std::vector<std::pair<int, int>>& ended) const {
    began.clear();
    ended.clear();

    for (uint64_t key : bodyOverlapsCurrent) {
        if (bodyOverlapsPrevious.find(key) == bodyOverlapsPrevious.end()) {
            began.push_back(decodeBodyPairKey(key));
        }
    }

    for (uint64_t key : bodyOverlapsPrevious) {
        if (bodyOverlapsCurrent.find(key) == bodyOverlapsCurrent.end()) {
            ended.push_back(decodeBodyPairKey(key));
        }
    }
}

void World::addContactMaterial(const ContactMaterial& contactMaterial) {
    const uint64_t key = makeMaterialPairKey(contactMaterial.materials[0], contactMaterial.materials[1]);
    if (key == 0ull) {
        return;
    }
    contactMaterialTable[key] = contactMaterial;
}

const ContactMaterial& World::getContactMaterial(const Material* matA, const Material* matB) const {
    const uint64_t key = makeMaterialPairKey(matA, matB);
    if (key != 0ull) {
        auto it = contactMaterialTable.find(key);
        if (it != contactMaterialTable.end()) {
            return it->second;
        }
    }
    return defaultContactMaterial;
}

void World::step(float dt_, float timeSinceLastCalled, int maxSubSteps) {
    if (timeSinceLastCalled < 0.0f) {
        internalStep(dt_);
        time += dt_;
        return;
    }

    accumulator += timeSinceLastCalled;
    int substeps = 0;

    while (accumulator >= dt_ && substeps < maxSubSteps) {
        internalStep(dt_);
        accumulator -= dt_;
        substeps++;
    }

    time += timeSinceLastCalled;
}

void World::internalStep(float dt_) {
    dt = dt_;
    narrowphase->releaseEquationPools(contacts, frictionEquations);
    bodyOverlapsPrevious = bodyOverlapsCurrent;
    bodyOverlapsCurrent.clear();

    for (Body* body : bodies) {
        if (body->type == BodyType::DYNAMIC && body->sleepState != 2 && body->useGravity && !body->isTrigger) {
            Vec3 gravityForce = gravity;
            gravityForce.scale(body->mass);
            body->force.add(gravityForce);
        }
    }

    std::vector<Body*> p1;
    std::vector<Body*> p2;
    std::vector<std::pair<Body*, Body*>> overlapOnlyPairs;
    const size_t N = bodies.size();

    auto passesFilters = [](Body* bi, Body* bj) {
        if (!bi || !bj) {
            return false;
        }

        if (bi->isTrigger || bj->isTrigger) {
            return false;
        }

        if (((bi->collisionMask & bj->collisionLayer) == 0u) ||
            ((bj->collisionMask & bi->collisionLayer) == 0u)) {
            return false;
        }

        return true;
    };

    auto shouldSolveCollision = [](Body* bi, Body* bj) {
        const bool biDynamic = (bi->type == BodyType::DYNAMIC);
        const bool bjDynamic = (bj->type == BodyType::DYNAMIC);
        return biDynamic || bjDynamic;
    };

    auto isKinematicEdgePair = [](Body* bi, Body* bj) {
        const bool biKinematic = (bi->type == BodyType::KINEMATIC);
        const bool bjKinematic = (bj->type == BodyType::KINEMATIC);
        const bool biStatic = (bi->type == BodyType::STATIC);
        const bool bjStatic = (bj->type == BodyType::STATIC);
        return (biKinematic && bjKinematic) || (biKinematic && bjStatic) || (bjKinematic && biStatic);
    };

    if (broadphaseType == BroadphaseType::SAP_X) {
        struct Entry {
            Body* body;
            AABB aabb;
        };

        std::vector<Entry> entries;
        entries.reserve(N);
        for (Body* body : bodies) {
            AABB aabb;
            if (computeBodyAABB(body, aabb)) {
                entries.push_back(Entry{body, aabb});
            }
        }

        std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) {
            return a.aabb.lowerBound.x < b.aabb.lowerBound.x;
        });

        for (size_t i = 0; i < entries.size(); ++i) {
            const Entry& ei = entries[i];
            for (size_t j = i + 1; j < entries.size(); ++j) {
                const Entry& ej = entries[j];
                if (ej.aabb.lowerBound.x > ei.aabb.upperBound.x) {
                    break;
                }

                if (!aabbOverlapYZ(ei.aabb, ej.aabb)) {
                    continue;
                }

                if (!passesFilters(ei.body, ej.body)) {
                    continue;
                }

                if (shouldSolveCollision(ei.body, ej.body)) {
                    p1.push_back(ei.body);
                    p2.push_back(ej.body);
                } else if (isKinematicEdgePair(ei.body, ej.body)) {
                    overlapOnlyPairs.emplace_back(ei.body, ej.body);
                }
            }
        }
    } else {
        for (size_t i = 0; i < N; ++i) {
            for (size_t j = i + 1; j < N; ++j) {
                Body* bi = bodies[i];
                Body* bj = bodies[j];
                if (!passesFilters(bi, bj)) {
                    continue;
                }

                if (shouldSolveCollision(bi, bj)) {
                    p1.push_back(bi);
                    p2.push_back(bj);
                } else if (isKinematicEdgePair(bi, bj)) {
                    overlapOnlyPairs.emplace_back(bi, bj);
                }
            }
        }
    }

    for (size_t i = 0; i < p1.size(); ++i) {
        if (p2[i] && p1[i] && p2[i]->id < p1[i]->id) {
            std::swap(p1[i], p2[i]);
        }
    }

    std::vector<size_t> pairOrder(p1.size());
    for (size_t i = 0; i < pairOrder.size(); ++i) {
        pairOrder[i] = i;
    }

    std::sort(pairOrder.begin(), pairOrder.end(), [&](size_t lhs, size_t rhs) {
        const std::pair<Body*, Body*> left{p1[lhs], p2[lhs]};
        const std::pair<Body*, Body*> right{p1[rhs], p2[rhs]};
        return pairLess(left, right);
    });

    std::vector<Body*> sortedP1;
    std::vector<Body*> sortedP2;
    sortedP1.reserve(p1.size());
    sortedP2.reserve(p2.size());
    for (size_t index : pairOrder) {
        sortedP1.push_back(p1[index]);
        sortedP2.push_back(p2[index]);
    }
    p1.swap(sortedP1);
    p2.swap(sortedP2);

    std::sort(overlapOnlyPairs.begin(), overlapOnlyPairs.end(), [](const auto& lhs, const auto& rhs) {
        std::pair<Body*, Body*> left = lhs;
        std::pair<Body*, Body*> right = rhs;
        canonicalizePair(left);
        canonicalizePair(right);
        return pairLess(left, right);
    });

    for (auto& pair : overlapOnlyPairs) {
        canonicalizePair(pair);
    }

    for (Constraint* constraint : constraints) {
        if (!constraint || constraint->collideConnected) {
            continue;
        }

        for (size_t i = 0; i < p1.size();) {
            Body* bodyA = p1[i];
            Body* bodyB = p2[i];
            if ((constraint->bodyA == bodyA && constraint->bodyB == bodyB) ||
                (constraint->bodyB == bodyA && constraint->bodyA == bodyB)) {
                p1.erase(p1.begin() + i);
                p2.erase(p2.begin() + i);
                continue;
            }
            ++i;
        }

        for (size_t i = 0; i < overlapOnlyPairs.size();) {
            Body* bodyA = overlapOnlyPairs[i].first;
            Body* bodyB = overlapOnlyPairs[i].second;
            if ((constraint->bodyA == bodyA && constraint->bodyB == bodyB) ||
                (constraint->bodyB == bodyA && constraint->bodyA == bodyB)) {
                overlapOnlyPairs.erase(overlapOnlyPairs.begin() + i);
                continue;
            }
            ++i;
        }
    }

    for (const auto& pair : overlapOnlyPairs) {
        Body* bi = pair.first;
        Body* bj = pair.second;
        if (narrowphase->bodiesOverlap(bi, bj)) {
            bodyOverlapsCurrent.insert(makeBodyPairKey(bi->id, bj->id));
        }
    }

    std::vector<FrictionEquation*> frictionPool;
    narrowphase->getContacts(p1, p2, *this, contacts, frictionEquations, frictionPool);

    for (ContactEquation* contact : contacts) {
        if (contact && contact->bi && contact->bj) {
            const int bodyIdA = std::min(contact->bi->id, contact->bj->id);
            const int bodyIdB = std::max(contact->bi->id, contact->bj->id);
            bodyOverlapsCurrent.insert(makeBodyPairKey(bodyIdA, bodyIdB));
        }
    }

    for (ContactEquation* contact : contacts) {
        if (!contact || !contact->enabled || !contact->bi || !contact->bj) {
            continue;
        }

        Body* bi = contact->bi;
        Body* bj = contact->bj;

        if (bi->allowSleep && bi->type == BodyType::DYNAMIC && bi->sleepState == 2 &&
            bj->sleepState == 0 && bj->type != BodyType::STATIC) {
            const float speedSquaredB = bj->velocity.lengthSquared() + bj->angularVelocity.lengthSquared();
            const float speedLimitSquaredB = bj->sleepSpeedLimit * bj->sleepSpeedLimit;
            if (speedSquaredB >= speedLimitSquaredB * 2.0f) {
                bi->wakeUpAfterNarrowphase = true;
            }
        }

        if (bj->allowSleep && bj->type == BodyType::DYNAMIC && bj->sleepState == 2 &&
            bi->sleepState == 0 && bi->type != BodyType::STATIC) {
            const float speedSquaredA = bi->velocity.lengthSquared() + bi->angularVelocity.lengthSquared();
            const float speedLimitSquaredA = bi->sleepSpeedLimit * bi->sleepSpeedLimit;
            if (speedSquaredA >= speedLimitSquaredA * 2.0f) {
                bj->wakeUpAfterNarrowphase = true;
            }
        }

        solver->addEquation(contact);
    }

    for (FrictionEquation* friction : frictionEquations) {
        solver->addEquation(friction);
    }

    for (Body* body : bodies) {
        if (body->wakeUpAfterNarrowphase) {
            body->wakeUp();
            body->wakeUpAfterNarrowphase = false;
        }
    }

    std::vector<std::pair<int, int>> sortedOverlaps;
    sortedOverlaps.reserve(bodyOverlapsCurrent.size());
    for (uint64_t key : bodyOverlapsCurrent) {
        sortedOverlaps.push_back(decodeBodyPairKey(key));
    }
    std::sort(sortedOverlaps.begin(), sortedOverlaps.end());
    bodyOverlapsCurrent.clear();
    for (const auto& pair : sortedOverlaps) {
        bodyOverlapsCurrent.insert(makeBodyPairKey(pair.first, pair.second));
    }

    for (Constraint* constraint : constraints) {
        if (!constraint) {
            continue;
        }
        constraint->update();
        for (Equation* eq : constraint->equations) {
            solver->addEquation(eq);
        }
    }

    for (Body* body : bodies) {
        body->updateSolveMassProperties();
        body->vlambda.setZero();
        body->wlambda.setZero();
    }

    const std::vector<SolverIsland> islands = buildSolverIslands(bodies, solver->equations);
    for (const SolverIsland& island : islands) {
        if (island.equations.empty() || island.bodies.empty()) {
            continue;
        }

        solver->equations = island.equations;
        solver->solve(dt_, *this, island.bodies);
    }

    solver->removeAllEquations();

    for (Body* body : bodies) {
        if (body->sleepState == 2) {
            body->force.setZero();
            body->torque.setZero();
            continue;
        }

        if (body->type == BodyType::DYNAMIC) {
            const float ld = std::pow(1.0f - body->linearDamping, dt_);
            body->velocity.scale(ld);
            const float ad = std::pow(1.0f - body->angularDamping, dt_);
            body->angularVelocity.scale(ad);
        }
        body->integrate(dt_, true);
        body->force.setZero();
        body->torque.setZero();

        if (allowSleep) {
            body->sleepTick(time);
        }
    }

    narrowphase->releaseEquationPools(contacts, frictionEquations);
    ++stepnumber;
}

void World::clearForces() {
    for (Body* body : bodies) {
        body->force.setZero();
        body->torque.setZero();
    }
}

void World::fixedStep(float dt_, int maxSubSteps) {
    step(dt_, dt_, maxSubSteps);
}

} // namespace cannon
} // namespace ge
