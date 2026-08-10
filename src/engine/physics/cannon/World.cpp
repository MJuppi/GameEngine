#include "engine/physics/cannon/World.h"
#include <algorithm>
#include <cmath>

namespace ge {
namespace cannon {

World::World()
    : gravity(), dt(1.0f / 60.0f), time(0.0f), accumulator(0.0f), stepnumber(0), allowSleep(true), solver(new GSSolver()), narrowphase(new Narrowphase(this)), defaultMaterial("default"), defaultContactMaterial(defaultMaterial, defaultMaterial, 0.3f, 0.0f) {
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
    for (Body* body : bodies) {
        if (body->type == BodyType::DYNAMIC && body->sleepState != 2 && body->useGravity && !body->isTrigger) {
            Vec3 gravityForce = gravity;
            gravityForce.scale(body->mass);
            body->force.add(gravityForce);
        }
    }

    for (ContactEquation* c : contacts) {
        delete c;
    }
    contacts.clear();
    for (ContactEquation* c : frictionEquations) {
        delete c;
    }
    frictionEquations.clear();

    std::vector<Body*> p1;
    std::vector<Body*> p2;
    const size_t N = bodies.size();
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = i + 1; j < N; ++j) {
            Body* bi = bodies[i];
            Body* bj = bodies[j];
            if (!bi || !bj) {
                continue;
            }

            const bool biDynamic = (bi->type == BodyType::DYNAMIC);
            const bool bjDynamic = (bj->type == BodyType::DYNAMIC);
            if (!biDynamic && !bjDynamic) {
                continue;
            }

            if (bi->isTrigger || bj->isTrigger) {
                continue;
            }

            if (((bi->collisionMask & bj->collisionLayer) == 0u) ||
                ((bj->collisionMask & bi->collisionLayer) == 0u)) {
                continue;
            }

            p1.push_back(bodies[i]);
            p2.push_back(bodies[j]);
        }
    }

    narrowphase->getContacts(p1, p2, *this, contacts, frictionEquations, frictionEquations);

    for (ContactEquation* contact : contacts) {
        solver->addEquation(contact);
    }

    for (ContactEquation* friction : frictionEquations) {
        solver->addEquation(friction);
    }

    solver->solve(dt_, *this);
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

    for (ContactEquation* c : contacts) {
        delete c;
    }
    for (ContactEquation* c : frictionEquations) {
        delete c;
    }
    contacts.clear();
    frictionEquations.clear();
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
