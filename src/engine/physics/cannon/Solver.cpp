#include "engine/physics/cannon/Solver.h"
#include "engine/physics/cannon/Body.h"
#include "engine/physics/cannon/World.h"

namespace ge {
namespace cannon {

Solver::Solver() : iterations(10), tolerance(1e-7f) {}

int Solver::solve(float dt, World& world) {
    (void)dt;
    (void)world;
    return 0;
}

void Solver::addEquation(Equation* eq) {
    if (eq && eq->enabled) {
        equations.push_back(eq);
    }
}

void Solver::removeAllEquations() {
    equations.clear();
}

GSSolver::GSSolver() {
    iterations = 10;
    tolerance = 1e-7f;
}

int GSSolver::solve(float dt, World& world) {
    const int Neq = static_cast<int>(equations.size());
    if (Neq == 0) {
        return 0;
    }

    for (Body* body : world.bodies) {
        body->updateSolveMassProperties();
        body->vlambda.setZero();
        body->wlambda.setZero();
    }

    std::vector<float> invCs(Neq);
    std::vector<float> Bs(Neq);
    std::vector<float> lambda(Neq, 0.0f);

    for (int i = 0; i < Neq; ++i) {
        Equation* c = equations[i];
        Bs[i] = c->computeB(1.0f, 1.0f, dt);
        invCs[i] = 1.0f / c->computeC();
    }

    int iter = 0;
    const float tolSquared = tolerance * tolerance;

    while (iter < iterations) {
        float deltalambdaTot = 0.0f;

        for (int j = 0; j < Neq; ++j) {
            Equation* c = equations[j];
            const float lambdaj = lambda[j];
            const float GWlambda = c->computeGWlambda();
            float deltalambda = invCs[j] * (Bs[j] - GWlambda - c->eps * lambdaj);

            if (lambdaj + deltalambda < c->minForce) {
                deltalambda = c->minForce - lambdaj;
            } else if (lambdaj + deltalambda > c->maxForce) {
                deltalambda = c->maxForce - lambdaj;
            }

            lambda[j] += deltalambda;
            deltalambdaTot += std::abs(deltalambda);
            c->addToWlambda(deltalambda);
        }

        if (deltalambdaTot * deltalambdaTot < tolSquared) {
            break;
        }

        ++iter;
    }

    for (Body* body : world.bodies) {
        body->velocity.addScaledVector(1.0f, body->vlambda);
        body->angularVelocity.addScaledVector(1.0f, body->wlambda);
    }

    return iter;
}

} // namespace cannon
} // namespace ge
