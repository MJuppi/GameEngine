#pragma once

#include <vector>
#include "engine/physics/cannon/Equation.h"

namespace ge {
namespace cannon {

class World;
class Body;

class Solver {
public:
    Solver();
    virtual ~Solver() = default;

    std::vector<Equation*> equations;
    int iterations;
    float tolerance;
    bool useWarmstarting;

    virtual int solve(float dt, World& world);
    virtual int solve(float dt, World& world, const std::vector<Body*>& bodies);
    void addEquation(Equation* eq);
    void removeAllEquations();
};

class GSSolver : public Solver {
public:
    GSSolver();
    int solve(float dt, World& world) override;
    int solve(float dt, World& world, const std::vector<Body*>& bodies) override;
};

} // namespace cannon
} // namespace ge
