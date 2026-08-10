#pragma once

#include <vector>
#include "engine/physics/cannon/Equation.h"

namespace ge {
namespace cannon {

class World;

class Solver {
public:
    Solver();
    virtual ~Solver() = default;

    std::vector<Equation*> equations;
    int iterations;
    float tolerance;

    virtual int solve(float dt, World& world);
    void addEquation(Equation* eq);
    void removeAllEquations();
};

class GSSolver : public Solver {
public:
    GSSolver();
    int solve(float dt, World& world) override;
};

} // namespace cannon
} // namespace ge
