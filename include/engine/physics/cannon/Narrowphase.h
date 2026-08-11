#pragma once

#include <vector>
#include "engine/physics/cannon/Body.h"
#include "engine/physics/cannon/Equation.h"

namespace ge {
namespace cannon {

class World;

class Narrowphase {
public:
    explicit Narrowphase(World* world);
    ~Narrowphase();
    void getContacts(const std::vector<Body*>& p1,
                     const std::vector<Body*>& p2,
                     World& world,
                     std::vector<ContactEquation*>& result,
                     std::vector<FrictionEquation*>& frictionResult,
                     std::vector<FrictionEquation*>& frictionPool);
    bool bodiesOverlap(const Body* bi, const Body* bj) const;
    void releaseEquationPools(std::vector<ContactEquation*>& contacts,
                              std::vector<FrictionEquation*>& frictionEquations);

    bool enableFrictionReduction;

private:
    World* world_;
    std::vector<ContactEquation*> contactPointPool_;
    std::vector<FrictionEquation*> frictionEquationPool_;

    ContactEquation* createContactEquation(Body* bi, Body* bj, const Material* shapeMatA, const Material* shapeMatB, World& world);
    bool createFrictionEquationsFromContact(ContactEquation* contactEquation,
                                            const Material* shapeMatA,
                                            const Material* shapeMatB,
                                            World& world,
                                            std::vector<FrictionEquation*>& outArray,
                                            std::vector<FrictionEquation*>& pool);
    void createFrictionFromAverage(size_t contactStartIndex,
                                   size_t numContacts,
                                   const Material* shapeMatA,
                                   const Material* shapeMatB,
                                   World& world,
                                   std::vector<ContactEquation*>& contacts,
                                   std::vector<FrictionEquation*>& frictionEquations,
                                   std::vector<FrictionEquation*>& frictionPool);
};

} // namespace cannon
} // namespace ge
