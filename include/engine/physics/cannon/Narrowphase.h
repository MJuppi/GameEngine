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
    void getContacts(const std::vector<Body*>& p1,
                     const std::vector<Body*>& p2,
                     World& world,
                     std::vector<ContactEquation*>& result,
                     std::vector<ContactEquation*>& frictionResult,
                     std::vector<ContactEquation*>& frictionPool);

private:
    World* world_;
    ContactEquation* createContactEquation(Body* bi, Body* bj);
    void createFrictionEquationsFromContact(ContactEquation* contactEquation, std::vector<ContactEquation*>& outArray);
};

} // namespace cannon
} // namespace ge
