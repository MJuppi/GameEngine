#pragma once

#include "engine/physics/cannon/Vec3.h"
#include "engine/physics/cannon/Mat3.h"
#include <cstdint>

namespace ge {
namespace cannon {

struct JacobianElement {
    Vec3 spatial;
    Vec3 rotational;
    JacobianElement();
    float multiplyVectors(const Vec3& lv, const Vec3& av) const;
};

class Body;

class Equation {
public:
    Equation(Body* bi, Body* bj, float minForce = -1e6f, float maxForce = 1e6f);
    virtual ~Equation() = default;

    float minForce;
    float maxForce;
    float a;
    float b;
    float eps;
    float multiplier;
    bool enabled;
    Body* bi;
    Body* bj;
    JacobianElement jacobianElementA;
    JacobianElement jacobianElementB;

    void setSpookParams(float stiffness, float relaxation, float timeStep);
    virtual float computeB(float a, float b, float h);
    virtual float computeC();
    float computeGW() const;
    float computeGWlambda() const;
    float computeGq() const;
    float computeGiMf() const;
    float computeGiMGt() const;
    void addToWlambda(float deltalambda);
};

class ContactEquation : public Equation {
public:
    ContactEquation(Body* bodyA, Body* bodyB, float maxForce = 1e6f);
    float restitution;
    Vec3 ri;
    Vec3 rj;
    Vec3 ni;

    float computeB(float a, float b, float h) override;
};

class FrictionEquation : public Equation {
public:
    FrictionEquation(Body* bodyA, Body* bodyB, float slipForce = 1e6f);

    Vec3 ri;
    Vec3 rj;
    Vec3 t;

    float computeB(float a, float b, float h) override;
};

class ConeEquation : public Equation {
public:
    ConeEquation(Body* bodyA, Body* bodyB, float maxForce = 1e6f);

    Vec3 axisA;
    Vec3 axisB;
    float angle;

    float computeB(float a, float b, float h) override;
};

class RotationalEquation : public Equation {
public:
    RotationalEquation(Body* bodyA, Body* bodyB, float maxForce = 1e6f);

    Vec3 axisA;
    Vec3 axisB;
    float maxAngle;

    float computeB(float a, float b, float h) override;
};

class RotationalMotorEquation : public Equation {
public:
    RotationalMotorEquation(Body* bodyA, Body* bodyB, float maxForce = 1e6f);

    Vec3 axisA;
    Vec3 axisB;
    float targetVelocity;

    float computeB(float a, float b, float h) override;
};

} // namespace cannon
} // namespace ge
