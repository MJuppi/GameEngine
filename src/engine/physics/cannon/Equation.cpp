#include "engine/physics/cannon/Equation.h"
#include "engine/physics/cannon/Body.h"

namespace ge {
namespace cannon {

JacobianElement::JacobianElement() : spatial(), rotational() {}

float JacobianElement::multiplyVectors(const Vec3& lv, const Vec3& av) const {
    return spatial.dot(lv) + rotational.dot(av);
}

Equation::Equation(Body* bi_, Body* bj_, float minForce_, float maxForce_)
    : minForce(minForce_), maxForce(maxForce_), a(0.0f), b(0.0f), eps(0.0f), multiplier(0.0f), enabled(true), bi(bi_), bj(bj_), jacobianElementA(), jacobianElementB() {
    setSpookParams(1e7f, 4.0f, 1.0f/60.0f);
}

void Equation::setSpookParams(float stiffness, float relaxation, float timeStep) {
    const float d = relaxation;
    const float k = stiffness;
    const float h = timeStep;
    a = 4.0f / (h * (1.0f + 4.0f * d));
    b = 4.0f * d / (1.0f + 4.0f * d);
    eps = 4.0f / (h * h * k * (1.0f + 4.0f * d));
}

float Equation::computeB(float a_, float b_, float h) {
    const float GW = computeGW();
    const float Gq = computeGq();
    const float GiMf = computeGiMf();
    return -Gq * a_ - GW * b_ - GiMf * h;
}

float Equation::computeGq() const {
    return 0.0f;
}

float Equation::computeC() {
    return computeGiMGt() + eps;
}

float Equation::computeGW() const {
    return jacobianElementA.multiplyVectors(bi->velocity, bi->angularVelocity) +
           jacobianElementB.multiplyVectors(bj->velocity, bj->angularVelocity);
}

float Equation::computeGWlambda() const {
    return jacobianElementA.multiplyVectors(bi->vlambda, bi->wlambda) +
           jacobianElementB.multiplyVectors(bj->vlambda, bj->wlambda);
}

float Equation::computeGiMf() const {
    Vec3 iMfi = bi->force;
    Vec3 iMfj = bj->force;
    iMfi.scale(bi->invMassSolve);
    iMfj.scale(bj->invMassSolve);
    Vec3 invIi_vmult_taui = bi->invInertiaWorldSolve.vmult(bi->torque);
    Vec3 invIj_vmult_tauj = bj->invInertiaWorldSolve.vmult(bj->torque);
    return jacobianElementA.multiplyVectors(iMfi, invIi_vmult_taui) +
           jacobianElementB.multiplyVectors(iMfj, invIj_vmult_tauj);
}

float Equation::computeGiMGt() const {
    float result = bi->invMassSolve + bj->invMassSolve;
    Vec3 tmp = bi->invInertiaWorldSolve.vmult(jacobianElementA.rotational);
    result += tmp.dot(jacobianElementA.rotational);
    tmp = bj->invInertiaWorldSolve.vmult(jacobianElementB.rotational);
    result += tmp.dot(jacobianElementB.rotational);
    return result;
}

void Equation::addToWlambda(float deltalambda) {
    Vec3 temp = bi->invInertiaWorldSolve.vmult(jacobianElementA.rotational);
    bi->vlambda.addScaledVector(bi->invMassSolve * deltalambda, jacobianElementA.spatial);
    bj->vlambda.addScaledVector(bj->invMassSolve * deltalambda, jacobianElementB.spatial);
    bi->wlambda.addScaledVector(deltalambda, temp);
    temp = bj->invInertiaWorldSolve.vmult(jacobianElementB.rotational);
    bj->wlambda.addScaledVector(deltalambda, temp);
}

ContactEquation::ContactEquation(Body* bodyA, Body* bodyB, float maxForce)
    : Equation(bodyA, bodyB, 0.0f, maxForce), restitution(0.0f), ri(), rj(), ni() {}

float ContactEquation::computeB(float a, float b, float h) {
    const Vec3 rixn = ri.cross(ni);
    const Vec3 rjxn = rj.cross(ni);
    const Vec3 n = ni;

    jacobianElementA.spatial = n;
    jacobianElementA.spatial.scale(-1.0f);
    jacobianElementA.rotational = rixn;
    jacobianElementA.rotational.scale(-1.0f);
    jacobianElementB.spatial = n;
    jacobianElementB.rotational = rjxn;

    const float contactSlop = 0.01f;
    const float g = (n.dot(bj->position) + n.dot(rj) - n.dot(bi->position) - n.dot(ri)) + contactSlop;
    const float ePlusOne = restitution + 1.0f;
    const float GW = ePlusOne * (bj->velocity.dot(n) - bi->velocity.dot(n)) + bj->angularVelocity.dot(rjxn) - bi->angularVelocity.dot(rixn);
    const float GiMf = computeGiMf();
    return -g * a - GW * b - h * GiMf;
}

} // namespace cannon
} // namespace ge
