#include "engine/physics/cannon/Constraint.h"
#include "engine/physics/cannon/Body.h"
#include "engine/physics/cannon/Equation.h"

#include <cmath>

namespace ge {
namespace cannon {

Constraint::Constraint(Body* bodyA_, Body* bodyB_, bool collideConnected_)
    : bodyA(bodyA_), bodyB(bodyB_), collideConnected(collideConnected_), equations() {}

void Constraint::update() {
}

PointToPointConstraint::PointToPointConstraint(Body* bodyA,
                                               const Vec3& pivotA_,
                                               Body* bodyB,
                                               const Vec3& pivotB_,
                                               float maxForce,
                                               bool collideConnected_)
    : Constraint(bodyA, bodyB, collideConnected_),
      pivotA(pivotA_),
      pivotB(pivotB_),
      equationX(new ContactEquation(bodyA, bodyB)),
      equationY(new ContactEquation(bodyA, bodyB)),
      equationZ(new ContactEquation(bodyA, bodyB)) {
    equations.push_back(equationX);
    equations.push_back(equationY);
    equations.push_back(equationZ);

    equationX->minForce = -maxForce;
    equationY->minForce = -maxForce;
    equationZ->minForce = -maxForce;
    equationX->maxForce = maxForce;
    equationY->maxForce = maxForce;
    equationZ->maxForce = maxForce;

    equationX->ni.set(1.0f, 0.0f, 0.0f);
    equationY->ni.set(0.0f, 1.0f, 0.0f);
    equationZ->ni.set(0.0f, 0.0f, 1.0f);
}

PointToPointConstraint::~PointToPointConstraint() {
    delete equationX;
    delete equationY;
    delete equationZ;
    equations.clear();
}

void PointToPointConstraint::update() {
    if (!bodyA || !bodyB || !equationX || !equationY || !equationZ) {
        return;
    }

    equationX->ri = bodyA->quaternion.vmult(pivotA);
    equationX->rj = bodyB->quaternion.vmult(pivotB);
    equationY->ri = equationX->ri;
    equationY->rj = equationX->rj;
    equationZ->ri = equationX->ri;
    equationZ->rj = equationX->rj;
}

DistanceConstraint::DistanceConstraint(Body* bodyA,
                                       Body* bodyB,
                                       float distance_,
                                       float maxForce,
                                       bool collideConnected_)
    : Constraint(bodyA, bodyB, collideConnected_),
      distance(distance_),
      distanceEquation(new ContactEquation(bodyA, bodyB)) {
    equations.push_back(distanceEquation);
    distanceEquation->minForce = -maxForce;
    distanceEquation->maxForce = maxForce;
}

DistanceConstraint::~DistanceConstraint() {
    delete distanceEquation;
    equations.clear();
}

void DistanceConstraint::update() {
    if (!bodyA || !bodyB || !distanceEquation) {
        return;
    }

    const float halfDist = distance * 0.5f;
    Vec3 normal = bodyB->position;
    normal.sub(bodyA->position);
    normal.normalize();

    distanceEquation->ni = normal;
    distanceEquation->ri = normal;
    distanceEquation->ri.scale(halfDist);
    distanceEquation->rj = normal;
    distanceEquation->rj.scale(-halfDist);
}

LockConstraint::LockConstraint(Body* bodyA,
                               Body* bodyB,
                               float maxForce,
                               bool collideConnected)
    : PointToPointConstraint(bodyA, Vec3(), bodyB, Vec3(), maxForce, collideConnected),
      rotationalEquation1(new RotationalEquation(bodyA, bodyB, maxForce)),
      rotationalEquation2(new RotationalEquation(bodyA, bodyB, maxForce)),
      rotationalEquation3(new RotationalEquation(bodyA, bodyB, maxForce)) {
        Vec3 halfWay = bodyA->position;
    halfWay.add(bodyB->position);
    halfWay.scale(0.5f);
    bodyB->pointToLocalFrame(halfWay, pivotB);
    bodyA->pointToLocalFrame(halfWay, pivotA);

    equations.push_back(rotationalEquation1);
    equations.push_back(rotationalEquation2);
    equations.push_back(rotationalEquation3);

    bodyA->vectorToLocalFrame(Vec3(1.0f, 0.0f, 0.0f), xA);
    bodyA->vectorToLocalFrame(Vec3(0.0f, 1.0f, 0.0f), yA);
    bodyA->vectorToLocalFrame(Vec3(0.0f, 0.0f, 1.0f), zA);
    bodyB->vectorToLocalFrame(Vec3(1.0f, 0.0f, 0.0f), xB);
    bodyB->vectorToLocalFrame(Vec3(0.0f, 1.0f, 0.0f), yB);
    bodyB->vectorToLocalFrame(Vec3(0.0f, 0.0f, 1.0f), zB);

    rotationalEquation1->maxAngle = 1.57079632679f;
    rotationalEquation2->maxAngle = 1.57079632679f;
    rotationalEquation3->maxAngle = 1.57079632679f;
}

LockConstraint::~LockConstraint() {
    delete rotationalEquation1;
    delete rotationalEquation2;
    delete rotationalEquation3;
}

void LockConstraint::update() {
    PointToPointConstraint::update();

    bodyA->vectorToWorldFrame(xA, rotationalEquation1->axisA);
    bodyB->vectorToWorldFrame(yB, rotationalEquation1->axisB);

    bodyA->vectorToWorldFrame(yA, rotationalEquation2->axisA);
    bodyB->vectorToWorldFrame(zB, rotationalEquation2->axisB);

    bodyA->vectorToWorldFrame(zA, rotationalEquation3->axisA);
    bodyB->vectorToWorldFrame(xB, rotationalEquation3->axisB);
}

HingeConstraint::HingeConstraint(Body* bodyA,
                                 Body* bodyB,
                                 const Vec3& pivotA,
                                 const Vec3& pivotB,
                                 const Vec3& axisA_,
                                 const Vec3& axisB_,
                                 float maxForce,
                                 bool collideConnected)
    : PointToPointConstraint(bodyA, pivotA, bodyB, pivotB, maxForce, collideConnected),
      axisA(axisA_),
      axisB(axisB_),
      rotationalEquation1(new RotationalEquation(bodyA, bodyB, maxForce)),
      rotationalEquation2(new RotationalEquation(bodyA, bodyB, maxForce)),
      motorEquation(new RotationalMotorEquation(bodyA, bodyB, maxForce)) {
    axisA.normalize();
    axisB.normalize();
    rotationalEquation1->maxAngle = 1.57079632679f;
    rotationalEquation2->maxAngle = 1.57079632679f;
    motorEquation->targetVelocity = 0.0f;
    motorEquation->enabled = false;

    equations.push_back(rotationalEquation1);
    equations.push_back(rotationalEquation2);
    equations.push_back(motorEquation);
}

HingeConstraint::~HingeConstraint() {
    delete rotationalEquation1;
    delete rotationalEquation2;
    delete motorEquation;
}

void HingeConstraint::enableMotor() {
    motorEquation->enabled = true;
}

void HingeConstraint::disableMotor() {
    motorEquation->enabled = false;
}

void HingeConstraint::setMotorSpeed(float speed) {
    motorEquation->targetVelocity = speed;
}

void HingeConstraint::setMotorMaxForce(float maxForce) {
    motorEquation->maxForce = maxForce;
    motorEquation->minForce = -maxForce;
}

void HingeConstraint::update() {
    PointToPointConstraint::update();

    Vec3 worldAxisA;
    Vec3 worldAxisB;
    bodyA->vectorToWorldFrame(axisA, worldAxisA);
    bodyB->vectorToWorldFrame(axisB, worldAxisB);

    worldAxisA.tangents(rotationalEquation1->axisA, rotationalEquation2->axisA);
    rotationalEquation1->axisB.copy(worldAxisB);
    rotationalEquation2->axisB.copy(worldAxisB);

    if (motorEquation->enabled) {
        bodyA->vectorToWorldFrame(axisA, motorEquation->axisA);
        bodyB->vectorToWorldFrame(axisB, motorEquation->axisB);
    }
}

ConeTwistConstraint::ConeTwistConstraint(Body* bodyA,
                                         Body* bodyB,
                                         const Vec3& pivotA,
                                         const Vec3& pivotB,
                                         const Vec3& axisA_,
                                         const Vec3& axisB_,
                                         float angle_,
                                         float twistAngle_,
                                         float maxForce,
                                         bool collideConnected)
    : PointToPointConstraint(bodyA, pivotA, bodyB, pivotB, maxForce, collideConnected),
      axisA(axisA_),
      axisB(axisB_),
      angle(angle_),
      twistAngle(twistAngle_),
      coneEquation(new ConeEquation(bodyA, bodyB, maxForce)),
      twistEquation(new RotationalEquation(bodyA, bodyB, maxForce)) {
    equations.push_back(coneEquation);
    equations.push_back(twistEquation);
    coneEquation->minForce = -maxForce;
    coneEquation->maxForce = 0.0f;
    twistEquation->minForce = -maxForce;
    twistEquation->maxForce = 0.0f;
}

ConeTwistConstraint::~ConeTwistConstraint() {
    delete coneEquation;
    delete twistEquation;
}

void ConeTwistConstraint::update() {
    PointToPointConstraint::update();

    bodyA->vectorToWorldFrame(axisA, coneEquation->axisA);
    bodyB->vectorToWorldFrame(axisB, coneEquation->axisB);

    Vec3 twistAxisA = axisA;
    Vec3 twistAxisB = axisB;
    axisA.tangents(twistAxisA, twistAxisA);
    bodyA->vectorToWorldFrame(twistAxisA, twistEquation->axisA);

    axisB.tangents(twistAxisB, twistAxisB);
    bodyB->vectorToWorldFrame(twistAxisB, twistEquation->axisB);

    coneEquation->angle = angle;
    twistEquation->maxAngle = twistAngle;
}

} // namespace cannon
} // namespace ge
