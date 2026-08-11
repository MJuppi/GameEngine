#pragma once

#include <vector>
#include "engine/physics/cannon/Vec3.h"

namespace ge {
namespace cannon {

class Body;
class Equation;
class ContactEquation;
class RotationalEquation;
class RotationalMotorEquation;
class ConeEquation;

class Constraint {
public:
    Constraint(Body* bodyA, Body* bodyB, bool collideConnected = true);
    virtual ~Constraint() = default;

    Body* bodyA;
    Body* bodyB;
    bool collideConnected;
    std::vector<Equation*> equations;

    virtual void update();
};

class PointToPointConstraint : public Constraint {
public:
    PointToPointConstraint(Body* bodyA,
                           const Vec3& pivotA,
                           Body* bodyB,
                           const Vec3& pivotB,
                           float maxForce = 1e6f,
                           bool collideConnected = true);
    ~PointToPointConstraint() override;

    Vec3 pivotA;
    Vec3 pivotB;
    ContactEquation* equationX;
    ContactEquation* equationY;
    ContactEquation* equationZ;

    void update() override;
};

class DistanceConstraint : public Constraint {
public:
    DistanceConstraint(Body* bodyA,
                       Body* bodyB,
                       float distance,
                       float maxForce = 1e6f,
                       bool collideConnected = true);
    ~DistanceConstraint() override;

    float distance;
    ContactEquation* distanceEquation;

    void update() override;
};

class LockConstraint : public PointToPointConstraint {
public:
    LockConstraint(Body* bodyA, Body* bodyB, float maxForce = 1e6f, bool collideConnected = true);
    ~LockConstraint() override;

    Vec3 xA;
    Vec3 xB;
    Vec3 yA;
    Vec3 yB;
    Vec3 zA;
    Vec3 zB;
    RotationalEquation* rotationalEquation1;
    RotationalEquation* rotationalEquation2;
    RotationalEquation* rotationalEquation3;

    void update() override;
};

class HingeConstraint : public PointToPointConstraint {
public:
    HingeConstraint(Body* bodyA,
                    Body* bodyB,
                    const Vec3& pivotA,
                    const Vec3& pivotB,
                    const Vec3& axisA,
                    const Vec3& axisB,
                    float maxForce = 1e6f,
                    bool collideConnected = true);
    ~HingeConstraint() override;

    Vec3 axisA;
    Vec3 axisB;
    RotationalEquation* rotationalEquation1;
    RotationalEquation* rotationalEquation2;
    RotationalMotorEquation* motorEquation;

    void enableMotor();
    void disableMotor();
    void setMotorSpeed(float speed);
    void setMotorMaxForce(float maxForce);
    void update() override;
};

class ConeTwistConstraint : public PointToPointConstraint {
public:
    ConeTwistConstraint(Body* bodyA,
                        Body* bodyB,
                        const Vec3& pivotA,
                        const Vec3& pivotB,
                        const Vec3& axisA,
                        const Vec3& axisB,
                        float angle = 0.0f,
                        float twistAngle = 0.0f,
                        float maxForce = 1e6f,
                        bool collideConnected = true);
    ~ConeTwistConstraint() override;

    Vec3 axisA;
    Vec3 axisB;
    float angle;
    float twistAngle;
    ConeEquation* coneEquation;
    RotationalEquation* twistEquation;

    void update() override;
};

} // namespace cannon
} // namespace ge
