#include "engine/physics/cannon/Body.h"
#include "engine/physics/BoxCollider.h"
#include "engine/physics/SphereCollider.h"
#include <glm/gtc/quaternion.hpp>

namespace ge {
namespace cannon {

Body::Body()
        : id(0), type(BodyType::STATIC), collisionLayer(0x1u), collisionMask(0xFFFFFFFFu), useGravity(true), position(), velocity(), force(), torque(), quaternion(), angularVelocity(),
    mass(0.0f), invMass(0.0f), invMassSolve(0.0f), invInertia(), invInertiaWorld(), invInertiaWorldSolve(), vlambda(), wlambda(), initPosition(), initQuaternion(),
      initVelocity(), initAngularVelocity(), allowSleep(true), sleepState(0), sleepSpeedLimit(0.1f), sleepTimeLimit(1.0f),
      timeLastSleepy(0.0f), wakeUpAfterNarrowphase(false), isTrigger(false), linearDamping(0.01f), angularDamping(0.01f),
    restitution(0.0f), material(nullptr), shapeOffset(), shapeOrientation(), shapeType(ColliderType::Box), collider(nullptr) {
}

Body::Body(const RigidBodyProps& props, const glm::mat4& transform)
        : id(0), type(props.isKinematic ? BodyType::KINEMATIC : (props.mass <= 0.0f ? BodyType::STATIC : BodyType::DYNAMIC)), collisionLayer(props.collisionLayer), collisionMask(props.collisionMask), useGravity(props.useGravity), position(), velocity(), force(), torque(),
    quaternion(), angularVelocity(), mass(props.mass), invMass(props.mass > 0.0f ? 1.0f / props.mass : 0.0f), invMassSolve(props.mass > 0.0f ? 1.0f / props.mass : 0.0f),
    invInertia(), invInertiaWorld(), invInertiaWorldSolve(), vlambda(), wlambda(), initPosition(), initQuaternion(), initVelocity(), initAngularVelocity(),
      allowSleep(true), sleepState(0), sleepSpeedLimit(0.1f), sleepTimeLimit(1.0f), timeLastSleepy(0.0f),
      wakeUpAfterNarrowphase(false), isTrigger(props.isTrigger), linearDamping(props.linearDamping), angularDamping(props.angularDamping),
            restitution(props.restitution), material(nullptr), shapeOffset(), shapeOrientation(), shapeType(ColliderType::Box), collider(nullptr) {
    position = Vec3(transform[3][0], transform[3][1], transform[3][2]);
}

void Body::addShape(std::unique_ptr<Collider> shape) {
    if (!shape) {
        return;
    }
    collider = std::move(shape);
    shapeType = collider->getType();
    updateMassProperties();
}

void Body::updateMassProperties() {
    if (mass <= 0.0f) {
        invMass = 0.0f;
        invMassSolve = 0.0f;
        invInertia = Vec3(0.0f, 0.0f, 0.0f);
        invInertiaWorld.setZero();
        invInertiaWorldSolve.setZero();
        return;
    }

    if (!collider) {
        invInertia = Vec3(0.0f, 0.0f, 0.0f);
        invInertiaWorld.setZero();
        invInertiaWorldSolve.setZero();
        return;
    }

    if (collider->getType() == ColliderType::Box) {
        const auto* box = static_cast<const BoxCollider*>(collider.get());
        const glm::vec3 halfExtents = box->getHalfExtents();
        const float x2 = halfExtents.x * halfExtents.x * 4.0f;
        const float y2 = halfExtents.y * halfExtents.y * 4.0f;
        const float z2 = halfExtents.z * halfExtents.z * 4.0f;
        invInertia.x = 3.0f / (mass * (y2 + z2));
        invInertia.y = 3.0f / (mass * (x2 + z2));
        invInertia.z = 3.0f / (mass * (x2 + y2));
    } else if (collider->getType() == ColliderType::Sphere) {
        const auto* sphere = static_cast<const SphereCollider*>(collider.get());
        const float radius = sphere->getRadius();
        if (radius <= 1e-6f) {
            invInertia.setZero();
            invInertiaWorld.setZero();
            invInertiaWorldSolve.setZero();
            return;
        }
        const float invInertiaScalar = 5.0f / (2.0f * mass * radius * radius);
        invInertia = Vec3(invInertiaScalar, invInertiaScalar, invInertiaScalar);
    }

    invInertiaWorld.identity();
    invInertiaWorld.elements[0] = invInertia.x;
    invInertiaWorld.elements[4] = invInertia.y;
    invInertiaWorld.elements[8] = invInertia.z;
    updateInertiaWorld();
}

void Body::updateInertiaWorld() {
    if (invInertia.x == 0.0f && invInertia.y == 0.0f && invInertia.z == 0.0f) {
        invInertiaWorld.setZero();
        invInertiaWorldSolve.setZero();
        return;
    }

    const Vec3 axisX = quaternion.vmult(Vec3(1.0f, 0.0f, 0.0f));
    const Vec3 axisY = quaternion.vmult(Vec3(0.0f, 1.0f, 0.0f));
    const Vec3 axisZ = quaternion.vmult(Vec3(0.0f, 0.0f, 1.0f));

    Mat3 rotation;
    rotation.elements = {
        axisX.x, axisY.x, axisZ.x,
        axisX.y, axisY.y, axisZ.y,
        axisX.z, axisY.z, axisZ.z
    };

    Mat3 localInverse;
    localInverse.setZero();
    localInverse.elements[0] = invInertia.x;
    localInverse.elements[4] = invInertia.y;
    localInverse.elements[8] = invInertia.z;

    Mat3 transpose;
    transpose.elements = {
        rotation.elements[0], rotation.elements[3], rotation.elements[6],
        rotation.elements[1], rotation.elements[4], rotation.elements[7],
        rotation.elements[2], rotation.elements[5], rotation.elements[8]
    };
    invInertiaWorld = rotation.mmult(localInverse).mmult(transpose);
    invInertiaWorldSolve = invInertiaWorld;
}

void Body::applyForce(const Vec3& f) {
    force.add(f);
}

void Body::integrate(float dt, bool quatNormalize) {
    if ((type != BodyType::DYNAMIC && type != BodyType::KINEMATIC) || sleepState == 2) {
        return;
    }

    if (type == BodyType::DYNAMIC) {
        velocity.addScaledVector(dt * invMass, force);

        Vec3 angularForce = invInertiaWorld.vmult(torque);
        angularVelocity.addScaledVector(dt, angularForce);
    }

    position.addScaledVector(dt, velocity);

    Vec3 axis = angularVelocity;
    const float len = axis.length();
    if (len > 1e-6f) {
        axis.scale(1.0f / len);
        Quaternion delta;
        delta.setFromAxisAngle(axis, len * dt);
        quaternion = quaternion.mult(delta);
    }

    if (quatNormalize) {
        quaternion.normalize();
    }

    updateInertiaWorld();
}

void Body::pointToWorldFrame(const Vec3& localPoint, Vec3& out) const {
    out = quaternion.vmult(localPoint);
    out.add(position);
}

void Body::pointToLocalFrame(const Vec3& worldPoint, Vec3& out) const {
    out = worldPoint;
    out.sub(position);
    Quaternion inv = quaternion;
    inv.conjugate();
    out = inv.vmult(out);
}

void Body::vectorToWorldFrame(const Vec3& localVector, Vec3& out) const {
    out = quaternion.vmult(localVector);
}

void Body::vectorToLocalFrame(const Vec3& worldVector, Vec3& out) const {
    Quaternion inv = quaternion;
    inv.conjugate();
    out = inv.vmult(worldVector);
}

void Body::wakeUp() {
    sleepState = 0;
    wakeUpAfterNarrowphase = false;
}

void Body::sleep() {
    sleepState = 2;
    velocity.setZero();
    angularVelocity.setZero();
    wakeUpAfterNarrowphase = false;
}

void Body::sleepTick(float time) {
    const float speedSquared = velocity.lengthSquared() + angularVelocity.lengthSquared();
    const float limitSquared = sleepSpeedLimit * sleepSpeedLimit;

    if (sleepState == 0 && speedSquared < limitSquared) {
        sleepState = 1;
        timeLastSleepy = time;
    } else if (sleepState == 1 && speedSquared > limitSquared) {
        wakeUp();
    } else if (sleepState == 1 && time - timeLastSleepy > sleepTimeLimit) {
        sleep();
    }
}

void Body::updateSolveMassProperties() {
    if (sleepState == 2 || type == BodyType::KINEMATIC) {
        invMassSolve = 0.0f;
        invInertiaWorldSolve.setZero();
    } else {
        invMassSolve = invMass;
        invInertiaWorldSolve = invInertiaWorld;
    }
}

} // namespace cannon
} // namespace ge
