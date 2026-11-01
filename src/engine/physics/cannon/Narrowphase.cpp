#include "engine/physics/cannon/Narrowphase.h"
#include "engine/physics/cannon/Equation.h"
#include "engine/physics/cannon/Body.h"
#include "engine/physics/cannon/World.h"
#include "engine/physics/cannon/Material.h"
#include "engine/physics/BoxCollider.h"
#include "engine/physics/SphereCollider.h"
#include <cmath>
#include <algorithm>

namespace ge {
namespace cannon {

namespace {
void getBoxAxes(const Quaternion& q, Vec3& ax, Vec3& ay, Vec3& az) {
    ax = q.vmult(Vec3(1.0f, 0.0f, 0.0f));
    ay = q.vmult(Vec3(0.0f, 1.0f, 0.0f));
    az = q.vmult(Vec3(0.0f, 0.0f, 1.0f));
}

float getComponent(const Vec3& value, int axis) {
    return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
}

void setComponent(Vec3& value, int axis, float component) {
    if (axis == 0) {
        value.x = component;
    } else if (axis == 1) {
        value.y = component;
    } else {
        value.z = component;
    }
}

float projectedRadiusOnAxis(const glm::vec3& halfExtents,
                            const Vec3& axis,
                            const Vec3& bx,
                            const Vec3& by,
                            const Vec3& bz) {
    return std::fabs(axis.dot(bx)) * halfExtents.x +
           std::fabs(axis.dot(by)) * halfExtents.y +
           std::fabs(axis.dot(bz)) * halfExtents.z;
}

Vec3 supportPointOnOBB(const Vec3& center,
                       const glm::vec3& halfExtents,
                       const Vec3& bx,
                       const Vec3& by,
                       const Vec3& bz,
                       const Vec3& direction) {
    const float sx = (direction.dot(bx) >= 0.0f) ? 1.0f : -1.0f;
    const float sy = (direction.dot(by) >= 0.0f) ? 1.0f : -1.0f;
    const float sz = (direction.dot(bz) >= 0.0f) ? 1.0f : -1.0f;

    Vec3 out = center;
    out.addScaledVector(sx * halfExtents.x, bx);
    out.addScaledVector(sy * halfExtents.y, by);
    out.addScaledVector(sz * halfExtents.z, bz);
    return out;
}

bool pointInOBB(const Vec3& point,
                const Vec3& center,
                const glm::vec3& halfExtents,
                const Quaternion& orientation,
                float margin = 1e-3f) {
    Vec3 local = point;
    local.sub(center);
    Quaternion inv = orientation;
    inv.conjugate();
    local = inv.vmult(local);
    return std::fabs(local.x) <= (halfExtents.x + margin) &&
           std::fabs(local.y) <= (halfExtents.y + margin) &&
           std::fabs(local.z) <= (halfExtents.z + margin);
}

void tangentsFromNormal(const Vec3& normal, Vec3& t1, Vec3& t2) {
    const Vec3 reference = (std::fabs(normal.x) < 0.9f) ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(0.0f, 1.0f, 0.0f);
    t1 = normal.cross(reference);
    if (t1.lengthSquared() < 1e-12f) {
        t1 = Vec3(0.0f, 0.0f, 1.0f);
    }
    t1.normalize();
    t2 = normal.cross(t1);
    t2.normalize();
}

void getShapeWorldTransform(const Body* body, Vec3& worldPosition, Quaternion& worldOrientation) {
    worldOrientation = body->quaternion.mult(body->shapeOrientation);
    const Vec3 worldOffset = body->quaternion.vmult(body->shapeOffset);
    worldPosition = body->position;
    worldPosition.vadd(worldOffset);
}

bool getSphereBoxContact(const SphereCollider& sphere,
                         const BoxCollider& box,
                         const Vec3& sphereCenter,
                         const Vec3& boxCenter,
                         const Quaternion& boxOrientation,
                         Vec3& normalSphereToBox,
                         Vec3& sphereContactPoint,
                         Vec3& boxContactPoint) {
    const glm::vec3 halfExtents = box.getHalfExtents();
    Vec3 sphereToBox = sphereCenter;
    sphereToBox.sub(boxCenter);
    Quaternion inverseOrientation = boxOrientation;
    inverseOrientation.conjugate();
    const Vec3 localCenter = inverseOrientation.vmult(sphereToBox);

    Vec3 closest(
        std::fmax(-halfExtents.x, std::fmin(localCenter.x, halfExtents.x)),
        std::fmax(-halfExtents.y, std::fmin(localCenter.y, halfExtents.y)),
        std::fmax(-halfExtents.z, std::fmin(localCenter.z, halfExtents.z)));
    boxContactPoint = boxOrientation.vmult(closest);
    boxContactPoint.vadd(boxCenter);

    const bool inside = std::fabs(localCenter.x) <= halfExtents.x &&
                        std::fabs(localCenter.y) <= halfExtents.y &&
                        std::fabs(localCenter.z) <= halfExtents.z;
    if (inside) {
        const float faceDistances[3] = {
            halfExtents.x - std::fabs(localCenter.x),
            halfExtents.y - std::fabs(localCenter.y),
            halfExtents.z - std::fabs(localCenter.z)
        };
        const int faceAxis = static_cast<int>(
            std::min_element(std::begin(faceDistances), std::end(faceDistances)) -
            std::begin(faceDistances));
        Vec3 outwardLocal(0.0f, 0.0f, 0.0f);
        const float faceSign = getComponent(localCenter, faceAxis) >= 0.0f ? 1.0f : -1.0f;
        setComponent(outwardLocal, faceAxis, faceSign);
        setComponent(closest, faceAxis, faceSign * halfExtents[faceAxis]);
        boxContactPoint = boxOrientation.vmult(closest);
        boxContactPoint.vadd(boxCenter);
        normalSphereToBox = boxOrientation.vmult(outwardLocal);
        normalSphereToBox.negate();
    } else {
        normalSphereToBox = boxContactPoint;
        normalSphereToBox.sub(sphereCenter);
        const float distanceSquared = normalSphereToBox.lengthSquared();
        const float radius = sphere.getRadius();
        if (distanceSquared > radius * radius) {
            return false;
        }
        if (distanceSquared > 1e-12f) {
            normalSphereToBox.scale(1.0f / std::sqrt(distanceSquared));
        } else {
            normalSphereToBox.set(0.0f, 1.0f, 0.0f);
        }
    }

    sphereContactPoint = sphereCenter;
    sphereContactPoint.addScaledVector(sphere.getRadius(), normalSphereToBox);
    return true;
}

} // namespace

Narrowphase::Narrowphase(World* world_)
    : enableFrictionReduction(false), world_(world_), contactPointPool_(), frictionEquationPool_() {}

Narrowphase::~Narrowphase() {
    for (ContactEquation* c : contactPointPool_) {
        delete c;
    }
    for (FrictionEquation* c : frictionEquationPool_) {
        delete c;
    }
}

void Narrowphase::releaseEquationPools(std::vector<ContactEquation*>& contacts,
                                       std::vector<FrictionEquation*>& frictionEquations) {
    contactPointPool_.reserve(contactPointPool_.size() + contacts.size());
    for (ContactEquation* c : contacts) {
        if (c) {
            contactPointPool_.push_back(c);
        }
    }
    contacts.clear();

    frictionEquationPool_.reserve(frictionEquationPool_.size() + frictionEquations.size());
    for (FrictionEquation* c : frictionEquations) {
        if (c) {
            frictionEquationPool_.push_back(c);
        }
    }
    frictionEquations.clear();
}

bool Narrowphase::bodiesOverlap(const Body* bi, const Body* bj) const {
    if (!bi || !bj || !bi->collider || !bj->collider) {
        return false;
    }

    Vec3 xi;
    Vec3 xj;
    Quaternion qi;
    Quaternion qj;
    getShapeWorldTransform(bi, xi, qi);
    getShapeWorldTransform(bj, xj, qj);

    if (bi->collider->getType() == ColliderType::Sphere && bj->collider->getType() == ColliderType::Sphere) {
        const SphereCollider* si = static_cast<const SphereCollider*>(bi->collider.get());
        const SphereCollider* sj = static_cast<const SphereCollider*>(bj->collider.get());
        Vec3 d = xj;
        d.sub(xi);
        const float r = si->getRadius() + sj->getRadius();
        return d.lengthSquared() <= r * r;
    }

    if (bi->collider->getType() == ColliderType::Sphere && bj->collider->getType() == ColliderType::Box) {
        const SphereCollider* sphere = static_cast<const SphereCollider*>(bi->collider.get());
        const BoxCollider* box = static_cast<const BoxCollider*>(bj->collider.get());

        Vec3 sphereToBox = xi;
        sphereToBox.sub(xj);
        Quaternion qjInv = qj;
        qjInv.conjugate();
        const Vec3 local = qjInv.vmult(sphereToBox);

        const glm::vec3 he = box->getHalfExtents();
        const Vec3 closest(
            std::fmax(-he.x, std::fmin(local.x, he.x)),
            std::fmax(-he.y, std::fmin(local.y, he.y)),
            std::fmax(-he.z, std::fmin(local.z, he.z)));

        Vec3 closestWorld = qj.vmult(closest);
        closestWorld.vadd(xj);
        Vec3 diff = closestWorld;
        diff.sub(xi);
        const float r = sphere->getRadius();
        return diff.lengthSquared() <= r * r;
    }

    if (bi->collider->getType() == ColliderType::Box && bj->collider->getType() == ColliderType::Sphere) {
        return bodiesOverlap(bj, bi);
    }

    if (bi->collider->getType() == ColliderType::Box && bj->collider->getType() == ColliderType::Box) {
        const BoxCollider* boxA = static_cast<const BoxCollider*>(bi->collider.get());
        const BoxCollider* boxB = static_cast<const BoxCollider*>(bj->collider.get());
        const glm::vec3 heA = boxA->getHalfExtents();
        const glm::vec3 heB = boxB->getHalfExtents();

        Vec3 ax, ay, az;
        Vec3 bx, by, bz;
        getBoxAxes(qi, ax, ay, az);
        getBoxAxes(qj, bx, by, bz);

        const Vec3 delta(xj.x - xi.x, xj.y - xi.y, xj.z - xi.z);
        const Vec3 axesA[3] = {ax, ay, az};
        const Vec3 axesB[3] = {bx, by, bz};
        std::vector<Vec3> testAxes = {ax, ay, az, bx, by, bz};
        for (const Vec3& axisA : axesA) {
            for (const Vec3& axisB : axesB) {
                testAxes.push_back(axisA.cross(axisB));
            }
        }
        for (const Vec3& axisIn : testAxes) {
            Vec3 axis = axisIn;
            const float axisLenSq = axis.lengthSquared();
            if (axisLenSq < 1e-10f) {
                continue;
            }
            axis.scale(1.0f / std::sqrt(axisLenSq));

            const float rA = projectedRadiusOnAxis(heA, axis, ax, ay, az);
            const float rB = projectedRadiusOnAxis(heB, axis, bx, by, bz);
            const float dist = std::fabs(delta.dot(axis));
            if (rA + rB - dist < 0.0f) {
                return false;
            }
        }
        return true;
    }

    return false;
}

void Narrowphase::getContacts(const std::vector<Body*>& p1,
                              const std::vector<Body*>& p2,
                              World& world,
                              std::vector<ContactEquation*>& contacts,
                              std::vector<FrictionEquation*>& frictionEquations,
                              std::vector<FrictionEquation*>& frictionPool) {
    (void)frictionPool;
    const size_t n = p1.size();
    contacts.reserve(contacts.size() + n);
    frictionEquations.reserve(frictionEquations.size() + (n * 2));

    auto emitContact = [&](Body* bi,
                           Body* bj,
                           const Material* shapeMatA,
                           const Material* shapeMatB,
                           const Vec3& normal,
                           const Vec3& contactPointI,
                           const Vec3& contactPointJ) {
        ContactEquation* contact = createContactEquation(bi, bj, shapeMatA, shapeMatB, world);
        if (!contact) {
            return;
        }

        contact->ni = normal;
        contact->ni.normalize();

        contact->ri = contactPointI;
        contact->ri.sub(bi->position);
        contact->rj = contactPointJ;
        contact->rj.sub(bj->position);

        contacts.push_back(contact);
    };

    auto finalizePairFriction = [&](size_t contactStartIndex,
                                    const Material* shapeMatA,
                                    const Material* shapeMatB) {
        const size_t numContacts = contacts.size() - contactStartIndex;
        if (numContacts == 0) {
            return;
        }

        if (enableFrictionReduction && numContacts > 1) {
            createFrictionFromAverage(contactStartIndex,
                                      numContacts,
                                      shapeMatA,
                                      shapeMatB,
                                      world,
                                      contacts,
                                      frictionEquations,
                                      frictionEquationPool_);
            return;
        }

        for (size_t idx = contactStartIndex; idx < contacts.size(); ++idx) {
            createFrictionEquationsFromContact(contacts[idx],
                                               shapeMatA,
                                               shapeMatB,
                                               world,
                                               frictionEquations,
                                               frictionEquationPool_);
        }
    };

    for (size_t i = 0; i < n; ++i) {
        Body* bi = p1[i];
        Body* bj = p2[i];
        const size_t contactStartIndex = contacts.size();
        const Material* shapeMatA = (bi && bi->collider) ? bi->collider->getMaterial() : nullptr;
        const Material* shapeMatB = (bj && bj->collider) ? bj->collider->getMaterial() : nullptr;

        if (!bi || !bj) {
            continue;
        }
        if (!bi->collider || !bj->collider) {
            continue;
        }
        if (bi->isTrigger || bj->isTrigger) {
            continue;
        }
        if (((bi->collisionMask & bj->collisionLayer) == 0u) ||
            ((bj->collisionMask & bi->collisionLayer) == 0u)) {
            continue;
        }

        Vec3 xi;
        Vec3 xj;
        Quaternion qi;
        Quaternion qj;
        getShapeWorldTransform(bi, xi, qi);
        getShapeWorldTransform(bj, xj, qj);

        if (bi->collider->getType() == ColliderType::Sphere && bj->collider->getType() == ColliderType::Sphere) {
            const SphereCollider* si = static_cast<const SphereCollider*>(bi->collider.get());
            const SphereCollider* sj = static_cast<const SphereCollider*>(bj->collider.get());

            Vec3 rij = xj;
            rij.sub(xi);
            const float dist = rij.length();
            const float r = si->getRadius() + sj->getRadius();

            if (dist < r) {
                Vec3 normal = rij;
                if (dist > 1e-6f) {
                    normal.scale(1.0f / dist);
                } else {
                    normal.set(1.0f, 0.0f, 0.0f);
                }

                Vec3 contactPointI = xi;
                contactPointI.addScaledVector(si->getRadius(), normal);
                Vec3 contactPointJ = xj;
                contactPointJ.addScaledVector(-sj->getRadius(), normal);
                emitContact(bi, bj, shapeMatA, shapeMatB, normal, contactPointI, contactPointJ);
            }
            finalizePairFriction(contactStartIndex, shapeMatA, shapeMatB);
            continue;
        }

        if (bi->collider->getType() == ColliderType::Sphere && bj->collider->getType() == ColliderType::Box) {
            const SphereCollider* sphere = static_cast<const SphereCollider*>(bi->collider.get());
            const BoxCollider* box = static_cast<const BoxCollider*>(bj->collider.get());
            Vec3 normal;
            Vec3 sphereContactPoint;
            Vec3 boxContactPoint;
            if (getSphereBoxContact(*sphere, *box, xi, xj, qj,
                                   normal, sphereContactPoint, boxContactPoint)) {
                emitContact(bi, bj, shapeMatA, shapeMatB,
                            normal, sphereContactPoint, boxContactPoint);
            }
            finalizePairFriction(contactStartIndex, shapeMatA, shapeMatB);
            continue;
        }

        if (bi->collider->getType() == ColliderType::Box && bj->collider->getType() == ColliderType::Sphere) {
            const SphereCollider* sphere = static_cast<const SphereCollider*>(bj->collider.get());
            const BoxCollider* box = static_cast<const BoxCollider*>(bi->collider.get());
            Vec3 normalSphereToBox;
            Vec3 sphereContactPoint;
            Vec3 boxContactPoint;
            if (getSphereBoxContact(*sphere, *box, xj, xi, qi,
                                   normalSphereToBox, sphereContactPoint, boxContactPoint)) {
                normalSphereToBox.negate();
                emitContact(bi, bj, shapeMatA, shapeMatB,
                            normalSphereToBox, boxContactPoint, sphereContactPoint);
            }
            finalizePairFriction(contactStartIndex, shapeMatA, shapeMatB);
            continue;
        }

        if (bi->collider->getType() == ColliderType::Box && bj->collider->getType() == ColliderType::Box) {
            const BoxCollider* boxA = static_cast<const BoxCollider*>(bi->collider.get());
            const BoxCollider* boxB = static_cast<const BoxCollider*>(bj->collider.get());
            const glm::vec3 heA = boxA->getHalfExtents();
            const glm::vec3 heB = boxB->getHalfExtents();

            Vec3 ax, ay, az;
            Vec3 bx, by, bz;
            getBoxAxes(qi, ax, ay, az);
            getBoxAxes(qj, bx, by, bz);

            const Vec3 delta(xj.x - xi.x, xj.y - xi.y, xj.z - xi.z);
            const Vec3 axesA[3] = {ax, ay, az};
            const Vec3 axesB[3] = {bx, by, bz};
            std::vector<Vec3> testAxes = {ax, ay, az, bx, by, bz};
            for (const Vec3& axisA : axesA) {
                for (const Vec3& axisB : axesB) {
                    testAxes.push_back(axisA.cross(axisB));
                }
            }
            float minOverlap = 1e30f;
            Vec3 bestAxis(1.0f, 0.0f, 0.0f);
            int bestAxisIndex = 0;
            bool separated = false;

            for (int axisIndex = 0; axisIndex < static_cast<int>(testAxes.size()); ++axisIndex) {
                const Vec3& axisIn = testAxes[axisIndex];
                Vec3 axis = axisIn;
                const float axisLenSq = axis.lengthSquared();
                if (axisLenSq < 1e-10f) {
                    continue;
                }
                axis.scale(1.0f / std::sqrt(axisLenSq));

                const float rA = projectedRadiusOnAxis(heA, axis, ax, ay, az);
                const float rB = projectedRadiusOnAxis(heB, axis, bx, by, bz);
                const float dist = std::fabs(delta.dot(axis));
                const float overlap = rA + rB - dist;

                if (overlap < 0.0f) {
                    separated = true;
                    break;
                }

                if (overlap < minOverlap) {
                    minOverlap = overlap;
                    bestAxis = axis;
                    bestAxisIndex = axisIndex;
                }
            }

            if (!separated) {
                Vec3 normal = bestAxis;
                if (delta.dot(normal) < 0.0f) {
                    normal.scale(-1.0f);
                }

                int emitted = 0;
                if (bestAxisIndex < 3) {
                    const Vec3 axesA[3] = {ax, ay, az};
                    const float heArrA[3] = {heA.x, heA.y, heA.z};
                    const int nIdx = bestAxisIndex;
                    const int t1Idx = (nIdx + 1) % 3;
                    const int t2Idx = (nIdx + 2) % 3;

                    Vec3 nAxis = axesA[nIdx];
                    if (normal.dot(nAxis) < 0.0f) {
                        nAxis.scale(-1.0f);
                    }
                    const Vec3 t1Axis = axesA[t1Idx];
                    const Vec3 t2Axis = axesA[t2Idx];

                    for (int s1 = -1; s1 <= 1; s1 += 2) {
                        for (int s2 = -1; s2 <= 1; s2 += 2) {
                            Vec3 contactPointI = xi;
                            contactPointI.addScaledVector(heArrA[nIdx], nAxis);
                            contactPointI.addScaledVector(static_cast<float>(s1) * heArrA[t1Idx], t1Axis);
                            contactPointI.addScaledVector(static_cast<float>(s2) * heArrA[t2Idx], t2Axis);

                            Vec3 contactPointJ = contactPointI;
                            contactPointJ.addScaledVector(-minOverlap, normal);

                            if (pointInOBB(contactPointJ, xj, heB, qj, 5e-2f)) {
                                emitContact(bi, bj, shapeMatA, shapeMatB, normal, contactPointI, contactPointJ);
                                emitted++;
                            }
                        }
                    }
                } else if (bestAxisIndex < 6) {
                    const Vec3 axesB[3] = {bx, by, bz};
                    const float heArrB[3] = {heB.x, heB.y, heB.z};
                    const int nIdx = bestAxisIndex - 3;
                    const int t1Idx = (nIdx + 1) % 3;
                    const int t2Idx = (nIdx + 2) % 3;

                    Vec3 nAxis = axesB[nIdx];
                    if (normal.dot(nAxis) > 0.0f) {
                        nAxis.scale(-1.0f);
                    }
                    const Vec3 t1Axis = axesB[t1Idx];
                    const Vec3 t2Axis = axesB[t2Idx];

                    for (int s1 = -1; s1 <= 1; s1 += 2) {
                        for (int s2 = -1; s2 <= 1; s2 += 2) {
                            Vec3 contactPointJ = xj;
                            contactPointJ.addScaledVector(heArrB[nIdx], nAxis);
                            contactPointJ.addScaledVector(static_cast<float>(s1) * heArrB[t1Idx], t1Axis);
                            contactPointJ.addScaledVector(static_cast<float>(s2) * heArrB[t2Idx], t2Axis);

                            Vec3 contactPointI = contactPointJ;
                            contactPointI.addScaledVector(minOverlap, normal);

                            if (pointInOBB(contactPointI, xi, heA, qi, 5e-2f)) {
                                emitContact(bi, bj, shapeMatA, shapeMatB, normal, contactPointI, contactPointJ);
                                emitted++;
                            }
                        }
                    }
                }

                if (emitted == 0) {
                    Vec3 negNormal = normal;
                    negNormal.scale(-1.0f);
                    Vec3 contactPointI = supportPointOnOBB(xi, heA, ax, ay, az, normal);
                    Vec3 contactPointJ = supportPointOnOBB(xj, heB, bx, by, bz, negNormal);
                    emitContact(bi, bj, shapeMatA, shapeMatB, normal, contactPointI, contactPointJ);
                }
            }
            finalizePairFriction(contactStartIndex, shapeMatA, shapeMatB);
        }
    }
}

ContactEquation* Narrowphase::createContactEquation(Body* bi,
                                                    Body* bj,
                                                    const Material* shapeMatA,
                                                    const Material* shapeMatB,
                                                    World& world) {
    if (!bi || !bj) {
        return nullptr;
    }

    ContactEquation* c = nullptr;
    if (!contactPointPool_.empty()) {
        c = contactPointPool_.back();
        contactPointPool_.pop_back();
        c->bi = bi;
        c->bj = bj;
    } else {
        c = new ContactEquation(bi, bj);
    }
    c->enabled = true;
    c->multiplier = 0.0f;

    const Material* matA = shapeMatA ? shapeMatA : bi->material;
    const Material* matB = shapeMatB ? shapeMatB : bj->material;
    const ContactMaterial& cm = world.getContactMaterial(matA, matB);

    c->restitution = std::max({cm.restitution, bi->restitution, bj->restitution});
    c->setSpookParams(cm.contactEquationStiffness, cm.contactEquationRelaxation, std::max(world.dt, 1e-6f));
    c->minForce = 0.0f;
    c->maxForce = 1e7f;

    if (matA && matB && matA->restitution >= 0.0f && matB->restitution >= 0.0f) {
        c->restitution = matA->restitution * matB->restitution;
    }

    return c;
}

bool Narrowphase::createFrictionEquationsFromContact(ContactEquation* contactEquation,
                                                     const Material* shapeMatA,
                                                     const Material* shapeMatB,
                                                     World& world,
                                                     std::vector<FrictionEquation*>& outArray,
                                                     std::vector<FrictionEquation*>& pool) {
    if (!contactEquation) {
        return false;
    }

    Body* bodyA = contactEquation->bi;
    Body* bodyB = contactEquation->bj;
    if (!bodyA || !bodyB) {
        return false;
    }

    const Material* matA = shapeMatA ? shapeMatA : bodyA->material;
    const Material* matB = shapeMatB ? shapeMatB : bodyB->material;
    const ContactMaterial& cm = world.getContactMaterial(matA, matB);
    const float gravityMagnitude = (world.frictionGravity > 0.0f) ? world.frictionGravity : world.gravity.length();
    const float invMassSum = bodyA->invMass + bodyB->invMass;
    float reducedMass = 0.0f;
    if (invMassSum > 0.0f) {
        reducedMass = 1.0f / invMassSum;
    }

    Vec3 velocityA = bodyA->angularVelocity.cross(contactEquation->ri);
    velocityA.vadd(bodyA->velocity);
    Vec3 velocityB = bodyB->angularVelocity.cross(contactEquation->rj);
    velocityB.vadd(bodyB->velocity);
    velocityB.vsub(velocityA);
    velocityB.addScaledVector(-velocityB.dot(contactEquation->ni), contactEquation->ni);
    const bool isStaticContact = velocityB.lengthSquared() <= 0.01f;

    auto combineFriction = [](float frictionA, float frictionB) {
        return std::sqrt(std::max(0.0f, frictionA) * std::max(0.0f, frictionB));
    };
    float dynamicFriction = combineFriction(bodyA->friction, bodyB->friction);
    float staticFriction = combineFriction(bodyA->staticFriction, bodyB->staticFriction);
    if (matA && matB && matA->friction >= 0.0f && matB->friction >= 0.0f) {
        dynamicFriction = matA->friction * matB->friction;
        staticFriction = dynamicFriction;
    }

    const float friction = isStaticContact ? staticFriction : dynamicFriction;
    const float slipForce = friction * gravityMagnitude * reducedMass;

    FrictionEquation* c1 = nullptr;
    FrictionEquation* c2 = nullptr;

    if (!pool.empty()) {
        c1 = pool.back();
        pool.pop_back();
        c1->bi = bodyA;
        c1->bj = bodyB;
    } else {
        c1 = new FrictionEquation(bodyA, bodyB, slipForce);
    }

    if (!pool.empty()) {
        c2 = pool.back();
        pool.pop_back();
        c2->bi = bodyA;
        c2->bj = bodyB;
    } else {
        c2 = new FrictionEquation(bodyA, bodyB, slipForce);
    }

    c1->minForce = -slipForce;
    c1->maxForce = slipForce;
    c2->minForce = -slipForce;
    c2->maxForce = slipForce;
    c1->angularOnly = false;
    c2->angularOnly = false;

    c1->ri = contactEquation->ri;
    c1->rj = contactEquation->rj;
    c2->ri = contactEquation->ri;
    c2->rj = contactEquation->rj;

    tangentsFromNormal(contactEquation->ni, c1->t, c2->t);

    c1->setSpookParams(cm.frictionEquationStiffness, cm.frictionEquationRelaxation, std::max(world.dt, 1e-6f));
    c2->setSpookParams(cm.frictionEquationStiffness, cm.frictionEquationRelaxation, std::max(world.dt, 1e-6f));
    c1->enabled = contactEquation->enabled;
    c2->enabled = contactEquation->enabled;

    if (friction > 0.0f) {
        outArray.push_back(c1);
        outArray.push_back(c2);
    } else {
        pool.push_back(c1);
        pool.push_back(c2);
    }

    const float rollingFriction = combineFriction(bodyA->rollingFriction, bodyB->rollingFriction);
    if (rollingFriction <= 0.0f || reducedMass <= 0.0f) {
        return friction > 0.0f;
    }

    auto shapeRadius = [](const Body* body) {
        if (!body->collider) {
            return 0.0f;
        }
        if (body->collider->getType() == ColliderType::Sphere) {
            return static_cast<const SphereCollider*>(body->collider.get())->getRadius();
        }
        const glm::vec3 halfExtents =
            static_cast<const BoxCollider*>(body->collider.get())->getHalfExtents();
        return std::min({halfExtents.x, halfExtents.y, halfExtents.z});
    };

    float radius = std::numeric_limits<float>::max();
    if (bodyA->invMass > 0.0f) {
        radius = std::min(radius, shapeRadius(bodyA));
    }
    if (bodyB->invMass > 0.0f) {
        radius = std::min(radius, shapeRadius(bodyB));
    }
    if (radius == std::numeric_limits<float>::max()) {
        radius = 0.0f;
    }

    const float rollingLimit = rollingFriction * gravityMagnitude * reducedMass * radius;
    if (rollingLimit <= 0.0f) {
        return friction > 0.0f;
    }

    FrictionEquation* rolling1 = nullptr;
    FrictionEquation* rolling2 = nullptr;
    auto acquireEquation = [&pool, contactEquation, &cm, &world](Body* a, Body* b, float limit) {
        FrictionEquation* equation = nullptr;
        if (!pool.empty()) {
            equation = pool.back();
            pool.pop_back();
            equation->bi = a;
            equation->bj = b;
        } else {
            equation = new FrictionEquation(a, b, limit);
        }
        equation->minForce = -limit;
        equation->maxForce = limit;
        equation->ri.setZero();
        equation->rj.setZero();
        equation->angularOnly = true;
        equation->enabled = contactEquation->enabled;
        equation->setSpookParams(cm.frictionEquationStiffness,
                                 cm.frictionEquationRelaxation,
                                 std::max(world.dt, 1e-6f));
        return equation;
    };

    rolling1 = acquireEquation(bodyA, bodyB, rollingLimit);
    rolling2 = acquireEquation(bodyA, bodyB, rollingLimit);
    tangentsFromNormal(contactEquation->ni, rolling1->t, rolling2->t);
    outArray.push_back(rolling1);
    outArray.push_back(rolling2);
    return true;
}

void Narrowphase::createFrictionFromAverage(size_t contactStartIndex,
                                            size_t numContacts,
                                            const Material* shapeMatA,
                                            const Material* shapeMatB,
                                            World& world,
                                            std::vector<ContactEquation*>& contacts,
                                            std::vector<FrictionEquation*>& frictionEquations,
                                            std::vector<FrictionEquation*>& frictionPool) {
    if (numContacts == 0) {
        return;
    }

    ContactEquation* lastContact = contacts[contactStartIndex + numContacts - 1];
    const size_t frictionStart = frictionEquations.size();
    if (!createFrictionEquationsFromContact(lastContact,
                                            shapeMatA,
                                            shapeMatB,
                                            world,
                                            frictionEquations,
                                            frictionPool) ||
        numContacts == 1) {
        return;
    }

    FrictionEquation* f1 = frictionEquations[frictionStart];
    FrictionEquation* f2 = frictionEquations[frictionStart + 1];
    Vec3 averageNormal;
    Vec3 averageContactPointA;
    Vec3 averageContactPointB;
    averageNormal.setZero();
    averageContactPointA.setZero();
    averageContactPointB.setZero();

    Body* bodyA = lastContact->bi;
    for (size_t i = 0; i < numContacts; ++i) {
        ContactEquation* c = contacts[contactStartIndex + i];
        if (c->bi != bodyA) {
            averageNormal.vadd(c->ni);
            averageContactPointA.vadd(c->ri);
            averageContactPointB.vadd(c->rj);
        } else {
            Vec3 negNormal = c->ni;
            negNormal.scale(-1.0f);
            averageNormal.vadd(negNormal);
            averageContactPointA.vadd(c->rj);
            averageContactPointB.vadd(c->ri);
        }
    }

    const float invNumContacts = 1.0f / static_cast<float>(numContacts);
    averageContactPointA.scale(invNumContacts);
    averageContactPointB.scale(invNumContacts);
    f1->ri = averageContactPointA;
    f1->rj = averageContactPointB;
    f2->ri = f1->ri;
    f2->rj = f1->rj;

    averageNormal.normalize();
    tangentsFromNormal(averageNormal, f1->t, f2->t);
}

} // namespace cannon
} // namespace ge
