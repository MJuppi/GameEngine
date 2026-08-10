#include "engine/physics/cannon/Narrowphase.h"
#include "engine/physics/cannon/Equation.h"
#include "engine/physics/cannon/Body.h"
#include "engine/physics/cannon/World.h"
#include "engine/physics/cannon/Material.h"
#include "engine/physics/BoxCollider.h"
#include "engine/physics/SphereCollider.h"
#include <cmath>

namespace ge {
namespace cannon {

namespace {

ContactEquation* buildContactEquation(World& world,
                                      Body* bi,
                                      Body* bj,
                                      const Vec3& normal,
                                      const Vec3& contactPointI,
                                      const Vec3& contactPointJ,
                                      const Material* matA,
                                      const Material* matB) {
    ContactMaterial* contactMaterial = nullptr;
    if (matA && matB && matA->name == matB->name) {
        contactMaterial = &world.defaultContactMaterial;
    }

    ContactEquation* eq = new ContactEquation(bi, bj);
    eq->ni = normal;

    Vec3 ri = contactPointI;
    ri.sub(bi->position);
    Vec3 rj = contactPointJ;
    rj.sub(bj->position);
    eq->ri = ri;
    eq->rj = rj;

    eq->restitution = contactMaterial ? contactMaterial->restitution : world.defaultContactMaterial.restitution;
    eq->minForce = 0.0f;
    eq->maxForce = 1e7f;
    return eq;
}

float extentAlongNormal(const glm::vec3& halfExtents, const Vec3& normal) {
    return std::fabs(normal.x) * halfExtents.x + std::fabs(normal.y) * halfExtents.y + std::fabs(normal.z) * halfExtents.z;
}

} // namespace

Narrowphase::Narrowphase(World* world_) : world_(world_) {}

void Narrowphase::getContacts(const std::vector<Body*>& p1,
                              const std::vector<Body*>& p2,
                              World& world,
                              std::vector<ContactEquation*>& contacts,
                              std::vector<ContactEquation*>& frictionEquations,
                              std::vector<ContactEquation*>& frictionPool) {
    (void)frictionEquations;
    (void)frictionPool;

    const size_t n = p1.size();
    contacts.reserve(contacts.size() + n);

    for (size_t i = 0; i < n; ++i) {
        Body* bi = p1[i];
        Body* bj = p2[i];

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

        if (bi->collider->getType() == ColliderType::Sphere && bj->collider->getType() == ColliderType::Sphere) {
            const SphereCollider* si = static_cast<const SphereCollider*>(bi->collider.get());
            const SphereCollider* sj = static_cast<const SphereCollider*>(bj->collider.get());

            Vec3 xi = bi->position;
            Vec3 xj = bj->position;
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

                const Material* matA = bi->material ? bi->material : &world.defaultMaterial;
                const Material* matB = bj->material ? bj->material : &world.defaultMaterial;
                Vec3 contactPointI = xi;
                contactPointI.addScaledVector(si->getRadius(), normal);
                Vec3 contactPointJ = xj;
                contactPointJ.addScaledVector(-sj->getRadius(), normal);
                contacts.push_back(buildContactEquation(world, bi, bj, normal, contactPointI, contactPointJ, matA, matB));
            }
            continue;
        }

        if (bi->collider->getType() == ColliderType::Sphere && bj->collider->getType() == ColliderType::Box) {
            const SphereCollider* sphere = static_cast<const SphereCollider*>(bi->collider.get());
            const BoxCollider* box = static_cast<const BoxCollider*>(bj->collider.get());

            const glm::vec3 he = box->getHalfExtents();
            const Vec3 centerSphere = bi->position;
            const Vec3 centerBox = bj->position;
            const Vec3 local(centerSphere.x - centerBox.x, centerSphere.y - centerBox.y, centerSphere.z - centerBox.z);

            const Vec3 closest(
                std::fmax(-he.x, std::fmin(local.x, he.x)),
                std::fmax(-he.y, std::fmin(local.y, he.y)),
                std::fmax(-he.z, std::fmin(local.z, he.z)));
            const Vec3 closestWorld(centerBox.x + closest.x, centerBox.y + closest.y, centerBox.z + closest.z);
            Vec3 diff = closestWorld;
            diff.sub(centerSphere);

            const float distSq = diff.lengthSquared();
            const float radius = sphere->getRadius();
            if (distSq <= radius * radius) {
                Vec3 normal = diff;
                const float dist = std::sqrt(std::fmax(0.0f, distSq));
                if (dist > 1e-6f) {
                    normal.scale(1.0f / dist);
                } else {
                    normal.set(0.0f, 1.0f, 0.0f);
                }

                const Material* matA = bi->material ? bi->material : &world.defaultMaterial;
                const Material* matB = bj->material ? bj->material : &world.defaultMaterial;
                Vec3 contactPointI = centerSphere;
                contactPointI.addScaledVector(sphere->getRadius(), normal);
                const Vec3 contactPointJ = closestWorld;
                contacts.push_back(buildContactEquation(world, bi, bj, normal, contactPointI, contactPointJ, matA, matB));
            }
            continue;
        }

        if (bi->collider->getType() == ColliderType::Box && bj->collider->getType() == ColliderType::Sphere) {
            const SphereCollider* sphere = static_cast<const SphereCollider*>(bj->collider.get());
            const BoxCollider* box = static_cast<const BoxCollider*>(bi->collider.get());

            const glm::vec3 he = box->getHalfExtents();
            const Vec3 centerSphere = bj->position;
            const Vec3 centerBox = bi->position;
            const Vec3 local(centerSphere.x - centerBox.x, centerSphere.y - centerBox.y, centerSphere.z - centerBox.z);

            const Vec3 closest(
                std::fmax(-he.x, std::fmin(local.x, he.x)),
                std::fmax(-he.y, std::fmin(local.y, he.y)),
                std::fmax(-he.z, std::fmin(local.z, he.z)));
            const Vec3 closestWorld(centerBox.x + closest.x, centerBox.y + closest.y, centerBox.z + closest.z);
            Vec3 diff = centerSphere;
            diff.sub(closestWorld);

            const float distSq = diff.lengthSquared();
            const float radius = sphere->getRadius();
            if (distSq <= radius * radius) {
                Vec3 normal = diff;
                const float dist = std::sqrt(std::fmax(0.0f, distSq));
                if (dist > 1e-6f) {
                    normal.scale(1.0f / dist);
                } else {
                    normal.set(0.0f, -1.0f, 0.0f);
                }

                const Material* matA = bi->material ? bi->material : &world.defaultMaterial;
                const Material* matB = bj->material ? bj->material : &world.defaultMaterial;
                const Vec3 contactPointI = closestWorld;
                Vec3 contactPointJ = centerSphere;
                contactPointJ.addScaledVector(-sphere->getRadius(), normal);
                contacts.push_back(buildContactEquation(world, bi, bj, normal, contactPointI, contactPointJ, matA, matB));
            }
            continue;
        }

        if (bi->collider->getType() == ColliderType::Box && bj->collider->getType() == ColliderType::Box) {
            const BoxCollider* boxA = static_cast<const BoxCollider*>(bi->collider.get());
            const BoxCollider* boxB = static_cast<const BoxCollider*>(bj->collider.get());
            const glm::vec3 heA = boxA->getHalfExtents();
            const glm::vec3 heB = boxB->getHalfExtents();

            const Vec3 delta(bj->position.x - bi->position.x, bj->position.y - bi->position.y, bj->position.z - bi->position.z);
            const float overlapX = heA.x + heB.x - std::fabs(delta.x);
            const float overlapY = heA.y + heB.y - std::fabs(delta.y);
            const float overlapZ = heA.z + heB.z - std::fabs(delta.z);

            if (overlapX >= 0.0f && overlapY >= 0.0f && overlapZ >= 0.0f) {
                float penetration = overlapX;
                Vec3 normal(delta.x >= 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f);
                if (overlapY < penetration) {
                    penetration = overlapY;
                    normal.set(0.0f, delta.y >= 0.0f ? 1.0f : -1.0f, 0.0f);
                }
                if (overlapZ < penetration) {
                    normal.set(0.0f, 0.0f, delta.z >= 0.0f ? 1.0f : -1.0f);
                }

                const float extentI = extentAlongNormal(heA, normal);
                const float extentJ = extentAlongNormal(heB, normal);
                Vec3 contactPointI = bi->position;
                contactPointI.addScaledVector(extentI, normal);
                Vec3 contactPointJ = bj->position;
                contactPointJ.addScaledVector(-extentJ, normal);
                const Material* matA = bi->material ? bi->material : &world.defaultMaterial;
                const Material* matB = bj->material ? bj->material : &world.defaultMaterial;
                contacts.push_back(buildContactEquation(world, bi, bj, normal, contactPointI, contactPointJ, matA, matB));
            }
        }
    }
}

} // namespace cannon
} // namespace ge
