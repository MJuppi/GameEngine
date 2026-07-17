#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include "engine/physics/BoxCollider.h"
#include "engine/physics/SphereCollider.h"
#include <algorithm>
#include <limits>

namespace ge {

BoxCollider::BoxCollider(const glm::vec3& halfExtents)
    : halfExtents_(halfExtents) {
}

std::string BoxCollider::getType() const {
    return "Box";
}

void BoxCollider::getLocalBounds(glm::vec3& min, glm::vec3& max) const {
    min = -halfExtents_;
    max = halfExtents_;
}

namespace {

struct OBB {
    glm::vec3 center;
    glm::vec3 axes[3];
    glm::vec3 halfExtents;
};

OBB getOBB(const glm::vec3& halfExtents, const glm::mat4& transform) {
    OBB obb;
    obb.center = glm::vec3(transform[3]);
    obb.axes[0] = glm::normalize(glm::vec3(transform[0]));
    obb.axes[1] = glm::normalize(glm::vec3(transform[1]));
    obb.axes[2] = glm::normalize(glm::vec3(transform[2]));

    // Scale half-extents by the transform scale
    glm::vec3 scale = glm::vec3(
        glm::length(glm::vec3(transform[0])),
        glm::length(glm::vec3(transform[1])),
        glm::length(glm::vec3(transform[2]))
    );
    obb.halfExtents = halfExtents * scale;
    return obb;
}

float getOverlap(const OBB& a, const OBB& b, const glm::vec3& axis) {
    if (glm::length2(axis) < 0.0001f) return std::numeric_limits<float>::max();

    glm::vec3 n = glm::normalize(axis);

    float radiusA = 0.0f;
    for (int i = 0; i < 3; ++i) radiusA += std::abs(glm::dot(a.axes[i] * a.halfExtents[i], n));

    float radiusB = 0.0f;
    for (int i = 0; i < 3; ++i) radiusB += std::abs(glm::dot(b.axes[i] * b.halfExtents[i], n));

    float distance = std::abs(glm::dot(b.center - a.center, n));
    return (radiusA + radiusB) - distance;
}

struct ClipVertex {
    glm::vec3 v;
    uint32_t id;
};

void clip(std::vector<ClipVertex>& vIn, const glm::vec3& n, float offset, std::vector<ClipVertex>& vOut) {
    vOut.clear();
    if (vIn.empty()) return;

    ClipVertex vA = vIn.back();
    float distA = glm::dot(vA.v, n) - offset;

    for (const auto& vB : vIn) {
        float distB = glm::dot(vB.v, n) - offset;

        if (distA * distB < 0.0f) {
            float t = distA / (distA - distB);
            vOut.push_back({vA.v + t * (vB.v - vA.v), vA.id}); // Simplified ID for clipping
        }

        if (distB >= 0.0f) {
            vOut.push_back(vB);
        }

        vA = vB;
        distA = distB;
    }
}

} // namespace

ContactManifold BoxCollider::checkCollision(
    const Collider& other,
    const glm::mat4& transformA,
    const glm::mat4& transformB
) const {
    ContactManifold result;

    if (other.getType() == std::string("Box")) {
        const BoxCollider& boxB = static_cast<const BoxCollider&>(other);

        OBB obbA = getOBB(halfExtents_, transformA);
        OBB obbB = getOBB(boxB.getHalfExtents(), transformB);

        glm::vec3 axes[15];
        for (int i = 0; i < 3; ++i) axes[i] = obbA.axes[i];
        for (int i = 0; i < 3; ++i) axes[i + 3] = obbB.axes[i];

        int axisIdx = 6;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                axes[axisIdx++] = glm::cross(obbA.axes[i], obbB.axes[j]);
            }
        }

        float minOverlap = std::numeric_limits<float>::max();
        int bestAxisIdx = -1;

        for (int i = 0; i < 15; ++i) {
            float overlap = getOverlap(obbA, obbB, axes[i]);
            if (overlap <= 0.0f) return result;

            float bias = (i >= 6) ? 0.95f : 1.0f;
            if (overlap * bias < minOverlap) {
                minOverlap = overlap;
                bestAxisIdx = i;
            }
        }

        result.isColliding = true;
        glm::vec3 normal = glm::normalize(axes[bestAxisIdx]);
        if (glm::dot(obbB.center - obbA.center, normal) < 0.0f) {
            normal = -normal;
        }
        result.normal = normal;

        // Identification of Reference and Incident Boxes
        const OBB* ref = &obbA;
        const OBB* inc = &obbB;
        bool flipped = false;

        // If the best axis is from Box B, Box B is the reference box
        if (bestAxisIdx >= 3 && bestAxisIdx < 6) {
            ref = &obbB;
            inc = &obbA;
            flipped = true;
        }

        // Find the incident face
        glm::vec3 incNormal = flipped ? normal : -normal;
        int incAxisIdx = 0;
        float minDot = std::numeric_limits<float>::max();
        for (int i = 0; i < 3; ++i) {
            float d = glm::dot(incNormal, inc->axes[i]);
            if (d < minDot) { minDot = d; incAxisIdx = i; }
            if (-d < minDot) { minDot = -d; incAxisIdx = i + 3; }
        }

        glm::vec3 faceNormal = (incAxisIdx < 3) ? inc->axes[incAxisIdx] : -inc->axes[incAxisIdx - 3];

        // Incident face vertices
        std::vector<ClipVertex> incidentVertices;
        glm::vec3 incAxes[2];
        int incA0 = (incAxisIdx % 3 + 1) % 3;
        int incA1 = (incAxisIdx % 3 + 2) % 3;
        incAxes[0] = inc->axes[incA0];
        incAxes[1] = inc->axes[incA1];

        glm::vec3 faceCenter = inc->center + faceNormal * inc->halfExtents[incAxisIdx % 3];
        incidentVertices.push_back({faceCenter + incAxes[0] * inc->halfExtents[incA0] + incAxes[1] * inc->halfExtents[incA1], 0});
        incidentVertices.push_back({faceCenter - incAxes[0] * inc->halfExtents[incA0] + incAxes[1] * inc->halfExtents[incA1], 1});
        incidentVertices.push_back({faceCenter - incAxes[0] * inc->halfExtents[incA0] - incAxes[1] * inc->halfExtents[incA1], 2});
        incidentVertices.push_back({faceCenter + incAxes[0] * inc->halfExtents[incA0] - incAxes[1] * inc->halfExtents[incA1], 3});

        // Clipping against reference face planes
        int refAxisIdx = bestAxisIdx % 3;
        glm::vec3 refAxes[2];
        int refA0 = (refAxisIdx + 1) % 3;
        int refA1 = (refAxisIdx + 2) % 3;
        refAxes[0] = ref->axes[refA0];
        refAxes[1] = ref->axes[refA1];

        std::vector<ClipVertex> vOut;
        clip(incidentVertices, refAxes[0], glm::dot(ref->center, refAxes[0]) - ref->halfExtents[refA0], vOut);
        clip(vOut, -refAxes[0], -glm::dot(ref->center, refAxes[0]) - ref->halfExtents[refA0], incidentVertices);
        clip(incidentVertices, refAxes[1], glm::dot(ref->center, refAxes[1]) - ref->halfExtents[refA1], vOut);
        clip(vOut, -refAxes[1], -glm::dot(ref->center, refAxes[1]) - ref->halfExtents[refA1], incidentVertices);

        // Keep points below reference face
        float refOffset = glm::dot(ref->center, normal * (flipped ? -1.0f : 1.0f)) + ref->halfExtents[refAxisIdx];

        for (const auto& v : incidentVertices) {
            float depth = refOffset - glm::dot(v.v, normal * (flipped ? -1.0f : 1.0f));
            if (depth >= 0.0f) {
                Contact contact;
                contact.point = v.v;
                contact.normal = normal;
                contact.depth = depth;
                contact.persistentId = v.id; // More robust ID would hash features
                result.contacts.push_back(contact);
            }
        }

    } else if (other.getType() == std::string("Sphere")) {
        const SphereCollider& sphereB = static_cast<const SphereCollider&>(other);

        // Proper OBB-Sphere collision
        // Transform sphere center into Box's local space
        const glm::mat4 invTransformA = glm::inverse(transformA);
        const glm::vec3 localSphereCenter = glm::vec3(invTransformA * glm::vec4(glm::vec3(transformB[3]), 1.0f));
        const float sphereRadius = sphereB.getRadius();

        // Find closest point on AABB to local sphere center
        glm::vec3 localClosestPoint = localSphereCenter;
        localClosestPoint.x = std::clamp(localClosestPoint.x, -halfExtents_.x, halfExtents_.x);
        localClosestPoint.y = std::clamp(localClosestPoint.y, -halfExtents_.y, halfExtents_.y);
        localClosestPoint.z = std::clamp(localClosestPoint.z, -halfExtents_.z, halfExtents_.z);

        const float distanceSq = glm::distance2(localSphereCenter, localClosestPoint);

        if (distanceSq <= sphereRadius * sphereRadius) {
            result.isColliding = true;

            const float distance = glm::sqrt(distanceSq);
            Contact contact;
            contact.depth = sphereRadius - distance;

            if (distance > 0.001f) {
                // Transform local normal back to world space
                glm::vec3 localNormal = glm::normalize(localSphereCenter - localClosestPoint);
                contact.normal = glm::normalize(glm::vec3(transformA * glm::vec4(localNormal, 0.0f)));
            } else {
                contact.normal = glm::normalize(glm::vec3(transformA[1])); // Up vector of box as fallback
            }

            contact.point = glm::vec3(transformA * glm::vec4(localClosestPoint, 1.0f));
            result.contacts.push_back(contact);
            result.normal = contact.normal;
        }
    }

    return result;
}

void BoxCollider::getWorldBounds(glm::vec3& min, glm::vec3& max, const glm::mat4& transform) const {
    // Transform the 8 corners and find the min/max
    glm::vec3 corners[8] = {
        {-halfExtents_.x, -halfExtents_.y, -halfExtents_.z},
        { halfExtents_.x, -halfExtents_.y, -halfExtents_.z},
        {-halfExtents_.x,  halfExtents_.y, -halfExtents_.z},
        { halfExtents_.x,  halfExtents_.y, -halfExtents_.z},
        {-halfExtents_.x, -halfExtents_.y,  halfExtents_.z},
        { halfExtents_.x, -halfExtents_.y,  halfExtents_.z},
        {-halfExtents_.x,  halfExtents_.y,  halfExtents_.z},
        { halfExtents_.x,  halfExtents_.y,  halfExtents_.z}
    };

    min = glm::vec3(std::numeric_limits<float>::max());
    max = glm::vec3(std::numeric_limits<float>::lowest());

    for (const auto& corner : corners) {
        glm::vec3 worldCorner = glm::vec3(transform * glm::vec4(corner, 1.0f));
        min = glm::min(min, worldCorner);
        max = glm::max(max, worldCorner);
    }
}

} // namespace ge
