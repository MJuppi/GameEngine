// =============================================================================
// Vertex shader — position, normal, material index
// =============================================================================

#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in uint inMaterialIndex;

struct PointLight {
    vec4 position;
    vec4 color;
    vec4 parameters;
};

layout(set = 0, binding = 0) uniform SceneUbo {
    mat4 model;
    mat4 viewProj;
    mat4 normalMatrix;
    vec4 lightDir;
    PointLight pointLight;
} scene;

layout(push_constant) uniform PushConstants {
    mat4 model;
    mat4 normalMatrix;
} push;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) flat out uint fragMaterialIndex;

void main() {
    fragNormal = normalize(mat3(push.normalMatrix) * inNormal);
    fragMaterialIndex = inMaterialIndex;
    gl_Position = scene.viewProj * push.model * vec4(inPosition, 1.0);
}
