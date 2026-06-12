// =============================================================================
// Vertex shader — position, normal, material index
// =============================================================================

#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in uint inMaterialIndex;

layout(set = 0, binding = 0) uniform SceneUbo {
    mat4 model;
    mat4 viewProj;
    mat4 normalMatrix;
    vec4 lightDir;
} scene;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) flat out uint fragMaterialIndex;

void main() {
    fragNormal = normalize(mat3(scene.normalMatrix) * inNormal);
    fragMaterialIndex = inMaterialIndex;
    gl_Position = scene.viewProj * scene.model * vec4(inPosition, 1.0);
}
