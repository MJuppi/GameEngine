// =============================================================================
// Vertex shader — position, normal, UV, material index
// =============================================================================

#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inTexCoord;
layout(location = 3) in uint inMaterialIndex;

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
    vec4 cameraPos;
    PointLight pointLight;
} scene;

layout(push_constant) uniform PushConstants {
    mat4 model;
    mat4 normalMatrix;
} push;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec2 fragTexCoord;
layout(location = 2) flat out uint fragMaterialIndex;
layout(location = 3) out vec3 fragPos;

void main() {
    vec4 worldPos = push.model * vec4(inPosition, 1.0);
    fragPos = worldPos.xyz;
    fragNormal = normalize(mat3(push.normalMatrix) * inNormal);
    fragTexCoord = inTexCoord;
    fragMaterialIndex = inMaterialIndex;
    gl_Position = scene.viewProj * worldPos;
}
