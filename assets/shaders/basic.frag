// =============================================================================
// Fragment shader — Blinn-Phong using MTL diffuse/specular per material
// =============================================================================

#version 450

layout(location = 0) in vec3 fragNormal;
layout(location = 1) flat in uint fragMaterialIndex;

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

struct Material {
    vec4 diffuse;
    vec4 specular;
};

layout(set = 0, binding = 1) uniform MaterialUbo {
    Material items[16];
} materials;

layout(location = 0) out vec4 outColor;

void main() {
    uint idx = min(fragMaterialIndex, 15u);
    Material mat = materials.items[idx];

    vec3 N = normalize(fragNormal);
    vec3 ambientLight = mat.diffuse.rgb * 0.15;

    vec3 lightPos = scene.pointLight.position.xyz;
    vec3 fragPos = vec3(0.0);
    vec3 L = normalize(lightPos - fragPos);
    vec3 V = normalize(vec3(0.0, 0.0, 1.0));
    vec3 H = normalize(L + V);

    float distanceToLight = length(lightPos - fragPos);
    float attenuation = 1.0 / (scene.pointLight.parameters.x + scene.pointLight.parameters.y * distanceToLight + scene.pointLight.parameters.z * distanceToLight * distanceToLight);
    float diff = max(dot(N, L), 0.0);
    float spec = pow(max(dot(N, H), 0.0), mat.specular.w);

    vec3 diffuse = mat.diffuse.rgb * diff * attenuation * scene.pointLight.color.rgb;
    vec3 specular = mat.specular.rgb * spec * attenuation * scene.pointLight.color.rgb;

    outColor = vec4(ambientLight + diffuse + specular, mat.diffuse.a);
}
