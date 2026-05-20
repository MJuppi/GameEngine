// =============================================================================
// Fragment shader — Blinn-Phong using MTL diffuse/specular per material
// =============================================================================

#version 450

layout(location = 0) in vec3 fragNormal;
layout(location = 1) flat in uint fragMaterialIndex;

layout(set = 0, binding = 0) uniform SceneUbo {
    mat4 model;
    mat4 viewProj;
    vec4 lightDir;
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
    vec3 L = normalize(-scene.lightDir.xyz);
    vec3 V = normalize(vec3(0.0, 0.0, 1.0));
    vec3 H = normalize(L + V);

    float diff = max(dot(N, L), 0.0);
    float spec = pow(max(dot(N, H), 0.0), mat.specular.w);

    vec3 ambient = mat.diffuse.rgb * 0.15;
    vec3 diffuse = mat.diffuse.rgb * diff;
    vec3 specular = mat.specular.rgb * spec;

    outColor = vec4(ambient + diffuse + specular, mat.diffuse.a);
}
