// =============================================================================
// Fragment shader — Blinn-Phong using MTL diffuse/specular per material
// =============================================================================

#version 450

struct PointLight {
    vec4 position;
    vec4 color;
    vec4 parameters; // x: constant, y: linear, z: quadratic, w: intensity
};

struct DirectionalLight {
    vec4 direction;
    vec4 color;
    float intensity;
};

struct AmbientLight {
    vec4 color;
    float intensity;
};

struct SceneLights {
    DirectionalLight directional;
    AmbientLight ambient;
    PointLight pointLights[8];
    uint pointLightCount;
};

layout(set = 0, binding = 0) uniform SceneUbo {
    mat4 model;
    mat4 viewProj;
    mat4 normalMatrix;
    vec4 lightDir; // Obsolete, use lights.directional
    vec4 cameraPos;
    SceneLights lights;
} scene;

struct Material {
    vec4 diffuse;
    vec4 specular;
    uint hasTexture;
};

layout(set = 0, binding = 1) uniform MaterialUbo {
    Material items[16];
} materials;

layout(set = 0, binding = 2) uniform sampler2D texSampler;

layout(location = 0) in vec3 fragNormal;
layout(location = 1) in vec2 fragTexCoord;
layout(location = 2) flat in uint fragMaterialIndex;
layout(location = 3) in vec3 fragPos;

layout(location = 0) out vec4 outColor;

void main() {
    uint idx = min(fragMaterialIndex, 15u);
    Material mat = materials.items[idx];

    vec3 N = normalize(fragNormal);
    vec4 baseColor = mat.diffuse;
    if (mat.hasTexture != 0) {
        baseColor *= texture(texSampler, fragTexCoord);
    }

    vec3 ambientLight = baseColor.rgb * scene.lights.ambient.color.rgb * scene.lights.ambient.intensity;

    // Direct lighting (directional light)
    vec3 L_dir = normalize(scene.lights.directional.direction.xyz);
    float diff_dir = max(dot(N, L_dir), 0.0);
    vec3 diffuse_dir = baseColor.rgb * diff_dir * scene.lights.directional.color.rgb * scene.lights.directional.intensity;

    // Point lights
    vec3 totalPointDiffuse = vec3(0.0);
    vec3 totalPointSpecular = vec3(0.0);
    vec3 V = normalize(scene.cameraPos.xyz - fragPos);

    uint lightCount = min(scene.lights.pointLightCount, 8u);
    for (uint i = 0; i < lightCount; ++i) {
        PointLight pl = scene.lights.pointLights[i];
        vec3 lightPos = pl.position.xyz;
        vec3 L = normalize(lightPos - fragPos);
        vec3 H = normalize(L + V);

        float distanceToLight = length(lightPos - fragPos);
        float attenuation = 1.0 / (pl.parameters.x + pl.parameters.y * distanceToLight + pl.parameters.z * distanceToLight * distanceToLight);
        attenuation *= pl.parameters.w; // intensity

        float diff = max(dot(N, L), 0.0);
        float spec = pow(max(dot(N, H), 0.0), mat.specular.w);

        totalPointDiffuse += baseColor.rgb * diff * attenuation * pl.color.rgb;
        totalPointSpecular += mat.specular.rgb * spec * attenuation * pl.color.rgb;
    }

    outColor = vec4(ambientLight + diffuse_dir + totalPointDiffuse + totalPointSpecular, baseColor.a);
}
