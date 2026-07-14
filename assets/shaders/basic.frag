// =============================================================================
// Fragment shader — Blinn-Phong using MTL diffuse/specular per material
// =============================================================================

#version 450

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

    vec3 ambientLight = baseColor.rgb * 0.15;

    // Direct lighting (directional light from SceneUbo)
    vec3 L_dir = normalize(scene.lightDir.xyz);
    float diff_dir = max(dot(N, L_dir), 0.0);
    vec3 diffuse_dir = baseColor.rgb * diff_dir * 0.5; // Half strength for directional light

    // Point light
    vec3 lightPos = scene.pointLight.position.xyz;
    vec3 L = normalize(lightPos - fragPos);
    vec3 V = normalize(scene.cameraPos.xyz - fragPos);
    vec3 H = normalize(L + V);

    float distanceToLight = length(lightPos - fragPos);
    float attenuation = 1.0 / (scene.pointLight.parameters.x + scene.pointLight.parameters.y * distanceToLight + scene.pointLight.parameters.z * distanceToLight * distanceToLight);

    float diff = max(dot(N, L), 0.0);
    float spec = pow(max(dot(N, H), 0.0), mat.specular.w);

    vec3 diffuse = baseColor.rgb * diff * attenuation * scene.pointLight.color.rgb;
    vec3 specular = mat.specular.rgb * spec * attenuation * scene.pointLight.color.rgb;

    outColor = vec4(ambientLight + diffuse_dir + diffuse + specular, baseColor.a);
}
