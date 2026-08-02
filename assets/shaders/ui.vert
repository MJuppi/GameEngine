#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inTexCoord;
layout(location = 3) in uint inMaterialIndex;

layout(push_constant) uniform PushConstants {
    vec2 uiPosition;
    vec2 uiSize;
    vec4 uiColor;
    vec4 uiUVRect; // x,y = min UV; z,w = max UV
    int hasTexture;
} pcs;

layout(location = 0) out vec4 outColor;
layout(location = 1) out vec2 outUV;
layout(location = 2) flat out int outHasTexture;

void main() {
    // Map 0..1 quad to screen space NDC (-1 to 1)
    vec2 pos = (inPosition.xy * pcs.uiSize + pcs.uiPosition) * 2.0 - 1.0;

    // Standard Vulkan NDC has Y pointing down, but we assume the caller handles
    // coordinate system consistency or expects this simple mapping.
    gl_Position = vec4(pos, 0.0, 1.0);
    outColor = pcs.uiColor;
    outUV = mix(pcs.uiUVRect.xy, pcs.uiUVRect.zw, inTexCoord);
    outHasTexture = pcs.hasTexture;
}
