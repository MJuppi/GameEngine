#version 450

layout(location = 0) in vec4 inColor;
layout(location = 1) in vec2 inUV;
layout(location = 2) flat in int inHasTexture;

layout(binding = 2) uniform sampler2D uiTexture;

layout(location = 0) out vec4 outColor;

void main() {
    if (inHasTexture != 0) {
        float alpha = texture(uiTexture, inUV).r;
        outColor = vec4(inColor.rgb, inColor.a * alpha);
    } else {
        outColor = inColor;
    }
}
