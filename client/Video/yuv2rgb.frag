#version 440

layout(binding = 1) uniform sampler2D yTex;
layout(binding = 2) uniform sampler2D uTex;
layout(binding = 3) uniform sampler2D vTex;

layout(location = 0) out vec3 fragColor;
layout(location = 0) in vec2 vTexCoord;

const mat3 yuv_to_rgb_matrix = mat3(
            1,   0,       1.402,
            1,  -0.344,  -0.714,
            1,   1.772,   0);

void main()
{
    vec2 vTexCoordSubs = vec2(vTexCoord.x/2., vTexCoord.y/2.);
    float y = texture(yTex, vTexCoord).x;
    float u = texture(uTex, vTexCoordSubs).x;
    float v = texture(vTex, vTexCoordSubs).x;
    vec3 yuv = vec3(y, u - 0.5, v - 0.5);
    fragColor = yuv_to_rgb_matrix * yuv;
}
