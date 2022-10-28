#version 440
layout(location = 0) in vec2 texCoord;
layout(location = 0) out vec4 fragColor;
layout(std140, binding = 0) uniform buf {
    mat4 qt_Matrix;
    float qt_Opacity;
};

layout(binding = 1) uniform sampler2D yTex;
layout(binding = 2) uniform sampler2D uTex;
layout(binding = 3) uniform sampler2D vTex;

const mat3 yuv_to_rgb_matrix = mat3(
            1.f,  0.000f,  1.4075f,
            1.f, -0.3455f, -0.7169f,
            1.f,  1.7790f,  0.000f);

void main()
{
    // vec2 coordSubs = vec2(coord.x/2., coord.y/2.);
    float y = texture(yTex, texCoord).r;
    float u = texture(uTex, texCoord).r;
    float v = texture(vTex, texCoord).r;
    vec3 yuv = vec3(y, u - 0.5, v - 0.5);
    fragColor = vec4(yuv*yuv_to_rgb_matrix, 1);
}
