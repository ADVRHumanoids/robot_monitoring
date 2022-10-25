#version 440
layout(location = 0) in vec2 coord;
layout(location = 0) out vec4 fragColor;
layout(std140, binding = 0) uniform buf {
    mat4 qt_Matrix;
    float qt_Opacity;
};
layout(binding = 1) uniform sampler2D yTex;
layout(binding = 2) uniform sampler2D uTex;
layout(binding = 3) uniform sampler2D vTex;

const mat3 yuv_to_rgb_matrix = mat3(
            1,   0,       1.402,
            1,  -0.344,  -0.714,
            1,   1.772,   0);

void main()
{
    // vec2 coordSubs = vec2(coord.x/2., coord.y/2.);
    float y = texture(yTex, coord).r;
    float u = texture(uTex, coord).r;
    float v = texture(vTex, coord).r;
    vec3 yuv = vec3(y, u - 0.5, v - 0.5);
    fragColor = vec4(yuv_to_rgb_matrix*yuv, 1);
}
