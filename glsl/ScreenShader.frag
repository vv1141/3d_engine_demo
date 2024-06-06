#version 330 core

in VsOut {
  vec2 uv;
}
fsIn;

layout(location = 0) out vec4 colour;

uniform sampler2D screenTextureSampler;

void main() {
  colour = texture(screenTextureSampler, fsIn.uv);

  // gamma correction
  float gamma = 2.2; // industry standard value
  colour.rgb = pow(colour.rgb, vec3(1.0 / gamma));
}
