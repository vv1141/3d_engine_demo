#version 330 core

in VsOut {
  vec2 uv;
}
fsIn;

layout(location = 0) out vec4 colour;

uniform sampler2D textureSampler;

void main() {
  int  size = 2;
  vec2 texSize = textureSize(textureSampler, 0).xy;
  colour = texture(textureSampler, fsIn.uv);
  colour.rgb = vec3(0);

  float count = 0.0;
  for(int i = -size; i <= size; ++i) {
    for(int j = -size; j <= size; ++j) {
      colour.rgb += texture(textureSampler, (gl_FragCoord.xy + vec2(i, j)) / texSize).rgb;
      count += 1.0;
    }
  }
  colour.rgb /= count;
}
