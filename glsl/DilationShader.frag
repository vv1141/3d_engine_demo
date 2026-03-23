#version 330 core

in VsOut {
  vec2 uv;
}
fsIn;

layout(location = 0) out vec4 colour;

uniform sampler2D textureSampler;
uniform int       size;
uniform float     separation;

void main() {
  float minThreshold = 0.2;
  float maxThreshold = 0.5;

  vec2 texSize = textureSize(textureSampler, 0).xy;
  colour = texture(textureSampler, fsIn.uv);

  float m = 0.0;
  vec4  p = colour;

  for(int i = -size; i <= size; ++i) {
    for(int j = -size; j <= size; ++j) {
      if(!(i * i + j * j <= size * size)) {
        continue;
      }
      vec4  c = texture(textureSampler, (gl_FragCoord.xy + (vec2(i, j) * separation)) / texSize);
      float t = dot(c.rgb, vec3(0.3, 0.59, 0.11));
      if(t > m) {
        m = t;
        p = c;
      }
    }
  }
  colour.rgb = mix(colour.rgb, p.rgb, smoothstep(minThreshold, maxThreshold, m));
}
