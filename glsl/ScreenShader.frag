#version 330 core

in VsOut {
  vec2 uv;
}
fsIn;

layout(location = 0) out vec4 colour;

uniform sampler2D screenTextureSampler;
uniform bool      depthOfFieldEnabled;
uniform sampler2D outOfFocusTextureSampler;
uniform sampler2D positionTextureSampler;
uniform vec3      minMaxFocus;
uniform vec2      nearFar;

float linearDepth(float depth) {
  float near = nearFar.x;
  float far = nearFar.y;
  float z = depth * 2.0 - 1.0;
  return (2.0 * near * far) / (far + near - z * (far - near));
}

vec3 calculateDepthOfFieldColour() {
  float minDistance = minMaxFocus.x;
  float maxDistance = minMaxFocus.y;
  float focusDepth = minMaxFocus.z;

  vec2 texSize = textureSize(screenTextureSampler, 0).xy;
  vec2 texCoord = gl_FragCoord.xy / texSize;

  vec3 focusColour = texture(screenTextureSampler, texCoord).rgb;
  vec3 outOfFocusColour = texture(outOfFocusTextureSampler, texCoord).rgb;

  float position = linearDepth(texture(positionTextureSampler, texCoord).r);
  float blur = smoothstep(minDistance, maxDistance, abs(position - focusDepth));
  return mix(focusColour, outOfFocusColour, blur);
}

void main() {
  colour = vec4(0.0);
  if(depthOfFieldEnabled) {
    colour.rgb = calculateDepthOfFieldColour();
  } else {
    colour.rgb = texture(screenTextureSampler, fsIn.uv).rgb;
  }

  // gamma correction
  float gamma = 2.2; // industry standard value
  colour.rgb = pow(colour.rgb, vec3(1.0 / gamma));
}
