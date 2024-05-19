#version 330 core

#define MAX_LIGHTS 4
#define SHADOW_MAP_LEVELS 5

layout(location = 0) in vec3 vertexPosition_modelspace;
layout(location = 1) in vec3 vertexNormal_modelspace;
layout(location = 2) in vec2 vertexUv;
layout(location = 3) in vec3 vertexTangent_modelspace;
layout(location = 4) in vec3 vertexBitangent_modelspace;

out vec4 fragPosition_lightspace[SHADOW_MAP_LEVELS];
out float fragDepth_viewspace;
out vec2 uv;
out vec3 position_worldspace;
out vec3 vertexToCamera_tangentspace;
out vec3 vertexToLight_tangentspace[MAX_LIGHTS];
out vec4 lightVectors_tangentspace[MAX_LIGHTS];

uniform mat4 mvp;
uniform mat4 v;
uniform mat4 m;
uniform mat3 mv3x3;
uniform mat4 depthBiasMvp[SHADOW_MAP_LEVELS];

uniform int lightCount;
uniform vec4 lightVectors_cameraspace[MAX_LIGHTS];

void main(){
  gl_Position = mvp * vec4(vertexPosition_modelspace, 1);
  for(int i = 0; i < SHADOW_MAP_LEVELS; i++){
    fragPosition_lightspace[i] = depthBiasMvp[i] * vec4(vertexPosition_modelspace, 1);
  }
  fragDepth_viewspace = gl_Position.z;
  uv = vertexUv;

  position_worldspace = (m * vec4(vertexPosition_modelspace, 1)).xyz;

  vec3 vertexPosition_cameraspace = (v * vec4(position_worldspace, 1)).xyz;
  vec3 vertexToCamera_cameraspace = vec3(0, 0, 0) - vertexPosition_cameraspace;

  // normal mapping
  vec3 vertexNormal_cameraspace = mv3x3 * vertexNormal_modelspace;
  vec3 vertexTangent_cameraspace = mv3x3 * vertexTangent_modelspace;
  vec3 vertexBitangent_cameraspace = mv3x3 * vertexBitangent_modelspace;
  mat3 tbn = transpose(mat3(vertexTangent_cameraspace, vertexBitangent_cameraspace, vertexNormal_cameraspace));
  vertexToCamera_tangentspace = tbn * vertexToCamera_cameraspace;

  vec3 vertexToLight_cameraspace = vec3(0, 0, 0);
  for(int i = 0; i < lightCount; i++){
    if(lightVectors_cameraspace[i].w == 0.0){
      vertexToLight_cameraspace = normalize(-lightVectors_cameraspace[i].xyz); // directional light
    }
    else if(lightVectors_cameraspace[i].w == 1.0){
      vertexToLight_cameraspace = lightVectors_cameraspace[i].xyz + vertexToCamera_cameraspace; // point light
    }
    vertexToLight_tangentspace[i] = tbn * vertexToLight_cameraspace;
    lightVectors_tangentspace[i].xyz = tbn * lightVectors_cameraspace[i].xyz;
    lightVectors_tangentspace[i].w = lightVectors_cameraspace[i].w;
  }
}
