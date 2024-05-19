#version 330 core

#define MAX_LIGHTS 4
#define SHADOW_MAP_LEVELS 5

in vec4 fragPosition_lightspace[SHADOW_MAP_LEVELS];
in float fragDepth_viewspace;
in vec2 uv;
in vec3 position_worldspace;
in vec3 vertexToCamera_tangentspace;
in vec3 vertexToLight_tangentspace[MAX_LIGHTS];
in vec4 lightVectors_tangentspace[MAX_LIGHTS];

out vec4 colour;

uniform bool enableReliefMapping;
uniform vec3 colourModifier;
uniform float cascadePlaneEnd[SHADOW_MAP_LEVELS];

uniform sampler2D diffuseMapSampler;
uniform sampler2D normalMapSampler;
uniform sampler2D depthMapSampler;
uniform sampler2D shadowMapSampler[SHADOW_MAP_LEVELS];

uniform int lightCount;
uniform vec4 lightVectors_worldspace[MAX_LIGHTS];
uniform vec3 lightComponents[MAX_LIGHTS];

const float relief_depth = 0.03;
const vec3 relief_normal = vec3(0.0, 0.0, 1.0); // tangent space normal

float ray_intersect_rm(vec2 P, vec2 V){
  const int linear_search_steps = 20;
  const int binary_search_steps = 8;

  float size = 1.0 / linear_search_steps;
  float depth = 0.0;
  float best_depth = 1.0;
  // linear search for the first point under the surface
  for(int i = 0; i < linear_search_steps - 1; i++){
    depth += size;
    float d = texture(depthMapSampler, P+V*depth).r;
    if (depth >= d){
      best_depth = depth;
      break;
    }
  }
  depth = best_depth;
  // binary search for the accurate depth
  for(int i = 0; i < binary_search_steps; i++){
    size *= 0.5;
    float d = texture(depthMapSampler, P+V*depth).r;
    if (depth >= d){
      best_depth = depth;
      depth -= 2*size;
    }
    depth += size;
  }
  return best_depth;
}

float calculateVisibility(){
  int layer = -1;
  for(int i = 0; i < SHADOW_MAP_LEVELS; ++i){
    float depthValue = fragDepth_viewspace;
    if (depthValue < cascadePlaneEnd[i]){
      layer = i;
      break;
    }
  }
  if(layer == -1){
    layer = SHADOW_MAP_LEVELS - 1;
  }
  if(fragPosition_lightspace[layer].z > 1.0) return 1.0;
  // percentage-closer filtering (PCF)
  float shadow = 0.0;
  // loop unrolled for GLSL 3.30 compatibility
  vec2 texelSize;
  float pcfDepth;
  switch(layer){
    case 0:
      texelSize = 1.0 / textureSize(shadowMapSampler[0], 0);
      for(int x = -1; x <= 1; ++x){
        for(int y = -1; y <= 1; ++y){
          pcfDepth = texture(shadowMapSampler[0], fragPosition_lightspace[layer].xy + vec2(x, y) * texelSize).z;
          shadow += fragPosition_lightspace[layer].z > pcfDepth ? 1.0 : 0.0;
        }
      }
      break;
    case 1:
      texelSize = 1.0 / textureSize(shadowMapSampler[1], 0);
      for(int x = -1; x <= 1; ++x){
        for(int y = -1; y <= 1; ++y){
          pcfDepth = texture(shadowMapSampler[1], fragPosition_lightspace[layer].xy + vec2(x, y) * texelSize).z;
          shadow += fragPosition_lightspace[layer].z > pcfDepth ? 1.0 : 0.0;
        }
      }
      break;
    case 2:
      texelSize = 1.0 / textureSize(shadowMapSampler[2], 0);
      for(int x = -1; x <= 1; ++x){
        for(int y = -1; y <= 1; ++y){
          pcfDepth = texture(shadowMapSampler[2], fragPosition_lightspace[layer].xy + vec2(x, y) * texelSize).z;
          shadow += fragPosition_lightspace[layer].z > pcfDepth ? 1.0 : 0.0;
        }
      }
      break;
    case 3:
      texelSize = 1.0 / textureSize(shadowMapSampler[3], 0);
      for(int x = -1; x <= 1; ++x){
        for(int y = -1; y <= 1; ++y){
          pcfDepth = texture(shadowMapSampler[3], fragPosition_lightspace[layer].xy + vec2(x, y) * texelSize).z;
          shadow += fragPosition_lightspace[layer].z > pcfDepth ? 1.0 : 0.0;
        }
      }
      break;
    case 4:
      texelSize = 1.0 / textureSize(shadowMapSampler[4], 0);
      for(int x = -1; x <= 1; ++x){
        for(int y = -1; y <= 1; ++y){
          pcfDepth = texture(shadowMapSampler[4], fragPosition_lightspace[layer].xy + vec2(x, y) * texelSize).z;
          shadow += fragPosition_lightspace[layer].z > pcfDepth ? 1.0 : 0.0;
        }
      }
      break;
  }
  shadow /= 9.0;
  return 1.0 - shadow;
}

vec3 calculateLightColour(vec4 lightVector_worldspace,
                          vec4 lightVector_tangentspace,
                          vec3 lightComponent,
                          vec3 vertexToLight_tangentspace,
                          vec3 normal_tangentspace,
                          vec3 materialDiffuseColor,
                          vec3 materialAmbientColor,
                          vec3 materialSpecularColor,
                          float d1,
                          vec3 P){
  float distance = distance(lightVector_worldspace.xyz, position_worldspace);
  if(lightVector_worldspace.w == 0.0){
    distance = 1.0f;
  }

  vec3 n = normal_tangentspace;
  vec3 l = normalize(vertexToLight_tangentspace);
  float cosTheta = clamp(dot(n, l), 0, 1);

  if(enableReliefMapping){
    vec3 lightDir = normalize(lightVector_tangentspace.xyz);
    if(lightVector_tangentspace.w == 1.0){
      lightDir = normalize(P - lightVector_tangentspace.xyz);
    }
    float size = relief_depth / dot(-relief_normal, lightDir);
    vec3 C = P - d1 * size * lightDir;
    vec3 D = C + lightDir * size;
    vec3 V = D - C;
    float d2 = ray_intersect_rm(C.xy, V.xy);
    if(d2 < d1){
      return materialAmbientColor;
    }
  }

  vec3 e = normalize(vertexToCamera_tangentspace);
  vec3 r = reflect(-l, n);
  float cosAlpha = clamp(dot(e, r), 0, 1);

  vec3 colour = materialAmbientColor + ((materialDiffuseColor * cosTheta) + (materialSpecularColor * pow(cosAlpha, 5))) * lightComponent * calculateVisibility() / (distance * distance);
  return colour;
}

void main(){
  vec2 mappedUv = uv;
  float d1 = 0;
  vec3 P = vec3(0, 0, 0);

  if(enableReliefMapping){
    vec3 A = vec3(uv.x, uv.y, 0.0);
    vec3 viewDir = normalize(-vertexToCamera_tangentspace);
    float size = relief_depth / dot(-relief_normal, viewDir);
    vec3 B = A + viewDir * size;
    vec3 V = B - A;
    d1 = ray_intersect_rm(A.xy, V.xy);
    P = A + d1 * V;
    mappedUv = P.xy;
  }

  vec3 normal_tangentspace = normalize((texture(normalMapSampler, mappedUv).rgb * 2.0) - 1.0);

  vec4 diffuseTexture = texture(diffuseMapSampler, mappedUv).rgba;

  vec3 materialDiffuseColor = diffuseTexture.rgb;
  vec3 materialAmbientColor = vec3(0.25, 0.25, 0.25) * materialDiffuseColor;
  vec3 materialSpecularColor = vec3(0.1, 0.1, 0.1);

  vec3 lightColour = vec3(0.0, 0.0, 0.0);

  for (int i = 0; i < lightCount; i++){
    lightColour += calculateLightColour(
      lightVectors_worldspace[i],
      lightVectors_tangentspace[i],
      lightComponents[i],
      vertexToLight_tangentspace[i],
      normal_tangentspace,
      materialDiffuseColor,
      materialAmbientColor,
      materialSpecularColor,
      d1,
      P
    );
  }
  colour = vec4(lightColour.x, lightColour.y, lightColour.z, diffuseTexture.a);
  colour.rgb += colourModifier;
}
