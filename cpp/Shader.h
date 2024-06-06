#ifndef SHADER_H
#define SHADER_H

#include <GL/glew.h>
#include <glm/glm.hpp>

#include "Debug.h"
#include "Light.h"
#include "Polygon.h"
#include "RenderObject.h"

class Shader {
private:
public:
  GLuint programId;

  Shader();
  virtual ~Shader();

  GLuint loadShaders(std::string vertexShader, std::string fragmentShader, std::string geometryShader);
};

class OpaqueShader: public Shader {
private:
public:
  GLuint              enableReliefMapping;
  GLuint              enablePreMultiplyAlpha;
  GLuint              colourModifier;
  std::vector<GLuint> cascadePlaneEndIds;

  GLuint              mvpMatrixId;
  GLuint              modelMatrixId;
  GLuint              viewMatrixId;
  GLuint              mv3x3MatrixId;
  std::vector<GLuint> depthBiasMvpMatrixIds;

  GLuint lightCountId;
  GLuint lightVectorsWorldSpaceId;
  GLuint lightVectorsCameraSpaceId;
  GLuint lightComponentsId;

  GLuint              diffuseMapSampler;
  GLuint              normalMapSampler;
  GLuint              depthMapSampler;
  std::vector<GLuint> shadowMapSamplers;

  OpaqueShader();
  ~OpaqueShader();

  bool setup(int shadowLevelCount);
  void render(const sf::RenderWindow*                     renderWindow,
              GLuint                                      framebuffer,
              const std::vector<Light::DirectionalLight>* directionalLights,
              const std::vector<Light::PointLight>*       pointLights,
              glm::mat4                                   view,
              std::vector<GLuint>                         depthTextures,
              const std::vector<RenderObject>*            renderObjects,
              double                                      alpha,
              glm::mat4                                   projectionViewMatrix,
              std::vector<glm::mat4>                      depthViewProjections,
              std::vector<glm::vec2>                      clippingPlanes);
};

class ShadowMappingShader: public Shader {
private:
  bool                   generateBuffers(int depthMapResolution, int shadowLevelCount);
  std::vector<glm::vec4> getFrustumCornersWorldSpace(glm::mat4 projectionViewInverse);

public:
  int depthMapResolution;
  int shadowLevelCount;

  GLuint mvpMatrixId;

  GLuint              framebuffer;
  std::vector<GLuint> depthTextures;

  ShadowMappingShader();
  ~ShadowMappingShader();

  bool                   setup(int depthMapResolution, int shadowLevelCount);
  std::vector<glm::mat4> calculateProjectionMatrices(const sf::RenderWindow* renderWindow, float fov, std::vector<glm::vec2> planes);
  std::vector<glm::vec2> calculateClippingPlanes(float nearClippingPlane, float farClippingPlane, int shadowLevelCount);
  glm::mat4              calculateDepthViewProjection(glm::vec3 lightDirection, glm::mat4 projectionViewInverse, float zMult, float zBias);
  void                   setupLightSpaceMatrices(const std::vector<glm::mat4>* depthViewProjections);
  void                   bindForWriting(GLuint texture);
  void                   render(const std::vector<RenderObject>* renderObjects,
                                double                           alpha,
                                std::vector<glm::mat4>           depthViewProjections);
};

class ScreenShader: public Shader {
private:
  bool generateBuffers(glm::ivec2 windowSize, bool multisamplingEnabled, int multisamplingSampleCount);

public:
  GLuint screenTextureSampler;

  Polygon screenQuad;
  GLuint  framebuffer;
  GLuint  texture;
  GLuint  renderbuffer;
  GLuint  intermediateFramebuffer;
  GLuint  intermediateTexture;

  ScreenShader();
  ~ScreenShader();

  bool setup(glm::ivec2 windowSize, bool multisamplingEnabled, int multisamplingSampleCount);
  void bind(GLuint t);
  void render(const sf::RenderWindow* renderWindow, bool multisamplingEnabled);
  void render(GLuint t, int x, int y, int xOffset, int yOffset);
  void draw();
};

#endif
