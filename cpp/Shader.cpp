
#include "Shader.h"

Shader::Shader() {
}
Shader::~Shader() {
  glDeleteProgram(programId);
}

std::string Shader::loadShader(std::string path) {
  std::string   code;
  std::ifstream shaderStream(path, std::ios::in);
  if(shaderStream.is_open()) {
    std::stringstream sstr;
    sstr << shaderStream.rdbuf();
    code = sstr.str();
    shaderStream.close();
    return code;
  } else {
    Debug::log("Error: Failed reading ", path);
    return "";
  }
  return "";
}

std::string Shader::loadShader(char** memPointer) {
  unsigned int size;
  Utility::readValue(memPointer, &size);
  std::string code(*memPointer, size);
  *memPointer += size;
  return code;
}

bool Shader::loadShaders(std::string vertexShader, std::string fragmentShader) {
  std::string basePath = "glsl/";
  vertexShaderCode = loadShader(basePath + vertexShader);
  if(vertexShaderCode == "") {
    return false;
  }
  fragmentShaderCode = loadShader(basePath + fragmentShader);
  if(fragmentShaderCode == "") {
    return false;
  }
  return true;
}

void Shader::loadShaders(char** memPointer) {
  vertexShaderCode = loadShader(memPointer);
  fragmentShaderCode = loadShader(memPointer);
}

void Shader::compileShader(GLuint id, std::string code) {
  char const* shader = code.c_str();
  glShaderSource(id, 1, &(shader), NULL);
  GLint result = GL_FALSE;
  int   infoLogLength;
  glCompileShader(id);
  glGetShaderiv(id, GL_COMPILE_STATUS, &result);
  glGetShaderiv(id, GL_INFO_LOG_LENGTH, &infoLogLength);
  if(infoLogLength > 0) {
    std::vector<char> errorMessage(infoLogLength + 1);
    glGetShaderInfoLog(id, infoLogLength, NULL, &errorMessage[0]);
    printf("%s\n", &errorMessage[0]);
  }
}

GLuint Shader::createShaders() {
  GLuint vertexShaderId = glCreateShader(GL_VERTEX_SHADER);
  GLuint fragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);

  GLint result = GL_FALSE;
  int   infoLogLength;

  Debug::log("Compiling vertex shader");
  compileShader(vertexShaderId, vertexShaderCode);
  Debug::log("Compiling fragment shader");
  compileShader(fragmentShaderId, fragmentShaderCode);

  GLuint programId = glCreateProgram();

  glAttachShader(programId, vertexShaderId);
  glAttachShader(programId, fragmentShaderId);
  glLinkProgram(programId);

  glGetProgramiv(programId, GL_LINK_STATUS, &result);
  glGetProgramiv(programId, GL_INFO_LOG_LENGTH, &infoLogLength);
  if(infoLogLength > 0) {
    std::vector<char> programErrorMessage(infoLogLength + 1);
    glGetProgramInfoLog(programId, infoLogLength, NULL, &programErrorMessage[0]);
    printf("%s\n", &programErrorMessage[0]);
  }

  glDetachShader(programId, vertexShaderId);
  glDeleteShader(vertexShaderId);
  glDetachShader(programId, fragmentShaderId);
  glDeleteShader(fragmentShaderId);

  return programId;
}

OpaqueShader::OpaqueShader() {
}
OpaqueShader::~OpaqueShader() {
}

bool OpaqueShader::setup(int shadowLevelCount) {
  programId = createShaders();
  if(!programId) return false;
  glUseProgram(programId);
  enableReliefMapping = glGetUniformLocation(programId, "enableReliefMapping");
  colourModifier = glGetUniformLocation(programId, "colourModifier");
  for(int i = 0; i < shadowLevelCount; i++) {
    std::string uniformName = "cascadePlaneEnd[" + std::to_string(i) + "]";
    cascadePlaneEndIds.push_back(glGetUniformLocation(programId, uniformName.c_str()));
  }
  mvpMatrixId = glGetUniformLocation(programId, "mvp");
  modelMatrixId = glGetUniformLocation(programId, "m");
  viewMatrixId = glGetUniformLocation(programId, "v");
  mv3x3MatrixId = glGetUniformLocation(programId, "mv3x3");
  for(int i = 0; i < shadowLevelCount; i++) {
    std::string uniformName = "depthBiasMvp[" + std::to_string(i) + "]";
    depthBiasMvpMatrixIds.push_back(glGetUniformLocation(programId, uniformName.c_str()));
  }
  lightCountId = glGetUniformLocation(programId, "lightCount");
  lightVectorsWorldSpaceId = glGetUniformLocation(programId, "lightVectors_worldspace");
  lightVectorsCameraSpaceId = glGetUniformLocation(programId, "lightVectors_cameraspace");
  lightComponentsId = glGetUniformLocation(programId, "lightComponents");
  diffuseMapSampler = glGetUniformLocation(programId, "diffuseMapSampler");
  normalMapSampler = glGetUniformLocation(programId, "normalMapSampler");
  depthMapSampler = glGetUniformLocation(programId, "depthMapSampler");
  for(int i = 0; i < shadowLevelCount; i++) {
    std::string uniformName = "shadowMapSampler[" + std::to_string(i) + "]";
    shadowMapSamplers.push_back(glGetUniformLocation(programId, uniformName.c_str()));
  }
  return true;
}

void OpaqueShader::render(const sf::RenderWindow*                     renderWindow,
                          GLuint                                      framebuffer,
                          const std::vector<Light::DirectionalLight>* directionalLights,
                          const std::vector<Light::PointLight>*       pointLights,
                          glm::mat4                                   view,
                          std::vector<GLuint>                         depthTextures,
                          const std::vector<RenderObject>*            renderObjects,
                          double                                      alpha,
                          glm::mat4                                   projectionViewMatrix,
                          std::vector<glm::mat4>                      depthViewProjections,
                          std::vector<glm::vec2>                      clippingPlanes) {
  glUseProgram(programId);
  glViewport(0, 0, renderWindow->getSize().x, renderWindow->getSize().y);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  glDepthFunc(GL_LESS);
  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
  glClearColor(0.8f, 0.6f, 0.3f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  Light::setupLights(*directionalLights,
                     *pointLights,
                     view,
                     lightCountId,
                     lightVectorsWorldSpaceId,
                     lightVectorsCameraSpaceId,
                     lightComponentsId);

  for(int i = 0; i < depthViewProjections.size(); i++) {
    glActiveTexture(GL_TEXTURE3 + i);
    glBindTexture(GL_TEXTURE_2D, depthTextures[i]);
    glUniform1i(shadowMapSamplers[i], 3 + i);
    glUniform1f(cascadePlaneEndIds[i], clippingPlanes[i].y);
  }

  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);
  glEnableVertexAttribArray(2);
  glEnableVertexAttribArray(3);
  glEnableVertexAttribArray(4);
  int activeModelId = -1;
  int activeTextureId = -1;
  for(auto it = renderObjects->begin(); it != renderObjects->end(); ++it) {
    if(it->modelTexturePair->model->getId() != activeModelId) {
      it->modelTexturePair->model->bindBuffers();
      activeModelId = it->modelTexturePair->model->getId();
    }
    if(it->modelTexturePair->texture->getId() != activeTextureId) {
      it->modelTexturePair->texture->bindTextures(diffuseMapSampler,
                                                  normalMapSampler,
                                                  depthMapSampler,
                                                  Texture::Flags::diffuse | Texture::Flags::normal | Texture::Flags::depth);
      activeTextureId = it->modelTexturePair->texture->getId();
    }
    glUniform1i(enableReliefMapping, false);
    glUniform3f(colourModifier, 0.0f, 0.0f, 0.0f);
    glm::mat4 model = it->object->getInterpolatedModelMatrix(alpha);
    glm::mat4 mvp = projectionViewMatrix * model;
    glm::mat3 mv3x3 = glm::mat3(view * model);

    for(int i = 0; i < depthBiasMvpMatrixIds.size(); i++) {
      glm::mat4 depthBiasMvp = depthViewProjections[i] * model;
      glUniformMatrix4fv(depthBiasMvpMatrixIds[i], 1, GL_FALSE, &depthBiasMvp[0][0]);
    }

    glUniformMatrix4fv(mvpMatrixId, 1, GL_FALSE, &mvp[0][0]);
    glUniformMatrix4fv(modelMatrixId, 1, GL_FALSE, &model[0][0]);
    glUniformMatrix4fv(viewMatrixId, 1, GL_FALSE, &view[0][0]);
    glUniformMatrix3fv(mv3x3MatrixId, 1, GL_FALSE, &mv3x3[0][0]);
    glDrawElements(GL_TRIANGLES, it->modelTexturePair->model->getIndexCount(), GL_UNSIGNED_INT, (void*)0);
  }
  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
  glDisableVertexAttribArray(2);
  glDisableVertexAttribArray(3);
  glDisableVertexAttribArray(4);
}

bool ShadowMappingShader::generateBuffers(int depthMapResolution, int shadowLevelCount) {
  glGenFramebuffers(1, &framebuffer);
  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
  for(int i = 0; i < shadowLevelCount; i++) {
    depthTextures.push_back(-1);
    GLuint dt = depthTextures.back();
    glGenTextures(1, &dt);
    depthTextures.back() = dt;
    glBindTexture(GL_TEXTURE_2D, dt);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, depthMapResolution, depthMapResolution, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    const float borderColor[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, dt, 0);
  }
  glDrawBuffer(GL_NONE);
  glReadBuffer(GL_NONE);
  if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    Debug::log("Error: ShadowMappingShader: Framebuffer is not complete");
    return false;
  }
  return true;
}

std::vector<glm::vec4> ShadowMappingShader::getFrustumCornersWorldSpace(glm::mat4 projectionViewInverse) {
  std::vector<glm::vec4> frustumCorners;
  for(int x = 0; x < 2; x++) {
    for(int y = 0; y < 2; y++) {
      for(int z = 0; z < 2; z++) {
        const glm::vec4 pt = projectionViewInverse * glm::vec4(2.0f * x - 1.0f,
                                                               2.0f * y - 1.0f,
                                                               2.0f * z - 1.0f,
                                                               1.0f);
        frustumCorners.push_back(pt / pt.w);
      }
    }
  }
  return frustumCorners;
}

ShadowMappingShader::ShadowMappingShader() {
}
ShadowMappingShader::~ShadowMappingShader() {
  glDeleteTextures(1, &framebuffer);
  for(int i = 0; i < shadowLevelCount; i++) {
    glDeleteTextures(1, &(depthTextures[i]));
  }
}

bool ShadowMappingShader::setup(int depthMapResolution, int shadowLevelCount) {
  this->depthMapResolution = depthMapResolution;
  this->shadowLevelCount = shadowLevelCount;
  programId = createShaders();
  if(!programId) return false;
  glUseProgram(programId);
  mvpMatrixId = glGetUniformLocation(programId, "depthMvp");
  return generateBuffers(depthMapResolution, shadowLevelCount);
}

std::vector<glm::vec2> ShadowMappingShader::calculateClippingPlanes(float nearClippingPlane, float farClippingPlane, int shadowLevelCount) {
  std::vector<glm::vec2> planes;
  planes.push_back(glm::vec2(nearClippingPlane, 2.0f));
  planes.push_back(glm::vec2(2.0f, 7.0f));
  planes.push_back(glm::vec2(7.0f, 12.0f));
  planes.push_back(glm::vec2(12.0f, 25.0f));
  planes.push_back(glm::vec2(25.0f, farClippingPlane));
  return planes;
}

std::vector<glm::mat4> ShadowMappingShader::calculateProjectionMatrices(const sf::RenderWindow* renderWindow, float fov, std::vector<glm::vec2> planes) {
  std::vector<glm::mat4> matrices;
  for(int i = 0; i < planes.size(); i++) {
    const float near2 = planes[i].x;
    const float far2 = planes[i].y;
    glm::mat4   projection = glm::perspective(
      glm::radians(fov),
      (float)renderWindow->getSize().x / (float)renderWindow->getSize().y,
      near2,
      far2);
    matrices.push_back(projection);
  }
  return matrices;
}

glm::mat4 ShadowMappingShader::calculateDepthViewProjection(glm::vec3 lightDirection, glm::mat4 projectionViewInverse, float zMult, float zBias) {
  std::vector<glm::vec4> corners = getFrustumCornersWorldSpace(projectionViewInverse);

  glm::vec3 center = glm::vec3(0, 0, 0);
  for(const auto& v: corners) {
    center += glm::vec3(v);
  }
  center /= corners.size();

  const auto lightView = glm::lookAt(
    center - lightDirection,
    center,
    glm::vec3(0.0f, 1.0f, 0.0f));

  float minX = std::numeric_limits<float>::max();
  float maxX = std::numeric_limits<float>::min();
  float minY = std::numeric_limits<float>::max();
  float maxY = std::numeric_limits<float>::min();
  float minZ = std::numeric_limits<float>::max();
  float maxZ = std::numeric_limits<float>::min();
  for(const auto& v: corners) {
    const auto trf = lightView * v;
    minX = std::min(minX, trf.x);
    maxX = std::max(maxX, trf.x);
    minY = std::min(minY, trf.y);
    maxY = std::max(maxY, trf.y);
    minZ = std::min(minZ, trf.z);
    maxZ = std::max(maxZ, trf.z);
  }

  if(minZ < 0) {
    minZ *= zMult;
  } else {
    minZ /= zMult;
  }
  if(maxZ < 0) {
    maxZ /= zMult;
  } else {
    maxZ *= zMult;
  }

  const glm::mat4 lightProjection = glm::ortho(minX, maxX, minY, maxY, maxZ * -1, minZ * -1 + zBias);
  return lightProjection * lightView;
}

void ShadowMappingShader::bindForWriting(GLuint texture) {
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, framebuffer);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texture, 0);
}

void ShadowMappingShader::render(const std::vector<RenderObject>* renderObjects,
                                 double                           alpha,
                                 std::vector<glm::mat4>           depthViewProjections) {
  glUseProgram(programId);
  glViewport(0, 0, depthMapResolution, depthMapResolution);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_FRONT);
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  glDepthFunc(GL_LESS);

  glEnableVertexAttribArray(0);
  int activeModelId = -1;
  for(int i = 0; i < depthViewProjections.size(); i++) {
    bindForWriting(depthTextures[i]);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    for(auto it = renderObjects->begin(); it != renderObjects->end(); ++it) {
      if(it->modelTexturePair->model->getId() != activeModelId) {
        it->modelTexturePair->model->bindVertexBuffer();
        activeModelId = it->modelTexturePair->model->getId();
      }
      glm::mat4 model = it->object->getInterpolatedModelMatrix(alpha);
      glm::mat4 depthMvp = depthViewProjections[i] * model;
      glUniformMatrix4fv(mvpMatrixId, 1, GL_FALSE, &depthMvp[0][0]);
      glDrawElements(GL_TRIANGLES, it->modelTexturePair->model->getIndexCount(), GL_UNSIGNED_INT, (void*)0);
    }
  }
  glDisableVertexAttribArray(0);
}

bool ScreenShader::generateBuffers(glm::ivec2 windowSize, bool multisamplingEnabled, int multisamplingSampleCount) {
  if(multisamplingEnabled) {
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, texture);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, multisamplingSampleCount, GL_RGBA, windowSize.x, windowSize.y, GL_TRUE);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, texture, 0);
    GLenum DrawBuffers[] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, DrawBuffers);
    glGenRenderbuffers(1, &renderbuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, renderbuffer);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, multisamplingSampleCount, GL_DEPTH24_STENCIL8, windowSize.x, windowSize.y);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, renderbuffer);
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      Debug::log("Error: ScreenShader: MSAA framebuffer is not complete");
      return false;
    }
    glGenFramebuffers(1, &intermediateFramebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, intermediateFramebuffer);
    glGenTextures(1, &intermediateTexture);
    glBindTexture(GL_TEXTURE_2D, intermediateTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, windowSize.x, windowSize.y, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, intermediateTexture, 0);
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      Debug::log("Error: ScreenShader: Post-processing framebuffer is not complete");
      return false;
    }
  } else {
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, windowSize.x, windowSize.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);
    GLenum DrawBuffers[] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, DrawBuffers);
    glGenRenderbuffers(1, &renderbuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, renderbuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, windowSize.x, windowSize.y);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, renderbuffer);
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      Debug::log("Error: ScreenShader: Framebuffer is not complete");
      return false;
    }
  }
  return true;
}

ScreenShader::ScreenShader() {
}
ScreenShader::~ScreenShader() {
  glDeleteTextures(1, &texture);
  glDeleteRenderbuffers(1, &renderbuffer);
  glDeleteFramebuffers(1, &framebuffer);
}

bool ScreenShader::setup(glm::ivec2 windowSize, bool multisamplingEnabled, int multisamplingSampleCount) {
  programId = createShaders();
  if(!programId) return false;
  glUseProgram(programId);
  screenTextureSampler = glGetUniformLocation(programId, "screenTextureSampler");
  screenQuad.setupScreenQuad(true);
  return generateBuffers(windowSize, multisamplingEnabled, multisamplingSampleCount);
}

void ScreenShader::bind(GLuint t) {
  screenQuad.bindBuffers();
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, t);
  glUniform1i(screenTextureSampler, 0);
}

void ScreenShader::render(const sf::RenderWindow* renderWindow, bool multisamplingEnabled) {
  glUseProgram(programId);
  glEnable(GL_MULTISAMPLE);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
  if(multisamplingEnabled) {
    glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, intermediateFramebuffer);
    glBlitFramebuffer(0, 0, renderWindow->getSize().x, renderWindow->getSize().y, 0, 0, renderWindow->getSize().x, renderWindow->getSize().y, GL_COLOR_BUFFER_BIT, GL_NEAREST);
  }
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);
  glViewport(0, 0, renderWindow->getSize().x, renderWindow->getSize().y);
  if(multisamplingEnabled) {
    bind(this->intermediateTexture);
  } else {
    bind(this->texture);
  }
  draw();
  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
}

void ScreenShader::render(GLuint t, int x, int y, int xOffset, int yOffset) {
  glUseProgram(programId);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);
  glViewport(x, y, xOffset, yOffset);
  bind(t);
  draw();
  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
}

void ScreenShader::draw() {
  glDrawElements(GL_TRIANGLES, screenQuad.getIndexCount(), GL_UNSIGNED_INT, (void*)0);
}
