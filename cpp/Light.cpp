#include "Light.h"

float Light::lightVectorsWorldSpace[] = {0};
float Light::lightVectorsCameraSpace[] = {0};
float Light::lightComponents[] = {0};

void Light::setupLights(std::vector<DirectionalLight> directionalLights,
                        std::vector<PointLight>       pointLights,
                        glm::mat4                     view,
                        GLuint                        lightCountId,
                        GLuint                        lightVectorsWorldSpaceId,
                        GLuint                        lightVectorsCameraSpaceId,
                        GLuint                        lightComponentsId) {
  int totalLights = 0;
  for(int i = 0; i < directionalLights.size(); i++) {
    directionalLights[i].direction = glm::normalize(directionalLights[i].direction);
    lightVectorsWorldSpace[totalLights * 4 + 0] = directionalLights[i].direction.x;
    lightVectorsWorldSpace[totalLights * 4 + 1] = directionalLights[i].direction.y;
    lightVectorsWorldSpace[totalLights * 4 + 2] = directionalLights[i].direction.z;
    lightVectorsWorldSpace[totalLights * 4 + 3] = 0.0f;
    glm::vec4 lightDirectionCameraSpace = view * glm::vec4(directionalLights[i].direction, 0.0f);
    lightVectorsCameraSpace[totalLights * 4 + 0] = lightDirectionCameraSpace.x;
    lightVectorsCameraSpace[totalLights * 4 + 1] = lightDirectionCameraSpace.y;
    lightVectorsCameraSpace[totalLights * 4 + 2] = lightDirectionCameraSpace.z;
    lightVectorsCameraSpace[totalLights * 4 + 3] = lightDirectionCameraSpace.w;
    glm::vec3 lightComponent = directionalLights[i].colour * directionalLights[i].power;
    lightComponents[totalLights * 3 + 0] = lightComponent.x;
    lightComponents[totalLights * 3 + 1] = lightComponent.y;
    lightComponents[totalLights * 3 + 2] = lightComponent.z;
    totalLights++;
  }
  for(int i = 0; i < pointLights.size(); i++) {
    lightVectorsWorldSpace[totalLights * 4 + 0] = pointLights[i].position.x;
    lightVectorsWorldSpace[totalLights * 4 + 1] = pointLights[i].position.y;
    lightVectorsWorldSpace[totalLights * 4 + 2] = pointLights[i].position.z;
    lightVectorsWorldSpace[totalLights * 4 + 3] = 1.0f;
    glm::vec4 lightPositionCameraSpace = view * glm::vec4(pointLights[i].position, 1.0f);
    lightVectorsCameraSpace[totalLights * 4 + 0] = lightPositionCameraSpace.x;
    lightVectorsCameraSpace[totalLights * 4 + 1] = lightPositionCameraSpace.y;
    lightVectorsCameraSpace[totalLights * 4 + 2] = lightPositionCameraSpace.z;
    lightVectorsCameraSpace[totalLights * 4 + 3] = lightPositionCameraSpace.w;
    glm::vec3 lightComponent = pointLights[i].colour * pointLights[i].power;
    lightComponents[totalLights * 3 + 0] = lightComponent.x;
    lightComponents[totalLights * 3 + 1] = lightComponent.y;
    lightComponents[totalLights * 3 + 2] = lightComponent.z;
    totalLights++;
  }
  const int lightCount = directionalLights.size() + pointLights.size();
  glUniform1i(lightCountId, lightCount);
  glUniform4fv(lightVectorsWorldSpaceId, lightCount, lightVectorsWorldSpace);
  glUniform4fv(lightVectorsCameraSpaceId, lightCount, lightVectorsCameraSpace);
  glUniform3fv(lightComponentsId, lightCount, lightComponents);
}
