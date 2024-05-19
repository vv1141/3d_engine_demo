#ifndef LIGHT_H
#define LIGHT_H

#define MAX_LIGHTS 4

#include <vector>

#include <GL/glew.h>
#include <SFML/OpenGL.hpp>
#include <glm/glm.hpp>

class Light {
private:

  static float lightVectorsWorldSpace[4 * MAX_LIGHTS];
  static float lightVectorsCameraSpace[4 * MAX_LIGHTS];
  static float lightComponents[3 * MAX_LIGHTS];

public:

  struct PointLight {
    glm::vec3 position;
    glm::vec3 colour;
    float power;
  };

  struct DirectionalLight {
    glm::vec3 direction;
    glm::vec3 colour;
    float power;
  };

  struct SpotLight {
    glm::vec3 position;
    glm::vec3 direction;
    float cutOff;
    float cutOffRegionPercentage; // 0.0-1.0
    glm::vec3 colour;
    float power;
  };

  static void setupLights(std::vector<DirectionalLight> directionalLights,
                          std::vector<PointLight> pointLights,
                          glm::mat4 view,
                          GLuint lightCountId,
                          GLuint lightVectorsWorldSpaceId,
                          GLuint lightVectorsCameraSpaceId,
                          GLuint lightComponentsId);

};

#endif
