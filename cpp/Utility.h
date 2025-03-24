#ifndef UTILITY_H
#define UTILITY_H

#include <GL/glew.h>
#include <SFML/OpenGL.hpp>
#include <SFML/Graphics.hpp>
#include <glm/glm.hpp>

#include <cstring>
#include <fstream>
#include <random>
#include <sstream>
#include <string>

#include <math.h>

class Debug;

class Utility {
public:
  class Debouncer {
  public:
    int  ticks;
    void set(int ticks) {
      this->ticks = ticks;
    }
    bool get() {
      return ticks > 0;
    }
    void update() {
      if(ticks > 0) ticks--;
    }
  };

  static const float tau;
  static const float epsilon;

  static sf::RenderWindow* renderWindow;
  static glm::mat4*        projectionViewMatrix;
  static glm::vec2         screenSize;

  static char*        fontData;
  static sf::Font     font;
  static sf::Text     text;
  static std::mt19937 generator;

  static bool      onDistance(glm::vec3 a, glm::vec3 b, float distance);
  static double    distanceSquared(glm::vec3 a, glm::vec3 b);
  static glm::vec3 getOneOrthogonalVector(glm::vec3 vector);
  static glm::vec3 getOneOrthogonalUnitVector(glm::vec3 vector);
  static bool      triangleHasArea(glm::vec3* triangleVertices, glm::vec3 triangleNormal);
  static bool      pointInsideTriangle(glm::vec3 point, glm::vec3* triangleVertices, glm::vec3 triangleNormal);
  static glm::vec3 getTriangleNormal(std::vector<glm::vec3>* vertices);
  static bool      verticesFormPlane(std::vector<glm::vec3>* vertices, glm::vec3* normal);
  static float     getFraction(float value, float lowerThreshold, float upperThreshold);
  static glm::vec3 vectorRejection(glm::vec3 vector, glm::vec3 normal);
  static bool      isNan(glm::vec3 vec);
  static void      initRenderState(sf::RenderWindow* renderWindow, glm::mat4* projectionViewMatrix);
  static void      renderLine(glm::vec2 startPosition, glm::vec2 endPosition, sf::Color colour);
  static void      renderMarker(glm::vec2 position, sf::Color colour);
  static void      renderCircle(glm::vec2 position, float size, sf::Color colour);
  static void      renderRectangle(glm::vec2 position, glm::vec2 size, sf::Color colour);
  static void      renderRectangle(glm::vec2 position, glm::vec2 size, sf::Color colour, float angle);
  static void      renderRectangle(glm::vec2 position, glm::vec2 size, sf::Color colour, sf::Texture);
  static void      initTextRendering();
  static void      renderText(glm::vec2 position, std::string string);
  static void      renderText(glm::vec2 position, std::string string, sf::Color colour);
  static void      renderTitleText(glm::vec2 position, std::string string);
  static void      renderMenuText(glm::vec2 position, std::string string, sf::Color colour);
  static void      initPRNG();
  static int       randomInt(int min, int max);
  static double    randomDouble(double min, double max);
  static glm::vec2 getScreenCoordinates(glm::vec3 position);
  static bool      writeFile(std::string path, char* source, unsigned int sizeInBytes);
  static bool      readFile(std::string path, char** destination, unsigned int* sizeInBytes);

  static bool checkFile(std::string path);
  static bool readFontFromMemoryBlock(char** memPointer);

  template<class T> static unsigned int getVectorDataSize(std::vector<T>* vector) {
    return 4 + vector->size() * sizeof(T);
  }

  template<class T> static void writeValue(char** destination, T value) {
    size_t dataSize = sizeof(T);
    memcpy(*destination, &value, dataSize);
    *destination += dataSize;
  }

  template<class T> static void writeVector(char** destination, std::vector<T>* vector) {
    unsigned int size = (unsigned int)vector->size();
    memcpy(*destination, &size, 4);
    *destination += 4;
    size_t vectorDataSize = size * sizeof(T);
    memcpy(*destination, vector->data(), vectorDataSize);
    *destination += vectorDataSize;
  }

  template<class T> static void readValue(char** source, T* value) {
    size_t dataSize = sizeof(T);
    memcpy(value, *source, dataSize);
    *source += dataSize;
  }

  template<class T> static void readVector(char** source, std::vector<T>* vector) {
    unsigned int size;
    memcpy(&size, *source, 4);
    vector->resize(size);
    *source += 4;
    size_t vectorDataSize = size * sizeof(T);
    memcpy(vector->data(), *source, vectorDataSize);
    *source += vectorDataSize;
  }
};

#endif
