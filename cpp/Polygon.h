#ifndef POLYGON_H
#define POLYGON_H

#include <vector>

#include <GL/glew.h>
#include <glm/glm.hpp>

class Polygon {

private:

  static int uniqueIdGenerator;
  int id;

  std::vector<glm::vec2> vertices;
  std::vector<glm::vec2> uvs;
  std::vector<unsigned int> indices;

  GLuint vertexBuffer;
  GLuint uvBuffer;
  GLuint indexBuffer;

  bool useUvs;
  GLenum usage;

public:

  Polygon();
  ~Polygon();

  void bufferData();
  void bindBuffers();
  void setupScreenQuad(bool useUvs);
  int getId();
  unsigned int getIndexCount();

};

#endif
