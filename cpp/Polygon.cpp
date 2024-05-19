
#include "Polygon.h"
#include "Debug.h"

int Polygon::uniqueIdGenerator = 0;

Polygon::Polygon(){
  id = uniqueIdGenerator;
  uniqueIdGenerator++;
}

Polygon::~Polygon(){
  glDeleteBuffers(1, &vertexBuffer);
  if(useUvs) glDeleteBuffers(1, &uvBuffer);
  glDeleteBuffers(1, &indexBuffer);
}

void Polygon::bufferData(){
  glGenBuffers(1, &vertexBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
  glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2) * vertices.size(), &vertices[0], usage);
  if(useUvs){
    glGenBuffers(1, &uvBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, uvBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2) * uvs.size(), &uvs[0], usage);
  }
  glGenBuffers(1, &indexBuffer);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * indices.size(), &indices[0] , usage);
}

void Polygon::bindBuffers(){
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
  glVertexAttribPointer(
    0,
    2,
    GL_FLOAT,
    GL_FALSE,
    0,
    (void*)0
  );
  if(useUvs){
    glBindBuffer(GL_ARRAY_BUFFER, uvBuffer);
    glVertexAttribPointer(
      1,
      2,
      GL_FLOAT,
      GL_FALSE,
      0,
      (void*)0
    );
  }
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
}

void Polygon::setupScreenQuad(bool useUvs){
  this->useUvs = useUvs;
  usage = GL_STATIC_DRAW;
  vertices = std::vector<glm::vec2>{ glm::vec2(-1.0f, 1.0f), glm::vec2(-1.0f, -1.0f), glm::vec2(1.0f, -1.0f), glm::vec2(1.0f, 1.0f) };
  if(useUvs) uvs = std::vector<glm::vec2>{ glm::vec2(0.0f, 1.0f), glm::vec2(0.0f, 0.0f), glm::vec2(1.0f, 0.0f), glm::vec2(1.0f, 1.0f) };
  indices = std::vector<unsigned int>{ 0, 1, 3, 3, 1, 2 };
  bufferData();
}

int Polygon::getId(){
  return id;
}

unsigned int Polygon::getIndexCount(){
  return indices.size();
}
