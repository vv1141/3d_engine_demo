#ifndef MODEL_H
#define MODEL_H

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <GL/glew.h>
#include <SFML/OpenGL.hpp>
#include <glm/glm.hpp>

#include "Collision.h"
#include "Utility.h"

class Model {
private:
  static int uniqueIdGenerator;
  int        id;

  std::vector<glm::vec3>    vertices;
  std::vector<glm::vec3>    normals;
  std::vector<glm::vec2>    uvs;
  std::vector<glm::vec3>    tangents;
  std::vector<glm::vec3>    bitangents;
  std::vector<unsigned int> indices;

  GLuint vertexBuffer;
  GLuint normalBuffer;
  GLuint uvBuffer;
  GLuint tangentBuffer;
  GLuint bitangentBuffer;
  GLuint indexBuffer;

  float maxWidth;

  static bool                   isNear(float a, float b);
  static bool                   isNear(glm::vec2 a, glm::vec2 b);
  static bool                   isNear(glm::vec3 a, glm::vec3 b);
  static std::vector<glm::vec3> getUniqueNormals(std::vector<glm::vec3> normals);
  static void                   computeTangentBasis(std::vector<glm::vec3>  vertices,
                                                    std::vector<glm::vec2>  uvs,
                                                    std::vector<glm::vec3>  normals,
                                                    std::vector<glm::vec3>& tangents,
                                                    std::vector<glm::vec3>& bitangents);
  static float                  calculateMaxWidth(std::vector<glm::vec3>* vertices);

public:
  Model();
  ~Model();

  void         bufferData();
  void         bindBuffers();
  void         bindVertexBuffer();
  bool         loadObj(std::string directory, std::string fileName, float scale = 1.0f, bool triangulate = false, bool averageVertexNormals = false);
  static bool  loadCollisionMeshFromObj(std::string path, std::vector<Collision::Polyhedron>* polyhedrons, float scale = 1.0f, bool mirrorXZ = false);
  bool         writeModelToFile(std::string path);
  bool         readModelFromFile(std::string path);
  void         copyModel(Model* model, float scale = 1.0f);
  int          getId();
  unsigned int getIndexCount();
  float        getMaxWidth();
  void         printInfo();
};

#endif
