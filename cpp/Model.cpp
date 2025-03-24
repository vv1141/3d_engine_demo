
#include "Model.h"
#include "Debug.h"

int Model::uniqueIdGenerator = 0;

bool Model::isNear(float a, float b) {
  return std::abs(a - b) < 0.001f;
}

bool Model::isNear(glm::vec2 a, glm::vec2 b) {
  return isNear(a.x, b.x) && isNear(a.y, b.y);
}

bool Model::isNear(glm::vec3 a, glm::vec3 b) {
  return isNear(a.x, b.x) && isNear(a.y, b.y) && isNear(a.z, b.z);
}

std::vector<glm::vec3> Model::getUniqueNormals(std::vector<glm::vec3> normals) {
  std::vector<glm::vec3> uniqueNormals;
  for(int i = 0; i < normals.size(); i++) {
    bool isUnique = true;
    for(int j = 0; j < uniqueNormals.size(); j++) {
      if(isNear(normals[i], uniqueNormals[j])) {
        isUnique = false;
        break;
      }
    }
    if(isUnique) {
      uniqueNormals.emplace_back(normals[i]);
    }
  }
  return uniqueNormals;
}

void Model::computeTangentBasis(std::vector<glm::vec3>  vertices,
                                std::vector<glm::vec2>  uvs,
                                std::vector<glm::vec3>  normals,
                                std::vector<glm::vec3>& tangents,
                                std::vector<glm::vec3>& bitangents) {
  for(int i = 0; i < vertices.size(); i += 3) {
    glm::vec3 v0 = vertices[i + 0];
    glm::vec3 v1 = vertices[i + 1];
    glm::vec3 v2 = vertices[i + 2];
    glm::vec2 uv0 = uvs[i + 0];
    glm::vec2 uv1 = uvs[i + 1];
    glm::vec2 uv2 = uvs[i + 2];

    glm::vec3 deltaPos1 = v1 - v0;
    glm::vec3 deltaPos2 = v2 - v0;
    glm::vec2 deltaUv1 = uv1 - uv0;
    glm::vec2 deltaUv2 = uv2 - uv0;

    float     r = 1.0f / (deltaUv1.x * deltaUv2.y - deltaUv1.y * deltaUv2.x);
    glm::vec3 tangent = (deltaPos1 * deltaUv2.y - deltaPos2 * deltaUv1.y) * r;
    glm::vec3 bitangent = (deltaPos2 * deltaUv1.x - deltaPos1 * deltaUv2.x) * r;

    // merge tangents and bitangents in VBO indexing
    tangents.push_back(tangent);
    tangents.push_back(tangent);
    tangents.push_back(tangent);
    bitangents.push_back(bitangent);
    bitangents.push_back(bitangent);
    bitangents.push_back(bitangent);
  }

  for(int i = 0; i < vertices.size(); i++) {
    glm::vec3& n = normals[i];
    glm::vec3& t = tangents[i];
    glm::vec3& b = bitangents[i];

    // Gram-Schmidt orthogonalize
    t = glm::normalize(t - n * glm::dot(n, t));

    // calculate handedness
    if(glm::dot(glm::cross(n, t), b) < 0.0f) {
      t = t * -1.0f;
    }
  }
}

float Model::calculateMaxWidth(std::vector<glm::vec3>* vertices) {
  float maxDistanceToCenter = 0.0f;
  for(int i = 0; i < vertices->size(); i++) {
    float distance = glm::length2((*vertices)[i]);
    if(distance > maxDistanceToCenter) {
      maxDistanceToCenter = distance;
    }
  }
  return 2.0f * sqrt(maxDistanceToCenter);
}

Model::Model() {
  id = uniqueIdGenerator;
  uniqueIdGenerator++;
}

Model::~Model() {
  glDeleteBuffers(1, &vertexBuffer);
  glDeleteBuffers(1, &normalBuffer);
  glDeleteBuffers(1, &uvBuffer);
  glDeleteBuffers(1, &tangentBuffer);
  glDeleteBuffers(1, &bitangentBuffer);
  glDeleteBuffers(1, &indexBuffer);
}

void Model::bufferData() {
  glGenBuffers(1, &vertexBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
  glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
  glGenBuffers(1, &normalBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
  glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * normals.size(), &normals[0], GL_STATIC_DRAW);
  glGenBuffers(1, &uvBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, uvBuffer);
  glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2) * uvs.size(), &uvs[0], GL_STATIC_DRAW);
  glGenBuffers(1, &tangentBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, tangentBuffer);
  glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * tangents.size(), &tangents[0], GL_STATIC_DRAW);
  glGenBuffers(1, &bitangentBuffer);
  glBindBuffer(GL_ARRAY_BUFFER, bitangentBuffer);
  glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * bitangents.size(), &bitangents[0], GL_STATIC_DRAW);
  glGenBuffers(1, &indexBuffer);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * indices.size(), &indices[0], GL_STATIC_DRAW);
}

void Model::bindBuffers() {
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
  glVertexAttribPointer(
    0,
    3,
    GL_FLOAT,
    GL_FALSE,
    0,
    (void*)0);
  glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
  glVertexAttribPointer(
    1,
    3,
    GL_FLOAT,
    GL_FALSE,
    0,
    (void*)0);
  glBindBuffer(GL_ARRAY_BUFFER, uvBuffer);
  glVertexAttribPointer(
    2,
    2,
    GL_FLOAT,
    GL_FALSE,
    0,
    (void*)0);
  glBindBuffer(GL_ARRAY_BUFFER, tangentBuffer);
  glVertexAttribPointer(
    3,
    3,
    GL_FLOAT,
    GL_FALSE,
    0,
    (void*)0);
  glBindBuffer(GL_ARRAY_BUFFER, bitangentBuffer);
  glVertexAttribPointer(
    4,
    3,
    GL_FLOAT,
    GL_FALSE,
    0,
    (void*)0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
}

void Model::bindVertexBuffer() {
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
  glVertexAttribPointer(
    0,
    3,
    GL_FLOAT,
    GL_FALSE,
    0,
    (void*)0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
}

void Model::readCollisionMeshFromMemoryBlock(std::vector<Collision::Polyhedron>* polyhedrons, char** memPointer) {
  unsigned int size;
  Utility::readValue(memPointer, &size);
  for(unsigned int i = 0; i < size; i++) {
    std::vector<glm::vec3> faceVerticeVector;
    Utility::readVector(memPointer, &faceVerticeVector);
    glm::vec3 faceNormal = Utility::getTriangleNormal(&faceVerticeVector);
    polyhedrons->emplace_back();
    Collision::constructPlane(&polyhedrons->back(), &faceVerticeVector, &faceNormal);
  }
}

void Model::readModelFromMemoryBlock(char** memPointer) {
  char* start = *memPointer;
  Utility::readVector(memPointer, &vertices);
  Utility::readVector(memPointer, &normals);
  Utility::readVector(memPointer, &uvs);
  Utility::readVector(memPointer, &tangents);
  Utility::readVector(memPointer, &bitangents);
  Utility::readVector(memPointer, &indices);
}

void Model::copyModel(Model* model, float scale) {
  vertices.clear();
  vertices.reserve(model->vertices.size());
  for(int i = 0; i < model->vertices.size(); i++) {
    vertices.emplace_back(model->vertices[i].x * scale, model->vertices[i].y * scale, model->vertices[i].z * scale);
  }
  normals = model->normals;
  uvs = model->uvs;
  tangents = model->tangents;
  bitangents = model->bitangents;
  indices = model->indices;
}

int Model::getId() {
  return id;
}

unsigned int Model::getIndexCount() {
  return indices.size();
}

float Model::getMaxWidth() {
  return maxWidth;
}

void Model::printInfo() {
  std::cout << "normals" << std::endl;
  for(int i = 0; i < normals.size(); i++) {
    glm::vec3 n = normals[i];
    std::cout << std::to_string(n.x) << std::endl;
    std::cout << std::to_string(n.y) << std::endl;
    std::cout << std::to_string(n.z) << std::endl;
    std::cout << "---" << std::endl;
  }
}
