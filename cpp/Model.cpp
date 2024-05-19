#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include "Model.h"
#include "Debug.h"

int Model::uniqueIdGenerator = 0;

bool Model::isNear(float a, float b){
  return std::abs(a - b) < 0.001f;
}

bool Model::isNear(glm::vec2 a, glm::vec2 b){
  return isNear(a.x, b.x) && isNear(a.y, b.y);
}

bool Model::isNear(glm::vec3 a, glm::vec3 b){
  return isNear(a.x, b.x) && isNear(a.y, b.y) && isNear(a.z, b.z);
}

std::vector<glm::vec3> Model::getUniqueNormals(std::vector<glm::vec3> normals){
  std::vector<glm::vec3> uniqueNormals;
  for(int i = 0; i < normals.size(); i++){
    bool isUnique = true;
    for(int j = 0; j < uniqueNormals.size(); j++){
      if(isNear(normals[i], uniqueNormals[j])){
        isUnique = false;
        break;
      }
    }
    if(isUnique){
      uniqueNormals.emplace_back(normals[i]);
    }
  }
  return uniqueNormals;
}

void Model::computeTangentBasis(std::vector<glm::vec3> vertices,
                                std::vector<glm::vec2> uvs,
                                std::vector<glm::vec3> normals,
                                std::vector<glm::vec3>& tangents,
                                std::vector<glm::vec3>& bitangents){

  for(int i = 0; i < vertices.size(); i += 3){
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

    float r = 1.0f / (deltaUv1.x * deltaUv2.y - deltaUv1.y * deltaUv2.x);
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

  for(int i = 0; i < vertices.size(); i++){
    glm::vec3& n = normals[i];
    glm::vec3& t = tangents[i];
    glm::vec3& b = bitangents[i];

    // Gram-Schmidt orthogonalize
    t = glm::normalize(t - n * glm::dot(n, t));

    // calculate handedness
    if (glm::dot(glm::cross(n, t), b) < 0.0f){
      t = t * -1.0f;
    }
  }
}

float Model::calculateMaxWidth(std::vector<glm::vec3>* vertices){
  float maxDistanceToCenter = 0.0f;
  for(int i = 0; i < vertices->size(); i++){
    float distance = glm::length2((*vertices)[i]);
    if(distance > maxDistanceToCenter){
      maxDistanceToCenter = distance;
    }
  }
  return 2.0f * sqrt(maxDistanceToCenter);
}

Model::Model(){
  id = uniqueIdGenerator;
  uniqueIdGenerator++;
}

Model::~Model(){
  glDeleteBuffers(1, &vertexBuffer);
  glDeleteBuffers(1, &normalBuffer);
  glDeleteBuffers(1, &uvBuffer);
  glDeleteBuffers(1, &tangentBuffer);
  glDeleteBuffers(1, &bitangentBuffer);
  glDeleteBuffers(1, &indexBuffer);
}

void Model::bufferData(){
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
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * indices.size(), &indices[0] , GL_STATIC_DRAW);
}

void Model::bindBuffers(){
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
  glVertexAttribPointer(
    0,
    3,
    GL_FLOAT,
    GL_FALSE,
    0,
    (void*)0
  );
  glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
  glVertexAttribPointer(
    1,
    3,
    GL_FLOAT,
    GL_FALSE,
    0,
    (void*)0
  );
  glBindBuffer(GL_ARRAY_BUFFER, uvBuffer);
  glVertexAttribPointer(
    2,
    2,
    GL_FLOAT,
    GL_FALSE,
    0,
    (void*)0
  );
  glBindBuffer(GL_ARRAY_BUFFER, tangentBuffer);
  glVertexAttribPointer(
    3,
    3,
    GL_FLOAT,
    GL_FALSE,
    0,
    (void*)0
  );
  glBindBuffer(GL_ARRAY_BUFFER, bitangentBuffer);
  glVertexAttribPointer(
    4,
    3,
    GL_FLOAT,
    GL_FALSE,
    0,
    (void*)0
  );
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
}

void Model::bindVertexBuffer(){
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
  glVertexAttribPointer(
    0,
    3,
    GL_FLOAT,
    GL_FALSE,
    0,
    (void*)0
  );
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
}

bool Model::loadObj(std::string directory, std::string fileName, float scale, bool triangulate, bool averageVertexNormals){

  std::string objPath = directory + "/" + fileName;
  std::string mtlPath = directory;

  tinyobj::attrib_t                 attrib;
  std::vector<tinyobj::shape_t>     shapes;
  std::vector<tinyobj::material_t>  materials;

  std::string                       warn;
  std::string                       err;

  bool returnValue = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, objPath.c_str(), mtlPath.c_str(), triangulate);

  if (!err.empty()) {
    std::cout << "error loading model " + objPath + ": " + err << std::endl;
  }
  if (!warn.empty() && warn.find("Material file") == std::string::npos) {
    std::cout << "warning loading model " + objPath + ": " + warn << std::endl;
  }

  int uvIndexCount = (int)attrib.texcoords.size();
  if (uvIndexCount == 0) {
    std::cout << "warning loading model " + objPath + ": no texture coordinates" << std::endl;
  }

  if (!returnValue) {
    return false;
  }

  std::vector<glm::vec3> unindexedVertices;
  std::vector<glm::vec3> unindexedNormals;
  std::vector<glm::vec2> unindexedUvs;

  for(size_t i = 0; i < shapes.size(); i++){
    size_t index_offset = 0;
    for(size_t j = 0; j < shapes[i].mesh.num_face_vertices.size(); j++){
      int faceVertices = shapes[i].mesh.num_face_vertices[j];
      if (faceVertices != 3) {
        std::cout << "error loading model " + objPath + ": non-triangular faces in the model" << std::endl;
        return false;
      }
      for(size_t k = 0; k < faceVertices; k++){
        tinyobj::index_t idx = shapes[i].mesh.indices[index_offset + k];
        tinyobj::real_t vx = attrib.vertices[3*idx.vertex_index+0];
        tinyobj::real_t vy = attrib.vertices[3*idx.vertex_index+1];
        tinyobj::real_t vz = attrib.vertices[3*idx.vertex_index+2];
        tinyobj::real_t nx = attrib.normals[3*idx.normal_index+0];
        tinyobj::real_t ny = attrib.normals[3*idx.normal_index+1];
        tinyobj::real_t nz = attrib.normals[3*idx.normal_index+2];
        tinyobj::real_t tx;
        tinyobj::real_t ty;
        glm::vec3 vertex(vx, vy, vz);
        glm::vec3 normal(nx, ny, nz);
        if(uvIndexCount > 0){
          tx = attrib.texcoords[2*idx.texcoord_index+0];
          ty = attrib.texcoords[2*idx.texcoord_index+1];
        } else {
          if(k == 0){
            tx = 0.0f;
            ty = 0.0f;
          } else if(k == 1){
            tx = 1.0f;
            ty = 0.0f;
          } else if(k == 2){
            tx = 0.0f;
            ty = 1.0f;
          }
        }
        unindexedVertices.emplace_back(vertex * scale);
        unindexedNormals.emplace_back(glm::normalize(normal));
        unindexedUvs.emplace_back(tx, 1.0f - ty);
      }
      index_offset += faceVertices;
    }
  }

  if(averageVertexNormals){
    std::vector<std::vector<glm::vec3>> vertexNormals;
    for(int i = 0; i < unindexedVertices.size(); i++){
      vertexNormals.emplace_back(std::vector<glm::vec3>());
      for(int j = 0; j < unindexedVertices.size(); j++){
        if(isNear(unindexedVertices[i], unindexedVertices[j])){
          vertexNormals[i].emplace_back(unindexedNormals[j]);
        }
      }
    }
    for(int i = 0; i < vertexNormals.size(); i++){
      vertexNormals[i] = getUniqueNormals(vertexNormals[i]);
      glm::vec3 sumVector(0.0f, 0.0f, 0.0f);
      for(int j = 0; j < vertexNormals[i].size(); j++){
        sumVector += vertexNormals[i][j];
      }
      unindexedNormals[i] = glm::normalize(sumVector);
    }
  }

  std::vector<glm::vec3> unindexedTangents;
  std::vector<glm::vec3> unindexedBitangents;

  computeTangentBasis(unindexedVertices, unindexedUvs, unindexedNormals, unindexedTangents, unindexedBitangents);

  struct PackedVertex {
    glm::vec3 vertex;
    glm::vec2 uv;
    glm::vec3 normal;
    bool operator < (const PackedVertex other) const {
      return (!isNear(vertex, other.vertex) ||
              !isNear(uv, other.uv) ||
              !isNear(normal, other.normal));
    };
  };

  unsigned int newIndex;
  std::map<PackedVertex, unsigned int> packedVertexMap;
  for(int i = 0; i < unindexedVertices.size(); i++){
    PackedVertex packedVertex = {unindexedVertices[i], unindexedUvs[i], unindexedNormals[i]};
    unsigned int index;
    std::map<PackedVertex, unsigned int>::iterator it = packedVertexMap.find(packedVertex);
    if(it == packedVertexMap.end()){
      vertices.emplace_back(unindexedVertices[i]);
      normals.emplace_back(unindexedNormals[i]);
      uvs.emplace_back(unindexedUvs[i]);
      tangents.emplace_back(unindexedTangents[i]);
      bitangents.emplace_back(unindexedBitangents[i]);
      newIndex = (unsigned int)vertices.size() - 1;
      indices.emplace_back(newIndex);
      packedVertexMap[packedVertex] = newIndex;
    }
    else {
      index = it->second;
      indices.emplace_back(index);
      tangents[index] += unindexedTangents[i];
      bitangents[index] += unindexedBitangents[i];
    }
  }
  for(int i = 0; i < tangents.size(); i++){
    tangents[i] = glm::normalize(tangents[i]);
    bitangents[i] = glm::normalize(bitangents[i]);
  }

  maxWidth = calculateMaxWidth(&unindexedVertices);

  std::cout << objPath + " index count: " + std::to_string(indices.size()) << std::endl;

  return true;
}

bool Model::loadCollisionMeshFromObj(std::string path, std::vector<Collision::Polyhedron>* polyhedrons, float scale, bool mirrorXZ){
  tinyobj::attrib_t                 attrib;
  std::vector<tinyobj::shape_t>     shapes;
  std::vector<tinyobj::material_t>  materials;

  std::string                       warn;
  std::string                       err;

  bool returnValue = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, path.c_str(), nullptr, false);

  if (!err.empty()) {
    std::cout << "error loading model " + path + ": " + err << std::endl;
  }
  if (!warn.empty() && warn.find("Material file") == std::string::npos) {
    std::cout << "warning loading model " + path + ": " + warn << std::endl;
  }
  if (mirrorXZ){
    std::cout << "WARNING: using mirroring of X and Z axis; make sure there is no overlapping geometry: " + path << std::endl;
  }

  if (!returnValue) {
    return false;
  }

  for(size_t i = 0; i < shapes.size(); i++){
    size_t index_offset = 0;
    for(size_t j = 0; j < shapes[i].mesh.num_face_vertices.size(); j++){
      std::vector<glm::vec3> faceVerticeVector;
      int faceVertices = shapes[i].mesh.num_face_vertices[j];
      for(size_t k = 0; k < faceVertices; k++){
        tinyobj::index_t idx = shapes[i].mesh.indices[index_offset + k];
        tinyobj::real_t vx = attrib.vertices[3*idx.vertex_index+0];
        tinyobj::real_t vy = attrib.vertices[3*idx.vertex_index+1];
        tinyobj::real_t vz = attrib.vertices[3*idx.vertex_index+2];
        faceVerticeVector.emplace_back(vx*scale, vy*scale, vz*scale);
      }
      glm::vec3 faceNormal = Utility::getTriangleNormal(&faceVerticeVector);
      if (!Utility::verticesFormPlane(&faceVerticeVector, &faceNormal)) {
        return false;
      }
      polyhedrons->emplace_back();
      Collision::constructPlane(&polyhedrons->back(), &faceVerticeVector, &faceNormal);

      if(mirrorXZ){
        for(size_t k = 0; k < faceVertices; k++){
          faceVerticeVector[k].x = -faceVerticeVector[k].x;
        }
        faceNormal = Utility::getTriangleNormal(&faceVerticeVector);
        polyhedrons->emplace_back();
        Collision::constructPlane(&polyhedrons->back(), &faceVerticeVector, &faceNormal);
        for(size_t k = 0; k < faceVertices; k++){
          faceVerticeVector[k].z = -faceVerticeVector[k].z;
        }
        faceNormal = Utility::getTriangleNormal(&faceVerticeVector);
        polyhedrons->emplace_back();
        Collision::constructPlane(&polyhedrons->back(), &faceVerticeVector, &faceNormal);
        for(size_t k = 0; k < faceVertices; k++){
          faceVerticeVector[k].x = -faceVerticeVector[k].x;
        }
        faceNormal = Utility::getTriangleNormal(&faceVerticeVector);
        polyhedrons->emplace_back();
        Collision::constructPlane(&polyhedrons->back(), &faceVerticeVector, &faceNormal);
      }

      index_offset += faceVertices;
    }
  }
  return true;
}

bool Model::writeModelToFile(std::string path){
  unsigned int size = Utility::getVectorDataSize(&vertices) +
                      Utility::getVectorDataSize(&normals) +
                      Utility::getVectorDataSize(&uvs) +
                      Utility::getVectorDataSize(&tangents) +
                      Utility::getVectorDataSize(&bitangents) +
                      Utility::getVectorDataSize(&indices);
  char* memBlock = new char [size];
  char* memPointer = memBlock;
  memPointer += Utility::writeVector(memPointer, &vertices);
  memPointer += Utility::writeVector(memPointer, &normals);
  memPointer += Utility::writeVector(memPointer, &uvs);
  memPointer += Utility::writeVector(memPointer, &tangents);
  memPointer += Utility::writeVector(memPointer, &bitangents);
  memPointer += Utility::writeVector(memPointer, &indices);
  std::ofstream file(path, std::ios::out|std::ios::binary|std::ios::trunc);
  if(file.is_open()){
    file.write (memBlock, size);
    file.close();
    delete[] memBlock;
  } else {
    std::cout << "error writing model: " + path << std::endl;
    return false;
  }
  std::cout << "model written to file: " + path << std::endl;
  return true;
}

bool Model::readModelFromFile(std::string path){
  std::streampos size;
  char* memBlock;
  std::ifstream file(path, std::ios::in|std::ios::binary|std::ios::ate);
  if(file.is_open()){
    size = file.tellg();
    memBlock = new char [size];
    file.seekg (0, std::ios::beg);
    file.read (memBlock, size);
    file.close();
    char* memPointer = memBlock;
    memPointer += Utility::readVector(memPointer, &vertices);
    memPointer += Utility::readVector(memPointer, &normals);
    memPointer += Utility::readVector(memPointer, &uvs);
    memPointer += Utility::readVector(memPointer, &tangents);
    memPointer += Utility::readVector(memPointer, &bitangents);
    memPointer += Utility::readVector(memPointer, &indices);
    delete[] memBlock;
  } else {
    std::cout << "error reading model: " + path << std::endl;
    return false;
  }
  std::cout << "model read from file: " + path << std::endl;
  return true;
}

void Model::copyModel(Model* model, float scale){
  vertices.clear();
  vertices.reserve(model->vertices.size());
  for(int i = 0; i < model->vertices.size(); i++){
    vertices.emplace_back(model->vertices[i].x*scale, model->vertices[i].y*scale, model->vertices[i].z*scale);
  }
  normals = model->normals;
  uvs = model->uvs;
  tangents = model->tangents;
  bitangents = model->bitangents;
  indices = model->indices;
}

int Model::getId(){
  return id;
}

unsigned int Model::getIndexCount(){
  return indices.size();
}

float Model::getMaxWidth(){
  return maxWidth;
}

void Model::printInfo(){
  std::cout << "normals" << std::endl;
  for(int i = 0; i < normals.size(); i++){
    glm::vec3 n = normals[i];
    std::cout << std::to_string(n.x) << std::endl;
    std::cout << std::to_string(n.y) << std::endl;
    std::cout << std::to_string(n.z) << std::endl;
    std::cout << "---" << std::endl;
  }
}
