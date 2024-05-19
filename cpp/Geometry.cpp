#include "Geometry.h"
#include "Debug.h"

Geometry::Geometry(){
  type = Type::null;
  maxOverlapDistance = 0.0f;
  rootGeometry = nullptr;
}
Geometry::~Geometry(){
}

Box::Box(){
  type = Type::box;
}
Box::~Box(){
}

void Box::setup(Collision::Polyhedron* collisionPolyhedron){
  this->maxOverlapDistance = glm::length(collisionPolyhedron->maxHalfWidth);
  this->collisionPolyhedron = collisionPolyhedron;
  Collision::copyPolyhedron(collisionPolyhedron, &(this->collisionPolyhedronWorldSpace));
}

void Box::transformToWorldSpace(glm::mat4 modelMatrix){
  Collision::transformPolyhedron(collisionPolyhedron, &collisionPolyhedronWorldSpace, modelMatrix);
}

Cylinder::Cylinder(){
  type = Type::cylinder;
}
Cylinder::~Cylinder(){
}

void Cylinder::setup(float radius, float halfHeight, Collision::Polyhedron* collisionPolyhedron){
  this->maxOverlapDistance = glm::length(glm::vec2(radius, halfHeight));
  this->radius = radius;
  this->halfHeight = halfHeight;
  this->collisionPolyhedron = collisionPolyhedron;
  Collision::copyPolyhedron(collisionPolyhedron, &(this->collisionPolyhedronWorldSpace));
}

void Cylinder::transformToWorldSpace(glm::mat4 modelMatrix){
  Collision::transformPolyhedron(collisionPolyhedron, &collisionPolyhedronWorldSpace, modelMatrix);
}

Sphere::Sphere(){
  type = Type::sphere;
}
Sphere::~Sphere(){}

void Sphere::setup(float radius, float fakeTyreWidth, Object* fakeTyreParent){
  if(fakeTyreWidth > 0.0f){
    this->radius = fakeTyreWidth * 0.5f;
    this->offset = radius - this->radius;
    this->fakeTyreParent = fakeTyreParent;
    this->maxOverlapDistance = this->radius;
  } else {
    this->radius = radius;
    this->offset = 0.0f;
    this->fakeTyreParent = nullptr;
    this->maxOverlapDistance = radius;
  }
}

void Sphere::transformToWorldSpace(glm::mat4 modelMatrix){
}


float PrimitiveGroup::calculateMaxOverlapDistance(){
  float maxDistance = 0.0f;
  for(int i = 0; i < this->primitives.size(); i++){
    float distance = glm::length(this->primitives[i].position) + this->primitives[i].geometry->maxOverlapDistance;
    if(distance > maxDistance){
      maxDistance = distance;
    }
  }
  return maxDistance;
}

PrimitiveGroup::PrimitiveGroup(){
  type = Type::primitiveGroup;
}
PrimitiveGroup::~PrimitiveGroup(){
}

void PrimitiveGroup::setup(){
}

void PrimitiveGroup::addBox(Identifier identifier, glm::vec3 position, glm::mat4 modelMatrix, Collision::Polyhedron* collisionPolyhedron){
  primitives.emplace_back(Primitive{identifier, position, modelMatrix, std::make_unique<Box>()});
  Box* box = static_cast<Box*>(primitives.back().geometry.get());
  box->setup(collisionPolyhedron);
  box->rootGeometry = this;
  this->maxOverlapDistance = calculateMaxOverlapDistance();
}

void PrimitiveGroup::addCylinder(Identifier identifier, glm::vec3 position, glm::mat4 modelMatrix, Collision::Polyhedron* collisionPolyhedron){
  primitives.emplace_back(Primitive{identifier, position, modelMatrix, std::make_unique<Cylinder>()});
  Cylinder* cylinder = static_cast<Cylinder*>(primitives.back().geometry.get());
  cylinder->setup(collisionPolyhedron->maxHalfWidth.x, collisionPolyhedron->maxHalfWidth.y, collisionPolyhedron);
  cylinder->rootGeometry = this;
  this->maxOverlapDistance = calculateMaxOverlapDistance();
}

void PrimitiveGroup::addSphere(Identifier identifier, glm::vec3 position, glm::mat4 modelMatrix, float radius){
  primitives.emplace_back(Primitive{identifier, position, modelMatrix, std::make_unique<Sphere>()});
  Sphere* sphere = static_cast<Sphere*>(primitives.back().geometry.get());
  sphere->setup(radius);
  sphere->rootGeometry = this;
  this->maxOverlapDistance = calculateMaxOverlapDistance();
}

void PrimitiveGroup::transformToWorldSpace(glm::mat4 modelMatrix){
  for(int i = 0; i < primitives.size(); i++){
    primitives[i].geometry->transformToWorldSpace(modelMatrix * primitives[i].modelMatrix);
  }
}

float Plane::calculateMaxOverlapDistance(Collision::Polyhedron* collisionPolyhedron){
  float maxDistance = 0.0f;
  for(int i = 0; i < collisionPolyhedron->vertices.size(); i++){
    float distance = glm::length(collisionPolyhedron->vertices[i]);
    if(distance > maxDistance){
      maxDistance = distance;
    }
  }
  return maxDistance;
}

Plane::Plane(){
  type = Type::plane;
}
Plane::~Plane(){
}

void Plane::setup(Collision::Polyhedron* collisionPolyhedron){
  this->maxOverlapDistance = calculateMaxOverlapDistance(collisionPolyhedron);
  this->collisionPolyhedron = collisionPolyhedron;
  Collision::copyPolyhedron(collisionPolyhedron, &(this->collisionPolyhedronWorldSpace));
}

void Plane::transformToWorldSpace(glm::mat4 modelMatrix){
  Collision::transformPolyhedron(collisionPolyhedron, &collisionPolyhedronWorldSpace, modelMatrix);
}

float Mesh::calculateMaxOverlapDistance(){
  float maxDistance = 0.0f;
  for(int i = 0; i < this->planes.size(); i++){
    float distance = glm::length(this->planes[i].positionLocalSpace) + this->planes[i].plane.maxOverlapDistance;
    if(distance > maxDistance){
      maxDistance = distance;
    }
  }
  return maxDistance;
}

Mesh::Aabb Mesh::calculateAabb(std::vector<Collision::Polyhedron>* polyhedrons){
  Aabb aabb = Aabb{glm::vec3(FLT_MAX, FLT_MAX, FLT_MAX), glm::vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX)};
  for(int i = 0; i < polyhedrons->size(); i++){
    Collision::Polyhedron* polyhedron = &((*polyhedrons)[i]);
    glm::vec3 supportPointMinX = Collision::getSupportPointCoordinate(polyhedron, glm::vec3(-1.0f, 0.0f, 0.0f));
    if(supportPointMinX.x < aabb.min.x){
      aabb.min.x = supportPointMinX.x;
    }
    glm::vec3 supportPointMaxX = Collision::getSupportPointCoordinate(polyhedron, glm::vec3(1.0f, 0.0f, 0.0f));
    if(supportPointMaxX.x > aabb.max.x){
      aabb.max.x = supportPointMaxX.x;
    }
    glm::vec3 supportPointMinY = Collision::getSupportPointCoordinate(polyhedron, glm::vec3(0.0f, -1.0f, 0.0f));
    if(supportPointMinY.y < aabb.min.y){
      aabb.min.y = supportPointMinY.y;
    }
    glm::vec3 supportPointMaxY = Collision::getSupportPointCoordinate(polyhedron, glm::vec3(0.0f, 1.0f, 0.0f));
    if(supportPointMaxY.y > aabb.max.y){
      aabb.max.y = supportPointMaxY.y;
    }
    glm::vec3 supportPointMinZ = Collision::getSupportPointCoordinate(polyhedron, glm::vec3(0.0f, 0.0f, -1.0f));
    if(supportPointMinZ.z < aabb.min.z){
      aabb.min.z = supportPointMinZ.z;
    }
    glm::vec3 supportPointMaxZ = Collision::getSupportPointCoordinate(polyhedron, glm::vec3(0.0f, 0.0f, 1.0f));
    if(supportPointMaxZ.z > aabb.max.z){
      aabb.max.z = supportPointMaxZ.z;
    }
  }
  return aabb;
}

Mesh::Aabb Mesh::calculateAabb(Collision::Polyhedron* polyhedron){
  Aabb aabb;
  aabb.min.x = Collision::getSupportPointCoordinate(polyhedron, glm::vec3(-1.0f, 0.0f, 0.0f)).x;
  aabb.max.x = Collision::getSupportPointCoordinate(polyhedron, glm::vec3(1.0f, 0.0f, 0.0f)).x;
  aabb.min.y = Collision::getSupportPointCoordinate(polyhedron, glm::vec3(0.0f, -1.0f, 0.0f)).y;
  aabb.max.y = Collision::getSupportPointCoordinate(polyhedron, glm::vec3(0.0f, 1.0f, 0.0f)).y;
  aabb.min.z = Collision::getSupportPointCoordinate(polyhedron, glm::vec3(0.0f, 0.0f, -1.0f)).z;
  aabb.max.z = Collision::getSupportPointCoordinate(polyhedron, glm::vec3(0.0f, 0.0f, 1.0f)).z;
  return aabb;
}

Mesh::Aabb Mesh::calculateCubicAabb(Aabb aabb){
  const float x = aabb.max.x - aabb.min.x;
  const float y = aabb.max.y - aabb.min.y;
  const float z = aabb.max.z - aabb.min.z;
  if(x > y){
    if(x > z){
      const float yDiff = (x-y)*0.5f;
      const float zDiff = (x-z)*0.5f;
      return Aabb{glm::vec3(aabb.min.x, aabb.min.y-yDiff, aabb.min.z-zDiff), glm::vec3(aabb.max.x, aabb.max.y+yDiff, aabb.max.z+zDiff)};
    }
    const float xDiff = (z-x)*0.5f;
    const float yDiff = (z-y)*0.5f;
    return Aabb{glm::vec3(aabb.min.x-xDiff, aabb.min.y-yDiff, aabb.min.z), glm::vec3(aabb.max.x+xDiff, aabb.max.y+yDiff, aabb.max.z)};
  }
  if(x > z){
    const float xDiff = (y-x)*0.5f;
    const float zDiff = (y-z)*0.5f;
    return Aabb{glm::vec3(aabb.min.x-xDiff, aabb.min.y, aabb.min.z-zDiff), glm::vec3(aabb.max.x+xDiff, aabb.max.y, aabb.max.z+zDiff)};
  }
  const float xDiff = (z-x)*0.5f;
  const float yDiff = (z-y)*0.5f;
  return Aabb{glm::vec3(aabb.min.x-xDiff, aabb.min.y-yDiff, aabb.min.z), glm::vec3(aabb.max.x+xDiff, aabb.max.y+yDiff, aabb.max.z)};
}

bool Mesh::getAabbOverlap(Aabb a, Aabb b){
  return !(a.max.x < b.min.x || a.min.x > b.max.x ||
           a.max.y < b.min.y || a.min.y > b.max.y ||
           a.max.z < b.min.z || a.min.z > b.max.z);
}

bool Mesh::getAabbSphereOverlap(Aabb aabb, glm::vec3 spherePosition, float sphereRadius){
  glm::vec3 nearestPointInBox(
    std::max(aabb.min.x, std::min(spherePosition.x, aabb.max.x)),
    std::max(aabb.min.y, std::min(spherePosition.y, aabb.max.y)),
    std::max(aabb.min.z, std::min(spherePosition.z, aabb.max.z))
  );
  return (glm::length2(spherePosition - nearestPointInBox) < (sphereRadius * sphereRadius));
}

int Mesh::getIndex(glm::ivec3 sectorPosition){
  return (sectorCount * sectorCount * sectorPosition.z) + (sectorCount * sectorPosition.y) + sectorPosition.x;
}

glm::vec3 Mesh::getMeshAabbPosition(glm::vec3 meshSpacePosition){
  return meshSpacePosition - this->aabb.min;
}

glm::ivec3 Mesh::getSectorPosition(glm::vec3 position){
  return glm::ivec3(position / sectorSize);
}

Mesh::Mesh(){
  type = Type::mesh;
}
Mesh::~Mesh(){}

float Mesh::getSectorSize(){
  return this->sectorSize;
}

void Mesh::getPotentiallyOverlappingGeometry(std::vector<PlaneData*>* geometry, glm::mat4 meshModelMatrixInverse, glm::vec3 objectPosition, float objectMaxOverlapDistance){
  glm::vec3 objectMeshSpacePosition = glm::vec3(meshModelMatrixInverse * glm::vec4(objectPosition, 1.0f));
  glm::vec3 objectMeshAabbPosition = getMeshAabbPosition(objectMeshSpacePosition);
  glm::ivec3 sectorPosition = getSectorPosition(objectMeshAabbPosition);
  glm::vec3 sectorCenter;
  glm::vec3 sectorPositionReal = glm::vec3(sectorPosition);
  sectorCenter = (sectorPositionReal + 0.5f) * this->sectorSize;

  glm::ivec3 offset = glm::ivec3(0, 0, 0);
  if(objectMeshAabbPosition.x < sectorCenter.x){
    offset += glm::ivec3(-1, 0, 0);
  }
  if(objectMeshAabbPosition.y < sectorCenter.y){
    offset += glm::ivec3(0, -1, 0);
  }
  if(objectMeshAabbPosition.z < sectorCenter.z){
    offset += glm::ivec3(0, 0, -1);
  }

  std::map<PlaneData*, bool> checkedPlanes;
  for(int x = 0; x < 2; x++){
    for(int y = 0; y < 2; y++){
      for(int z = 0; z < 2; z++){
        const glm::ivec3 testSector = sectorPosition + glm::ivec3(x, y, z) + offset;
        if(testSector.x >= 0 &&
           testSector.y >= 0 &&
           testSector.z >= 0 &&
           testSector.x < this->sectorCount &&
           testSector.y < this->sectorCount &&
           testSector.z < this->sectorCount){
          const std::vector<PlaneData*>* planeData = &(this->sectors[getIndex(testSector)].planes);
          for(int i = 0; i < planeData->size(); i++){
            PlaneData* planeDataAddress = (*planeData)[i];
            if(checkedPlanes.count(planeDataAddress) == 0){
              if(getAabbSphereOverlap(planeDataAddress->aabb, objectMeshSpacePosition, objectMaxOverlapDistance)){
                geometry->emplace_back(planeDataAddress);
              }
              checkedPlanes[planeDataAddress] = true;
            }
          }
        }
      }
    }
  }
}

std::vector<Mesh::PlaneData>* Mesh::getPlanes(){
  return &(this->planes);
}

void Mesh::setup(int sectorCount, std::vector<Collision::Polyhedron>* polyhedrons){
  this->sectorCount = sectorCount;
  const Aabb meshAabb = calculateAabb(polyhedrons);
  this->aabb = calculateCubicAabb(meshAabb);
  this->sectors.resize(sectorCount * sectorCount * sectorCount);
  const float aabbSize = this->aabb.max.x - this->aabb.min.x;
  this->sectorSize = aabbSize / (float)sectorCount;

  this->planes.resize(polyhedrons->size());
  for(int i = 0; i < polyhedrons->size(); i++){
    Collision::Polyhedron* polyhedron = &((*polyhedrons)[i]);
    this->planes[i].plane.setup(polyhedron);
    this->planes[i].plane.rootGeometry = this;
    const Aabb planeAabb = calculateAabb(polyhedron);
    this->planes[i].aabb = planeAabb;
    this->planes[i].positionLocalSpace = planeAabb.min + 0.5f * (planeAabb.max - planeAabb.min);
  }

  this->maxOverlapDistance = calculateMaxOverlapDistance();

  for(int x = 0; x < sectorCount; x++){
    for(int y = 0; y < sectorCount; y++){
      for(int z = 0; z < sectorCount; z++){
        const glm::ivec3 ivec = glm::ivec3(x, y, z);
        const int sectorIndex = getIndex(ivec);
        const glm::vec3 sectorMin = this->aabb.min + (glm::vec3(ivec) * this->sectorSize);
        this->sectors[sectorIndex] = Sector{Aabb{sectorMin, sectorMin + glm::vec3(this->sectorSize, this->sectorSize, this->sectorSize)}};
        for(int i = 0; i < this->planes.size(); i++){
          if(getAabbOverlap(this->sectors[sectorIndex].aabb, this->planes[i].aabb)){
            this->sectors[sectorIndex].planes.emplace_back(&(this->planes[i]));
          }
        }
      }
    }
  }

}

void Mesh::transformToWorldSpace(glm::mat4 modelMatrix){
  for(int i = 0; i < this->planes.size(); i++){
    this->planes[i].plane.transformToWorldSpace(modelMatrix);
    this->planes[i].positionWorldSpace = glm::vec3(modelMatrix * glm::vec4(this->planes[i].positionLocalSpace, 1.0f));
  }
}
