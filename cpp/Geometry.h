#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <map>
#include <memory>
#include <vector>

#include "Collision.h"
#include "Object.h"

class Geometry {
private:
public:
  enum class Type {
    null,
    box,
    cylinder,
    sphere,
    primitiveGroup,
    plane,
    mesh
  };

  Type      type;
  float     maxOverlapDistance;
  Geometry* rootGeometry;

  Geometry();
  virtual ~Geometry();

  virtual void transformToWorldSpace(glm::mat4 modelMatrix) = 0;
};

class Box: public Geometry {
private:
public:
  Collision::Polyhedron* collisionPolyhedron;
  Collision::Polyhedron  collisionPolyhedronWorldSpace;

  Box();
  ~Box();

  void setup(Collision::Polyhedron* collisionPolyhedron);
  void transformToWorldSpace(glm::mat4 modelMatrix);
};

class Cylinder: public Geometry {
private:
public:
  float                  radius;
  float                  halfHeight;
  Collision::Polyhedron* collisionPolyhedron;
  Collision::Polyhedron  collisionPolyhedronWorldSpace;

  Cylinder();
  ~Cylinder();

  void setup(float radius, float halfHeight, Collision::Polyhedron* collisionPolyhedron);
  void transformToWorldSpace(glm::mat4 modelMatrix);
};

class Sphere: public Geometry {
private:
public:
  float   radius;
  float   offset;
  Object* fakeTyreParent;

  Sphere();
  ~Sphere();

  void setup(float radius, float fakeTyreWidth = 0.0f, Object* fakeTyreParent = nullptr);
  void transformToWorldSpace(glm::mat4 modelMatrix);
};

class PrimitiveGroup: public Geometry {
private:
  float calculateMaxOverlapDistance();

public:
  enum class Identifier {
    null,
    hull,
    tyre
  };

  struct Primitive {
    Identifier                identifier;
    glm::vec3                 position;
    glm::mat4                 modelMatrix;
    std::unique_ptr<Geometry> geometry;
  };

  std::vector<Primitive> primitives;

  PrimitiveGroup();
  ~PrimitiveGroup();

  void setup();
  void addBox(Identifier identifier, glm::vec3 position, glm::mat4 modelMatrix, Collision::Polyhedron* collisionPolyhedron);
  void addCylinder(Identifier identifier, glm::vec3 position, glm::mat4 modelMatrix, Collision::Polyhedron* collisionPolyhedron);
  void addSphere(Identifier identifier, glm::vec3 position, glm::mat4 modelMatrix, float radius);
  void transformToWorldSpace(glm::mat4 modelMatrix);
};

class Plane: public Geometry {
private:
  float calculateMaxOverlapDistance(Collision::Polyhedron* collisionPolyhedron);

public:
  Collision::Polyhedron* collisionPolyhedron;
  Collision::Polyhedron  collisionPolyhedronWorldSpace;

  Plane();
  ~Plane();

  void setup(Collision::Polyhedron* collisionPolyhedron);
  void transformToWorldSpace(glm::mat4 modelMatrix);
};

class Mesh: public Geometry {
public:
  struct Aabb {
    glm::vec3 min;
    glm::vec3 max;
  };

  struct PlaneData {
    Plane     plane;
    Aabb      aabb;
    glm::vec3 positionLocalSpace;
    glm::vec3 positionWorldSpace;
  };

private:
  // implement Mesh as a vector of sectors (addressed by a location)
  // the idea is to avoid checking for collision against every plane (similarly to octree); first determine the general location to narrow down potentially overlapping geometry
  // each sector has a vector of Plane pointers
  // a Plane (with precalculated AABB) is included in each sector whose volume it overlaps

  struct Sector {
    Aabb                    aabb;
    std::vector<PlaneData*> planes;
  };

  int                    sectorCount;
  float                  sectorSize;
  Aabb                   aabb;
  std::vector<PlaneData> planes;
  std::vector<Sector>    sectors;

  float      calculateMaxOverlapDistance();
  Aabb       calculateAabb(std::vector<Collision::Polyhedron>* polyhedrons);
  Aabb       calculateAabb(Collision::Polyhedron* polyhedron);
  Aabb       calculateCubicAabb(Aabb aabb);
  bool       getAabbOverlap(Aabb a, Aabb b);
  bool       getAabbSphereOverlap(Aabb a, glm::vec3 spherePosition, float sphereRadius);
  int        getIndex(glm::ivec3 sectorPosition);
  glm::vec3  getMeshAabbPosition(glm::vec3 meshSpacePosition);
  glm::ivec3 getSectorPosition(glm::vec3 position);

public:
  Mesh();
  ~Mesh();

  float                   getSectorSize();
  void                    getPotentiallyOverlappingGeometry(std::vector<PlaneData*>* geometry, glm::mat4 meshModelMatrixInverse, glm::vec3 objectPosition, float objectMaxOverlapDistance);
  std::vector<PlaneData>* getPlanes();

  void setup(int sectorCount, std::vector<Collision::Polyhedron>* polyhedrons);
  void transformToWorldSpace(glm::mat4 modelMatrix);
};

#endif
