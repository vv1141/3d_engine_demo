#ifndef COLLISION_PAIR_H
#define COLLISION_PAIR_H

#include <vector>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/vector_angle.hpp>

#include <limits>

#include "Collision.h"
#include "RigidBody.h"
#include "Utility.h"

class CollisionPair {
public:
  struct Contact {
    Collision::ContactManifold contactManifold;
    RigidBody*                 A;
    RigidBody*                 B;
    bool                       orderFlipped;
    int                        getSortIdentifier() const {
      // depriorise car hull ground contacts
      if((A->getIdentifier() == RigidBody::Identifier::hull && B->getIdentifier() == RigidBody::Identifier::ground) ||
         (A->getIdentifier() == RigidBody::Identifier::ground && B->getIdentifier() == RigidBody::Identifier::hull)) {
        return std::numeric_limits<int>::max();
      }
      glm::vec3 averagePoint = glm::vec3(0.0f, 0.0f, 0.0f);
      for(int i = 0; i < contactManifold.points.size(); i++) {
        glm::vec3 p = contactManifold.points[i].position;
        averagePoint += p;
      }
      averagePoint *= (1.0f / (float)contactManifold.points.size());
      return glm::dot(averagePoint, sortDirection);
    }
  };

private:
  RigidBody* A;
  RigidBody* B;
  bool       useTemporalCoherence;

  struct PrimitiveData {
    glm::vec3 position;
    glm::mat4 modelMatrix;
    glm::mat4 modelMatrixInverse;
  };

  static glm::vec3                                       sortDirection;
  static std::map<Geometry*, PrimitiveData>              primitiveDataCache;
  static std::map<Geometry*, std::map<Geometry*, bool>>  primitiveGroupCollisions;
  static std::map<Geometry*, Collision::ContactManifold> primitiveProximityOverlap;

  bool    testBroadPhaseCollision(Geometry* geometryA, glm::vec3 positionA, Geometry* geometryB, glm::vec3 positionB);
  Contact getPolyhedronPolyhedronContact(Collision::Polyhedron* polyhedron1, Collision::Polyhedron* polyhedron2, bool flipOrder);
  Contact getBoxBoxContact(Geometry* box1, Geometry* box2);
  Contact getBoxCylinderContact(Geometry* box, Geometry* cylinder, bool flipOrder);
  Contact getCylinderCylinderContact(Geometry* cylinder1, Geometry* cylinder2);
  Contact getSphereBoxContact(Geometry* sphere, Geometry* box, glm::vec3 spherePosition, glm::mat4 boxModelMatrix, glm::mat4 boxModelMatrixInverse, bool flipOrder);
  Contact getSphereCylinderContact(Geometry* sphere, Geometry* cylinder, glm::vec3 spherePosition, glm::mat4 cylinderModelMatrix, glm::mat4 cylinderModelMatrixInverse, bool flipOrder);
  Contact getSphereSphereContactManifold(Geometry* sphere1, Geometry* sphere2, glm::vec3 sphere1Position, glm::vec3 sphere2Position);
  Contact getBoxPlaneContact(Geometry* box, Geometry* plane, bool flipOrder);
  Contact getCylinderPlaneContact(Geometry* cylinder, Geometry* plane, bool flipOrder);
  Contact getSpherePlaneContact(Geometry* sphere, Geometry* plane, glm::vec3 spherePosition, bool flipOrder);
  Contact getPlanePlaneContact(Geometry* plane1, Geometry* plane2);
  Contact getPrimitiveGroupContact(Geometry* primitiveGroup, glm::mat4 primitiveGroupModelMatrix, Geometry* geometryOther, glm::vec3 positionOther, glm::mat4 modelMatrixOther, glm::mat4 modelMatrixInverseOther, bool flipOrder);
  Contact getMeshContact(Geometry* mesh, glm::mat4 meshModelMatrix, glm::mat4 meshModelMatrixInverse, Geometry* geometryOther, glm::vec3 positionOther, glm::mat4 modelMatrixOther, glm::mat4 modelMatrixInverseOther, bool flipOrder);

public:
  CollisionPair(RigidBody* A, RigidBody* B, bool useTemporalCoherence = false);
  ~CollisionPair();

  static bool                                             checkMeshCollisionPairMaxOverlapValidity(std::vector<CollisionPair>* collisionPairs);
  static std::map<Geometry*, std::map<Geometry*, bool>>*  getPrimitiveGroupCollisions();
  static std::map<Geometry*, Collision::ContactManifold>* getPrimitiveProximityOverlap();
  static void                                             clearDataCaches();
  static void                                             sortContacts(std::vector<Contact>* contacts, glm::vec3 direction);
  Contact                                                 getContact();
  Contact                                                 getContact(Geometry* geometryA, glm::vec3 positionA, glm::mat4 modelMatrixA, glm::mat4 modelMatrixInverseA, Geometry* geometryB, glm::vec3 positionB, glm::mat4 modelMatrixB, glm::mat4 modelMatrixInverseB);
};

#endif
