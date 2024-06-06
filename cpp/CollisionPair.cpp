#include "CollisionPair.h"
#include "Debug.h"

glm::vec3                                         CollisionPair::sortDirection;
std::map<Geometry*, CollisionPair::PrimitiveData> CollisionPair::primitiveDataCache = std::map<Geometry*, CollisionPair::PrimitiveData>();
std::map<Geometry*, std::map<Geometry*, bool>>    CollisionPair::primitiveGroupCollisions = std::map<Geometry*, std::map<Geometry*, bool>>();
std::map<Geometry*, Collision::ContactManifold>   CollisionPair::primitiveProximityOverlap = std::map<Geometry*, Collision::ContactManifold>();

bool CollisionPair::testBroadPhaseCollision(Geometry* geometryA, glm::vec3 positionA, Geometry* geometryB, glm::vec3 positionB) {
  return Utility::onDistance(positionA, positionB, geometryA->maxOverlapDistance + geometryB->maxOverlapDistance);
}

CollisionPair::CollisionPair(RigidBody* A, RigidBody* B, bool useTemporalCoherence) {
  if(A->isStatic()) {
    this->A = B;
    this->B = A;
  } else {
    this->A = A;
    this->B = B;
  }
  this->useTemporalCoherence = useTemporalCoherence;
}

CollisionPair::~CollisionPair() {
}

CollisionPair::Contact CollisionPair::getPolyhedronPolyhedronContact(Collision::Polyhedron* polyhedron1, Collision::Polyhedron* polyhedron2, bool flipOrder) {
  Collision::IsStatic isStatic = Collision::IsStatic::null;
  if(B->isStatic()) {
    isStatic = Collision::IsStatic::B;
    if(flipOrder) isStatic = Collision::IsStatic::A;
  }
  Collision::ContactManifold contactManifold;
  contactManifold = Collision::getPolyhedronContactManifold(polyhedron1, polyhedron2, isStatic, this->useTemporalCoherence);
  if(flipOrder) return Contact{contactManifold, B, A, flipOrder};
  return Contact{contactManifold, A, B, flipOrder};
}

CollisionPair::Contact CollisionPair::getBoxBoxContact(Geometry* box1, Geometry* box2) {
  Collision::Polyhedron* box1Polyhedron = &(static_cast<Box*>(box1)->collisionPolyhedronWorldSpace);
  Collision::Polyhedron* box2Polyhedron = &(static_cast<Box*>(box2)->collisionPolyhedronWorldSpace);
  return getPolyhedronPolyhedronContact(box1Polyhedron, box2Polyhedron, false);
}

CollisionPair::Contact CollisionPair::getBoxCylinderContact(Geometry* box, Geometry* cylinder, bool flipOrder) {
  Collision::Polyhedron* boxPolyhedron = &(static_cast<Box*>(box)->collisionPolyhedronWorldSpace);
  Collision::Polyhedron* cylinderPolyhedron = &(static_cast<Cylinder*>(cylinder)->collisionPolyhedronWorldSpace);
  return getPolyhedronPolyhedronContact(boxPolyhedron, cylinderPolyhedron, flipOrder);
}

CollisionPair::Contact CollisionPair::getCylinderCylinderContact(Geometry* cylinder1, Geometry* cylinder2) {
  Collision::Polyhedron* cylinder1Polyhedron = &(static_cast<Cylinder*>(cylinder1)->collisionPolyhedronWorldSpace);
  Collision::Polyhedron* cylinder2Polyhedron = &(static_cast<Cylinder*>(cylinder2)->collisionPolyhedronWorldSpace);
  return getPolyhedronPolyhedronContact(cylinder1Polyhedron, cylinder2Polyhedron, false);
}

CollisionPair::Contact CollisionPair::getSphereBoxContact(Geometry* sphere, Geometry* box, glm::vec3 spherePosition, glm::mat4 boxModelMatrix, glm::mat4 boxModelMatrixInverse, bool flipOrder) {
  float                      sphereRadius = static_cast<Sphere*>(sphere)->radius;
  glm::vec3                  boxMaxHalfWidth = static_cast<Box*>(box)->collisionPolyhedron->maxHalfWidth;
  Collision::ContactManifold contactManifold = Collision::getSphereBoxContactManifold(spherePosition, sphereRadius, boxMaxHalfWidth, boxModelMatrix, boxModelMatrixInverse);
  if(flipOrder) return Contact{contactManifold, B, A, flipOrder};
  return Contact{contactManifold, A, B, flipOrder};
}

CollisionPair::Contact CollisionPair::getSphereCylinderContact(Geometry* sphere, Geometry* cylinder, glm::vec3 spherePosition, glm::mat4 cylinderModelMatrix, glm::mat4 cylinderModelMatrixInverse, bool flipOrder) {
  float                      sphereRadius = static_cast<Sphere*>(sphere)->radius;
  float                      cylinderRadius = static_cast<Cylinder*>(cylinder)->radius;
  float                      cylinderHalfHeight = static_cast<Cylinder*>(cylinder)->halfHeight;
  Collision::ContactManifold contactManifold = Collision::getSphereCylinderContactManifold(spherePosition, sphereRadius, cylinderRadius, cylinderHalfHeight, cylinderModelMatrix, cylinderModelMatrixInverse);
  if(flipOrder) return Contact{contactManifold, B, A, flipOrder};
  return Contact{contactManifold, A, B, flipOrder};
}

CollisionPair::Contact CollisionPair::getSphereSphereContactManifold(Geometry* sphere1, Geometry* sphere2, glm::vec3 sphere1Position, glm::vec3 sphere2Position) {
  float                      sphere1Radius = static_cast<Sphere*>(sphere1)->radius;
  float                      sphere2Radius = static_cast<Sphere*>(sphere2)->radius;
  Collision::ContactManifold contactManifold = Collision::getSphereSphereContactManifold(sphere1Position, sphere1Radius, sphere2Position, sphere2Radius);
  return Contact{contactManifold, A, B, false};
}

CollisionPair::Contact CollisionPair::getBoxPlaneContact(Geometry* box, Geometry* plane, bool flipOrder) {
  Collision::Polyhedron* boxPolyhedron = &(static_cast<Box*>(box)->collisionPolyhedronWorldSpace);
  Collision::Polyhedron* planePolyhedron = &(static_cast<Plane*>(plane)->collisionPolyhedronWorldSpace);
  return getPolyhedronPolyhedronContact(boxPolyhedron, planePolyhedron, flipOrder);
}

CollisionPair::Contact CollisionPair::getCylinderPlaneContact(Geometry* cylinder, Geometry* plane, bool flipOrder) {
  Collision::Polyhedron* cylinderPolyhedron = &(static_cast<Cylinder*>(cylinder)->collisionPolyhedronWorldSpace);
  Collision::Polyhedron* planePolyhedron = &(static_cast<Plane*>(plane)->collisionPolyhedronWorldSpace);
  return getPolyhedronPolyhedronContact(cylinderPolyhedron, planePolyhedron, flipOrder);
}

CollisionPair::Contact CollisionPair::getSpherePlaneContact(Geometry* sphere, Geometry* plane, glm::vec3 spherePosition, bool flipOrder) {
  float                      sphereRadius = static_cast<Sphere*>(sphere)->radius;
  Collision::Polyhedron*     planePolyhedron = &(static_cast<Plane*>(plane)->collisionPolyhedronWorldSpace);
  Collision::ContactManifold contactManifold = Collision::getSpherePlaneContactManifold(spherePosition, sphereRadius, planePolyhedron);
  if(flipOrder) return Contact{contactManifold, B, A, flipOrder};
  return Contact{contactManifold, A, B, flipOrder};
}

CollisionPair::Contact CollisionPair::getPlanePlaneContact(Geometry* plane1, Geometry* plane2) {
  Collision::Polyhedron* plane1Polyhedron = &(static_cast<Plane*>(plane1)->collisionPolyhedronWorldSpace);
  Collision::Polyhedron* plane2Polyhedron = &(static_cast<Plane*>(plane2)->collisionPolyhedronWorldSpace);
  return getPolyhedronPolyhedronContact(plane1Polyhedron, plane2Polyhedron, false);
}

CollisionPair::Contact CollisionPair::getPrimitiveGroupContact(Geometry* primitiveGroup, glm::mat4 primitiveGroupModelMatrix, Geometry* geometryOther, glm::vec3 positionOther, glm::mat4 modelMatrixOther, glm::mat4 modelMatrixInverseOther, bool flipOrder) {
  PrimitiveGroup*                         group = static_cast<PrimitiveGroup*>(primitiveGroup);
  std::vector<Collision::ContactManifold> contactManifolds;
  contactManifolds.reserve(group->primitives.size());

  int                         nonEmptyManifolds = 0;
  Collision::ContactManifold* nonEmptyManifold = nullptr;
  for(auto it = group->primitives.begin(); it != group->primitives.end(); ++it) {
    Geometry* p = it->geometry.get();
    auto      itCache = primitiveDataCache.find(p);
    if(itCache == primitiveDataCache.end()) {
      glm::mat4 modelMatrix = primitiveGroupModelMatrix * it->modelMatrix;
      primitiveDataCache[p] = PrimitiveData{
        glm::vec3(modelMatrix[3][0], modelMatrix[3][1], modelMatrix[3][2]),
        modelMatrix,
        glm::inverse(modelMatrix)};
    }

    Contact contact = getContact(p,
                                 primitiveDataCache[p].position,
                                 primitiveDataCache[p].modelMatrix,
                                 primitiveDataCache[p].modelMatrixInverse,
                                 geometryOther,
                                 positionOther,
                                 modelMatrixOther,
                                 modelMatrixInverseOther);
    if(contact.orderFlipped) contact.contactManifold.normal *= -1.0f;
    contactManifolds.emplace_back(contact.contactManifold);

    if(contact.contactManifold.points.size() > 0) {
      nonEmptyManifolds++;
      nonEmptyManifold = &contactManifolds.back();
      Geometry* otherPtr = geometryOther->rootGeometry;
      if(otherPtr == nullptr) otherPtr = geometryOther;
      primitiveGroupCollisions[p][otherPtr] = true;
    }
  }

  if(nonEmptyManifolds == 0) {
    return Contact{Collision::ContactManifold{}, nullptr, nullptr, false};
  }
  if(nonEmptyManifolds == 1) {
    if(flipOrder) return Contact{*nonEmptyManifold, B, A, flipOrder};
    return Contact{*nonEmptyManifold, A, B, flipOrder};
  }

  Collision::ContactManifold reducedManifold;
  if(Collision::joinContactManifolds(&contactManifolds, &reducedManifold)) {
    if(flipOrder) return Contact{reducedManifold, B, A, flipOrder};
    return Contact{reducedManifold, A, B, flipOrder};
  }
  return Contact{Collision::ContactManifold{}, nullptr, nullptr, false};
}

CollisionPair::Contact CollisionPair::getMeshContact(Geometry* mesh, glm::mat4 meshModelMatrix, glm::mat4 meshModelMatrixInverse, Geometry* geometryOther, glm::vec3 positionOther, glm::mat4 modelMatrixOther, glm::mat4 modelMatrixInverseOther, bool flipOrder) {
  std::vector<Collision::ContactManifold> contactManifolds;
  Mesh*                                   m = static_cast<Mesh*>(mesh);
  std::vector<Mesh::PlaneData*>           planes;
  m->getPotentiallyOverlappingGeometry(&planes, meshModelMatrixInverse, positionOther, geometryOther->maxOverlapDistance);

  for(auto it = planes.begin(); it != planes.end(); ++it) {
    Contact contact = getContact(&((*it)->plane),
                                 (*it)->positionWorldSpace,
                                 meshModelMatrix,
                                 meshModelMatrixInverse,
                                 geometryOther,
                                 positionOther,
                                 modelMatrixOther,
                                 modelMatrixInverseOther);
    if(contact.orderFlipped) contact.contactManifold.normal *= -1.0f;
    contactManifolds.emplace_back(contact.contactManifold);
  }

  int nonEmptyManifolds = 0;
  int nonEmptyManifoldIndex = 0;
  for(int i = 0; i < contactManifolds.size(); i++) {
    if(contactManifolds[i].points.size() > 0) {
      nonEmptyManifolds++;
      nonEmptyManifoldIndex = i;
    }
  }
  if(nonEmptyManifolds == 0) {
    return Contact{Collision::ContactManifold{}, nullptr, nullptr, false};
  }
  if(nonEmptyManifolds == 1) {
    if(flipOrder) return Contact{contactManifolds[nonEmptyManifoldIndex], B, A, flipOrder};
    return Contact{contactManifolds[nonEmptyManifoldIndex], A, B, flipOrder};
  }

  Collision::ContactManifold reducedManifold;
  if(Collision::joinContactManifolds(&contactManifolds, &reducedManifold)) {
    if(flipOrder) return Contact{reducedManifold, B, A, flipOrder};
    return Contact{reducedManifold, A, B, flipOrder};
  }
  return Contact{Collision::ContactManifold{}, nullptr, nullptr, false};
}

bool CollisionPair::checkMeshCollisionPairMaxOverlapValidity(std::vector<CollisionPair>* collisionPairs) {
  bool valid = true;
  for(auto it = collisionPairs->begin(); it != collisionPairs->end(); ++it) {
    if(it->A->getGeometry()->type == Geometry::Type::mesh) {
      Mesh* m = static_cast<Mesh*>(it->A->getGeometry());
      float dist = it->B->getGeometry()->maxOverlapDistance;
      if(dist > m->getSectorSize()) {
        Debug::log("Error: configured mesh sector size too small: sector size: " + std::to_string(m->getSectorSize()) + ", minimum size: " + std::to_string(dist));
        valid = false;
      }
    }
    if(it->B->getGeometry()->type == Geometry::Type::mesh) {
      Mesh* m = static_cast<Mesh*>(it->B->getGeometry());
      float dist = it->A->getGeometry()->maxOverlapDistance;
      if(dist > m->getSectorSize()) {
        Debug::log("Error: configured mesh sector size too small: sector size: " + std::to_string(m->getSectorSize()) + ", minimum size: " + std::to_string(dist));
        valid = false;
      }
    }
  }
  return valid;
}

std::map<Geometry*, std::map<Geometry*, bool>>* CollisionPair::getPrimitiveGroupCollisions() {
  return &primitiveGroupCollisions;
}

std::map<Geometry*, Collision::ContactManifold>* CollisionPair::getPrimitiveProximityOverlap() {
  return &primitiveProximityOverlap;
}

void CollisionPair::clearDataCaches() {
  primitiveDataCache.clear();
  primitiveGroupCollisions.clear();
  primitiveProximityOverlap.clear();
}

void CollisionPair::sortContacts(std::vector<Contact>* contacts, glm::vec3 direction) {
  sortDirection = direction;
  std::sort(
    contacts->begin(), contacts->end(), [](Contact const& a, Contact const& b) { return a.getSortIdentifier() < b.getSortIdentifier(); });
}

CollisionPair::Contact CollisionPair::getContact() {
  return getContact(A->getGeometry(), A->getPosition(B), A->getModelMatrix(), A->getModelMatrixInverse(), B->getGeometry(), B->getPosition(A), B->getModelMatrix(), B->getModelMatrixInverse());
}

CollisionPair::Contact CollisionPair::getContact(Geometry* geometryA, glm::vec3 positionA, glm::mat4 modelMatrixA, glm::mat4 modelMatrixInverseA, Geometry* geometryB, glm::vec3 positionB, glm::mat4 modelMatrixB, glm::mat4 modelMatrixInverseB) {
  if(testBroadPhaseCollision(geometryA, positionA, geometryB, positionB)) {
    if(geometryA->type == Geometry::Type::box) {
      if(geometryB->type == Geometry::Type::box) {
        return getBoxBoxContact(geometryA, geometryB);
      } else if(geometryB->type == Geometry::Type::cylinder) {
        return getBoxCylinderContact(geometryA, geometryB, false);
      } else if(geometryB->type == Geometry::Type::sphere) {
        return getSphereBoxContact(geometryB, geometryA, positionB, modelMatrixA, modelMatrixInverseA, true);
      } else if(geometryB->type == Geometry::Type::primitiveGroup) {
        return getPrimitiveGroupContact(geometryB, modelMatrixB, geometryA, positionA, modelMatrixA, modelMatrixInverseA, true);
      } else if(geometryB->type == Geometry::Type::plane) {
        return getBoxPlaneContact(geometryA, geometryB, false);
      } else if(geometryB->type == Geometry::Type::mesh) {
        return getMeshContact(geometryB, modelMatrixB, modelMatrixInverseB, geometryA, positionA, modelMatrixA, modelMatrixInverseA, true);
      }
    } else if(geometryA->type == Geometry::Type::cylinder) {
      if(geometryB->type == Geometry::Type::box) {
        return getBoxCylinderContact(geometryB, geometryA, true);
      } else if(geometryB->type == Geometry::Type::cylinder) {
        return getCylinderCylinderContact(geometryA, geometryB);
      } else if(geometryB->type == Geometry::Type::sphere) {
        return getSphereCylinderContact(geometryB, geometryA, positionB, modelMatrixA, modelMatrixInverseA, true);
      } else if(geometryB->type == Geometry::Type::primitiveGroup) {
        return getPrimitiveGroupContact(geometryB, modelMatrixB, geometryA, positionA, modelMatrixA, modelMatrixInverseA, true);
      } else if(geometryB->type == Geometry::Type::plane) {
        return getCylinderPlaneContact(geometryA, geometryB, false);
      } else if(geometryB->type == Geometry::Type::mesh) {
        return getMeshContact(geometryB, modelMatrixB, modelMatrixInverseB, geometryA, positionA, modelMatrixA, modelMatrixInverseA, true);
      }
    } else if(geometryA->type == Geometry::Type::sphere) {
      if(geometryB->type == Geometry::Type::box) {
        return getSphereBoxContact(geometryA, geometryB, positionA, modelMatrixB, modelMatrixInverseB, false);
      } else if(geometryB->type == Geometry::Type::cylinder) {
        return getSphereCylinderContact(geometryA, geometryB, positionA, modelMatrixB, modelMatrixInverseB, false);
      } else if(geometryB->type == Geometry::Type::sphere) {
        return getSphereSphereContactManifold(geometryA, geometryB, positionA, positionB);
      } else if(geometryB->type == Geometry::Type::primitiveGroup) {
        return getPrimitiveGroupContact(geometryB, modelMatrixB, geometryA, positionA, modelMatrixA, modelMatrixInverseA, true);
      } else if(geometryB->type == Geometry::Type::plane) {
        return getSpherePlaneContact(geometryA, geometryB, positionA, false);
      } else if(geometryB->type == Geometry::Type::mesh) {
        return getMeshContact(geometryB, modelMatrixB, modelMatrixInverseB, geometryA, positionA, modelMatrixA, modelMatrixInverseA, true);
      }
    } else if(geometryA->type == Geometry::Type::primitiveGroup) {
      if(geometryB->type == Geometry::Type::box ||
         geometryB->type == Geometry::Type::cylinder ||
         geometryB->type == Geometry::Type::sphere ||
         geometryB->type == Geometry::Type::primitiveGroup ||
         geometryB->type == Geometry::Type::plane) {
        return getPrimitiveGroupContact(geometryA, modelMatrixA, geometryB, positionB, modelMatrixB, modelMatrixInverseB, false);
      } else if(geometryB->type == Geometry::Type::mesh) {
        return getMeshContact(geometryB, modelMatrixB, modelMatrixInverseB, geometryA, positionA, modelMatrixA, modelMatrixInverseA, true);
      }
    } else if(geometryA->type == Geometry::Type::plane) {
      if(geometryB->type == Geometry::Type::box) {
        return getBoxPlaneContact(geometryB, geometryA, true);
      } else if(geometryB->type == Geometry::Type::cylinder) {
        return getCylinderPlaneContact(geometryB, geometryA, true);
      } else if(geometryB->type == Geometry::Type::sphere) {
        return getSpherePlaneContact(geometryB, geometryA, positionB, true);
      } else if(geometryB->type == Geometry::Type::primitiveGroup) {
        return getPrimitiveGroupContact(geometryB, modelMatrixB, geometryA, positionA, modelMatrixA, modelMatrixInverseA, true);
      } else if(geometryB->type == Geometry::Type::plane) {
        return getPlanePlaneContact(geometryA, geometryB);
      } else if(geometryB->type == Geometry::Type::mesh) {
        return getMeshContact(geometryB, modelMatrixB, modelMatrixInverseB, geometryA, positionA, modelMatrixA, modelMatrixInverseA, true);
      }
    } else if(geometryA->type == Geometry::Type::mesh) {
      if(geometryB->type == Geometry::Type::box ||
         geometryB->type == Geometry::Type::cylinder ||
         geometryB->type == Geometry::Type::sphere ||
         geometryB->type == Geometry::Type::primitiveGroup ||
         geometryB->type == Geometry::Type::plane ||
         geometryB->type == Geometry::Type::mesh) {
        return getMeshContact(geometryA, modelMatrixA, modelMatrixInverseA, geometryB, positionB, modelMatrixB, modelMatrixInverseB, false);
      }
    }
  }
  return Contact{Collision::ContactManifold{}, nullptr, nullptr, false};
}
