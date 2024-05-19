#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>

#include "Object.h"
#include "Collision.h"
#include "Geometry.h"

class CollisionPair;

class RigidBody {
public:

  enum class Identifier {
    null, hull, tyre, ground
  };

private:

  static const float                      epsilon;
  static const float                      clampingCoefficient;
  static const float                      muStatic;
  static const float                      muDynamic;
  static const float                      muStaticSpin;
  static const float                      muDynamicSpin;

  struct SupportManifold {
    glm::vec3 normal;
    bool isStable;
  };

  float                                   mass;
  float                                   inverseMass;

  glm::mat3                               inertiaTensor;
  glm::mat3                               inertiaTensorWorld;
  glm::mat3                               inverseInertiaTensor;
  glm::mat3                               inverseInertiaTensorWorld;
  std::unique_ptr<Geometry>               geometry;
  Identifier                              identifier;
  std::vector<Collision::ContactManifold> contactManifolds;
  SupportManifold                         supportManifold;
  bool                                    inRest;

  bool                                    tyreGroundCollision;
  float                                   collisionImpulseMagnitude;
  glm::vec3                               relativeTyreGroundPointVelocity;

  Object                                  object;
  glm::vec3                               momentum;
  glm::vec3                               angularMomentum;

  void recalculateLinear();
  void recalculateAngular();
  void recalculateInertiaTensorWorld();

  struct Impulse {
    glm::vec3 impulse;
    glm::vec3 point;
  };

  struct EnergyPreservingImpulseResult {
    float scaleFactor;
    float energyDifference;
  };

  float                                getVelocityDecay(glm::vec3 variable, float upperSpeed, float lowerSpeed, float minDecayFactor, float maxDecayFactor);
  bool                                 isStableContactManifold(Collision::ContactManifold* contactManifold, glm::vec3 gravityVector);
  bool                                 isStationaryOnSlope(glm::vec3 gravity, glm::vec3 normal);

public:

  RigidBody();
  ~RigidBody();

  void                                     setMassAndInertiaTensor(float mass, glm::mat3 inertiaTensor);
  void                                     setStatic();
  bool                                     isStatic();
  float                                    getMass();
  float                                    getInverseMass();
  glm::mat3                                getInverseInertiaTensorWorld();
  void                                     setGeometryBox(Collision::Polyhedron* collisionPolyhedron);
  void                                     setGeometryCylinder(Collision::Polyhedron* collisionPolyhedron);
  void                                     setGeometrySphere(float radius, float fakeTyreWidth = 0.0f, Object* fakeTyreParent = nullptr);
  void                                     setGeometryPrimitiveGroup();
  void                                     setGeometryMesh(int sectorCount, std::vector<Collision::Polyhedron>* collisionMesh);
  Geometry*                                getGeometry();
  void                                     setIdentifier(Identifier identifier);
  Identifier                               getIdentifier();
  void                                     addContactManifold(Collision::ContactManifold contactManifold);
  void                                     clearContactManifolds();
  std::vector<Collision::ContactManifold>* getContactManifolds();
  void                                     determineSupportManifold(glm::vec3 gravityVector);
  SupportManifold*                         getSupportManifold();
  bool                                     isInRest();

  void                           setPosition(glm::vec3 position);
  glm::vec3                      getPosition();
  glm::vec3                      getPosition(RigidBody* other);
  void                           setVelocity(glm::vec3 velocity);
  glm::vec3                      getVelocity();
  glm::vec3                      getPointVelocity(glm::vec3 point);
  void                           setOrientation(glm::quat orientation);
  void                           setOrientationFromDirection(glm::vec3 direction);
  void                           setOrientationFromDirection(glm::vec3 direction, glm::vec3 up);
  void                           rotate(glm::vec3 magnitude);
  void                           rotateGlobal(glm::vec3 magnitude);
  glm::quat                      getOrientation();
  void                           setAngularVelocity(glm::vec3 angularVelocity);
  glm::vec3                      getAngularVelocity();
  glm::vec3                      getAngularMomentum();

  glm::mat4                      getModelMatrix();
  glm::mat4                      getModelMatrixInverse();
  glm::mat3                      getRotationMatrix();
  Object*                        getObject();

  void                           setFrameVariables();
  bool                           getTyreGroundCollision();
  float                          getCollisionImpulseMagnitude();
  glm::vec3                      getRelativeTyreGroundPointVelocity();

  void                           applyImpulse(glm::vec3 impulse, glm::vec3 point);
  void                           applyGravity(glm::vec3 g, float dt);
  void                           applyAngularImpulse(glm::vec3 impulse);
  static void                    applyCollisionResponse(RigidBody* a, RigidBody* b, Collision::ContactManifold contactManifold);
  void                           applyRestDetection();
  void                           integrate(float dt);
};

#endif
