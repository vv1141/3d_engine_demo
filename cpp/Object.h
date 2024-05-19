#ifndef OBJECT_H
#define OBJECT_H

#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <limits>

class Object {
private:

  glm::vec3 position;
  glm::vec3 previousPosition;
  glm::vec3 previousInterpolatedPosition;
  glm::quat orientation;
  glm::quat previousOrientation;
  glm::quat previousInterpolatedOrientation;

  glm::vec3 velocity;
  glm::vec3 angularVelocity;

  glm::mat4 model;
  glm::mat4 modelInverse;
  glm::mat4 modelInterpolated;
  bool      modelInterpolatedUpdated;
  Object*   parentObject;
  bool      isStatic;

  glm::quat quaternionIntegrate(glm::quat q, glm::vec3 omega, float dt);
  void      calculateInterpolatedModelMatrix(float alpha);

public:

  Object();
  ~Object();

  void      setPosition(glm::vec3 position);
  glm::vec3 getPosition();
  void      setVelocity(glm::vec3 velocity);
  glm::vec3 getVelocity();
  void      setOrientation(glm::quat orientation);
  void      setOrientationFromDirection(glm::vec3 direction);
  void      setOrientationFromDirection(glm::vec3 direction, glm::vec3 up);
  void      setPositionAndOrientation(glm::vec3 position, glm::quat orientation);
  void      rotate(glm::vec3 magnitude);
  void      rotateGlobal(glm::vec3 magnitude);
  glm::vec3 rotateVectorToLocalSpaceAndNormalize(glm::vec3 v);
  glm::quat getOrientation();
  void      setAngularVelocity(glm::vec3 angularVelocity);
  glm::vec3 getAngularVelocity();

  bool      getModelInterpolatedUpdated();
  void      setParentObject(Object* object);
  void      setStatic(bool isStatic);
  glm::mat4 getModelMatrix();
  glm::mat4 getModelMatrixInverse();
  glm::mat3 getRotationMatrix();
  glm::mat4 getInterpolatedModelMatrix(float alpha);

  glm::vec3 getInterpolatedPosition(float alpha);
  glm::quat getInterpolatedOrientation(float alpha);
  glm::vec3 getPreviousInterpolatedPosition();
  glm::quat getPreviousInterpolatedOrientation();

  void      recalculateModelMatrix();
  void      integrate(float dt);

};

#endif
