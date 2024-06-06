#include "Object.h"

glm::quat Object::quaternionIntegrate(glm::quat q, glm::vec3 omega, float dt) {
  glm::vec3 v = omega * dt;
  float     theta = glm::length(v);
  glm::quat vq;
  float     s;

  // rotation quaternion: q = [cos(1/2theta), w*sin(1/2theta)], where w is a unit vector
  if(theta > 0.0f) {
    vq.w = cos(0.5f * theta);
    s = sin(0.5f * theta) / theta;
  } else {
    vq.w = 1.0f;
    s = 1.0f;
  }
  vq.x = s * v.x;
  vq.y = s * v.y;
  vq.z = s * v.z;

  return vq * q; //Two rotation quaternions can be combined into one equivalent quaternion by the relation: q′ = q2q1
}

void Object::calculateInterpolatedModelMatrix(float alpha) {
  if(parentObject == nullptr) {
    previousInterpolatedPosition = getInterpolatedPosition(alpha);
    previousInterpolatedOrientation = getInterpolatedOrientation(alpha);
    glm::mat4 translation = glm::translate(glm::mat4(1.0f), previousInterpolatedPosition);
    glm::mat4 rotation = glm::mat4_cast(previousInterpolatedOrientation);
    modelInterpolated = translation * rotation;
    modelInterpolatedUpdated = true;
  } else {
    if(!parentObject->getModelInterpolatedUpdated()) {
      parentObject->calculateInterpolatedModelMatrix(alpha);
    }
    modelInterpolated = parentObject->getInterpolatedModelMatrix(alpha) * model;
  }
}

Object::Object() {
  position = glm::vec3(0.0f, 0.0f, 0.0f);
  previousPosition = position;
  previousInterpolatedPosition = position;
  orientation = glm::quatLookAt(glm::normalize(glm::vec3(0.0f, 0.0f, 1.0f)), glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f)));
  previousOrientation = orientation;
  previousInterpolatedOrientation = orientation;
  velocity = glm::vec3(0.0f, 0.0f, 0.0f);
  angularVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
  parentObject = nullptr;
  recalculateModelMatrix();
}

Object::~Object() {
}

void Object::setPosition(glm::vec3 position) {
  this->position = position;
  if(isStatic) previousPosition = position;
  recalculateModelMatrix();
}

glm::vec3 Object::getPosition() {
  return position;
}

void Object::setVelocity(glm::vec3 velocity) {
  this->velocity = velocity;
}

glm::vec3 Object::getVelocity() {
  return velocity;
}

void Object::setOrientation(glm::quat orientation) {
  this->orientation = orientation;
  recalculateModelMatrix();
}

void Object::setOrientationFromDirection(glm::vec3 direction) {
  glm::vec3 normalizedDirection = glm::normalize(direction);
  if(normalizedDirection == glm::vec3(0.0f, 1.0f, 0.0f)) {
    normalizedDirection = glm::vec3(0.0f, 1.0f, 0.00001f);
  }
  orientation = glm::quatLookAt(normalizedDirection, glm::vec3(0.0f, 1.0f, 0.0f));
  recalculateModelMatrix();
}

void Object::setOrientationFromDirection(glm::vec3 direction, glm::vec3 up) {
  glm::vec3 normalizedDirection = glm::normalize(direction);
  orientation = glm::quatLookAt(normalizedDirection, up); // assume direction is not parallel to up vector;
  recalculateModelMatrix();
}

void Object::setPositionAndOrientation(glm::vec3 position, glm::quat orientation) {
  this->position = position;
  this->orientation = orientation;
  recalculateModelMatrix();
}

void Object::rotate(glm::vec3 magnitude) {
  orientation = quaternionIntegrate(orientation, getRotationMatrix() * magnitude, 1.0f);
  orientation = glm::normalize(orientation);
  recalculateModelMatrix();
}

void Object::rotateGlobal(glm::vec3 magnitude) {
  orientation = quaternionIntegrate(orientation, magnitude, 1.0f);
  orientation = glm::normalize(orientation);
  recalculateModelMatrix();
}

glm::vec3 Object::rotateVectorToLocalSpaceAndNormalize(glm::vec3 v) {
  return glm::normalize(glm::conjugate(orientation) * v);
}

glm::quat Object::getOrientation() {
  return orientation;
}

void Object::setAngularVelocity(glm::vec3 angularVelocity) {
  this->angularVelocity = angularVelocity;
}

glm::vec3 Object::getAngularVelocity() {
  return angularVelocity;
}

bool Object::getModelInterpolatedUpdated() {
  return modelInterpolatedUpdated;
}

void Object::setParentObject(Object* parentObject) {
  this->parentObject = parentObject;
}

void Object::setStatic(bool isStatic) {
  this->isStatic = isStatic;
}

glm::mat4 Object::getModelMatrix() {
  return model;
}

glm::mat4 Object::getModelMatrixInverse() {
  return modelInverse;
}

glm::mat3 Object::getRotationMatrix() {
  return glm::mat3(model);
}

glm::mat4 Object::getInterpolatedModelMatrix(float alpha) {
  if(parentObject == nullptr) {
    if(!modelInterpolatedUpdated) {
      calculateInterpolatedModelMatrix(alpha);
    }
  } else {
    calculateInterpolatedModelMatrix(alpha);
  }
  return modelInterpolated;
}

glm::vec3 Object::getInterpolatedPosition(float alpha) {
  return (position * alpha) + (previousPosition * (1.0f - alpha));
}

glm::quat Object::getInterpolatedOrientation(float alpha) {
  return glm::slerp(previousOrientation, orientation, alpha);
}

glm::vec3 Object::getPreviousInterpolatedPosition() {
  return previousInterpolatedPosition;
}

glm::quat Object::getPreviousInterpolatedOrientation() {
  return previousInterpolatedOrientation;
}

void Object::recalculateModelMatrix() {
  glm::mat4 translation = glm::translate(glm::mat4(1.0f), position);
  glm::mat4 rotation = glm::mat4_cast(orientation);
  model = translation * rotation;
  modelInverse = glm::inverse(model);
  modelInterpolatedUpdated = false;
}

void Object::integrate(float dt) {
  previousPosition = position;
  previousOrientation = orientation;
  position += velocity * dt;
  orientation = quaternionIntegrate(orientation, angularVelocity, dt);
  orientation = glm::normalize(orientation);
  recalculateModelMatrix();
}
