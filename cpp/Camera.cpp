#include "Camera.h"

Camera::Camera() {
  state = State::orbit;
  position = glm::vec3(0, 0, 0);
  viewMatrix = glm::mat4(1.0f);
  vehicle = nullptr;
}

Camera::~Camera() {
}

void Camera::setupVehicleCamera(Vehicle* vehicle, float distance, float height, float angle) {
  this->vehicle = vehicle;
  this->vehicleCameraDistance = distance;
  this->vehicleCameraHeight = height;
  this->vehicleCameraAngle = angle;
}

void Camera::setupOrbitCamera(Object* object, float initialDistance, float minDistance, float maxDistance, float minAngleVertical, float maxAngleVertical) {
  this->orbitCameraObject = object;
  this->orbitCameraInitialDistance = initialDistance;
  this->orbitCameraMinDistance = minDistance;
  this->orbitCameraMaxDistance = maxDistance;
  this->orbitCameraMinAngleVertical = minAngleVertical;
  this->orbitCameraMaxAngleVertical = maxAngleVertical;
  this->orbitCameraDistance = initialDistance;
  this->orbitCameraAngleHorizontal = Utility::tau * 0.125f;
  this->orbitCameraAngleVertical = Utility::tau * 0.125f;
}

glm::vec3 Camera::getPosition() {
  return position;
}
glm::mat4 Camera::getViewMatrix() {
  return viewMatrix;
}

void Camera::processInput(float dt, Input* input, bool isPaused) {
  if(input->keyHit("toggleCamera")) {
    if(state == State::vehicleFollow) {
      state = State::vehicleFixed;
    } else if(state == State::vehicleFixed) {
      state = State::orbit;
    } else if(state == State::orbit) {
      state = State::vehicleFollow;
    }

    if(state == State::vehicleFollow) {
    } else if(state == State::vehicleFixed) {
    } else if(state == State::orbit) {
      orbitCameraDistance = orbitCameraInitialDistance;
    }
  }
  if(state == State::vehicleFollow) {
  } else if(state == State::vehicleFixed) {
  } else if(state == State::orbit) {
    if(input->keyDown("rotateCamera")) {
      orbitCameraAngleHorizontal -= (float)input->getMouseVelocity().x / 100.0;
      orbitCameraAngleVertical += (float)input->getMouseVelocity().y / 100.0;
      orbitCameraAngleVertical = glm::clamp(orbitCameraAngleVertical, orbitCameraMinAngleVertical, orbitCameraMaxAngleVertical);
    }
    if(input->keyHit("moveCameraCloser")) {
      orbitCameraDistance *= 0.9f;
    }
    if(input->keyHit("moveCameraFurther")) {
      orbitCameraDistance *= 1.1f;
    }
    orbitCameraDistance = glm::clamp(orbitCameraDistance, orbitCameraMinDistance, orbitCameraMaxDistance);
  }
}

void Camera::updatePosition(double alpha, bool isPaused) {
  Object* p = vehicle->getHull()->getObject();
  if(state == State::vehicleFollow) {
    glm::vec3 vehiclePosition;
    glm::mat3 vehicleRotationMatrix;
    if(isPaused) {
      vehiclePosition = p->getPreviousInterpolatedPosition();
      vehicleRotationMatrix = glm::mat3(glm::mat4_cast(p->getPreviousInterpolatedOrientation()));
    } else {
      vehiclePosition = p->getInterpolatedPosition(alpha);
      vehicleRotationMatrix = glm::mat3(glm::mat4_cast(p->getInterpolatedOrientation(alpha)));
    }
    glm::vec3 directionVector = vehicleRotationMatrix * glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 directionNormal = vehicleRotationMatrix * glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 cameraHeightVector = glm::vec3(0.0f, vehicleCameraHeight, 0.0f);
    position = vehiclePosition - directionVector * vehicleCameraDistance + cameraHeightVector;
    this->viewMatrix = glm::lookAt(
      position,
      vehiclePosition + (1.0f - vehicleCameraAngle) * cameraHeightVector,
      glm::vec3(0.0f, 1.0f, 0.0f));
  } else if(state == State::vehicleFixed) {
    glm::vec3 vehiclePosition;
    glm::mat3 vehicleRotationMatrix;
    if(isPaused) {
      vehiclePosition = p->getPreviousInterpolatedPosition();
      vehicleRotationMatrix = glm::mat3(glm::mat4_cast(p->getPreviousInterpolatedOrientation()));
    } else {
      vehiclePosition = p->getInterpolatedPosition(alpha);
      vehicleRotationMatrix = glm::mat3(glm::mat4_cast(p->getInterpolatedOrientation(alpha)));
    }
    glm::vec3 directionVector = vehicleRotationMatrix * glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 directionNormal = vehicleRotationMatrix * glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 cameraHeightVector = directionNormal * vehicleCameraHeight;
    position = vehiclePosition - directionVector * -1.0f + glm::vec3(0.0f, 1.0f, 0.0f);
    this->viewMatrix = glm::lookAt(
      position,
      position + directionVector,
      directionNormal);
  } else if(state == State::orbit) {
    glm::vec3 unitHorizontal(sin(orbitCameraAngleHorizontal), 0.0f, cos(orbitCameraAngleHorizontal));
    glm::vec3 unitRotated = glm::rotate(unitHorizontal, orbitCameraAngleVertical, glm::cross(unitHorizontal, glm::vec3(0.0f, 1.0f, 0.0f)));
    glm::vec3 orbitCameraObjectPosition;
    if(isPaused) {
      orbitCameraObjectPosition = orbitCameraObject->getPreviousInterpolatedPosition();
    } else {
      orbitCameraObjectPosition = orbitCameraObject->getInterpolatedPosition(alpha);
    }
    position = orbitCameraObjectPosition + unitRotated * orbitCameraDistance;
    this->viewMatrix = glm::lookAt(
      position,
      orbitCameraObjectPosition,
      glm::vec3(0.0f, 1.0f, 0.0f));
  }
}
