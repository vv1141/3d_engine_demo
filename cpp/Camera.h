#ifndef CAMERA_H
#define CAMERA_H

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/rotate_vector.hpp>

#include "Vehicle.h"
#include "Input.h"
#include "Utility.h"
#include "Debug.h"

class Camera {
private:

  enum class State {
    vehicleFollow, vehicleFixed, orbit
  };

  State     state;
  glm::vec3 position;
  glm::mat4 viewMatrix;

  Input::PlayerKeybindProfiles* keybindProfiles;
  Input::Joystick*              joystick;
  float*                        fov;

  Vehicle*  vehicle;
  float     vehicleCameraDistance;
  float     vehicleCameraHeight;
  float     vehicleCameraAngle;

  Object*   orbitCameraObject;
  float     orbitCameraInitialDistance;
  float     orbitCameraMinDistance;
  float     orbitCameraMaxDistance;
  float     orbitCameraMinAngleVertical;
  float     orbitCameraMaxAngleVertical;
  float     orbitCameraDistance;
  float     orbitCameraAngleHorizontal;
  float     orbitCameraAngleVertical;

public:

  Camera();
  ~Camera();

  void setupVehicleCamera(Vehicle* vehicle, float distance, float height, float angle);
  void setupOrbitCamera(Object* object, float initialDistance, float minDistance, float maxDistance, float minAngleVertical, float maxAngleVertical);

  glm::vec3 getPosition();
  glm::mat4 getViewMatrix();
  void processInput(float dt, Input* input, bool isPaused);
  void updatePosition(double alpha, bool isPaused);

};

#endif
