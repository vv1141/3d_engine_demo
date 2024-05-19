#ifndef H_VEHICLE
#define H_VEHICLE

#include <vector>
#include <list>
#include <iterator>

#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/rotate_vector.hpp>

#include <limits>

#include "VehicleType.h"
#include "RigidBody.h"
#include "CollisionPair.h"
#include "Input.h"
#include "Constraint.h"
#include "SuspensionConstraint.h"
#include "FixedConstraint.h"

class Vehicle {
private:

  static const float epsilon;

  struct Spring {
    glm::vec3 center;
    RigidBody* tyre;
    Constraint* constraint;
  };

  float drivingImpulse;
  float brakingImpulse;
  float steeringSpeed;
  float steeringLimit;
  float rollingResistance;
  float internalRollingFriction;
  float tyreMaxAngularVelocity;

  struct VehicleInput {
    float accelerate;
    float decelerate;
    float turnLeft;
    float turnRight;
  };

  VehicleInput input;
  VehicleType* vehicleType;

  enum class TyrePosition {
    null, front, rear
  };

  struct TyreProperties {
    glm::vec3 position;
    glm::quat orientation;
    TyrePosition tyrePosition;
  };

  RigidBody* hull;
  std::vector<Spring> springs;
  std::vector<TyreProperties> tyreProperties;

  float steeringAngle;

  glm::vec3 getAxleVector(RigidBody* r);
  glm::vec3 getUpVector(RigidBody* r);
  glm::vec3 getForwardVector(RigidBody* r);
  float calculateSteeringSpeedFactor();
  void applyAngularImpulseToTyre(RigidBody* tyre, float impulse);
  void applyCounterRotationAngularImpulseToTyre(RigidBody* tyre, float impulse);

public:

  Vehicle();
  ~Vehicle();

  void setup(VehicleType* vehicleType, std::list<RigidBody>* rigidBodies, std::vector<CollisionPair>* collisionPairs, std::vector<std::unique_ptr<Constraint>>* constraints, float gravity);
  void setPosition(glm::vec3 position);
  void setVelocity(glm::vec3 velocity);
  void setAngularVelocity(glm::vec3 angularVelocity);
  void setOrientationFromDirection(glm::vec3 direction);
  void rotate(glm::vec3 magnitude);

  RigidBody* getTyre(int index);
  std::vector<RigidBody*> getTyres();
  int getTyresInContactCount();
  RigidBody* getHull();
  Spring* getSpring(int index);
  void applySpringForces(float dt);

  void processLocalInput(Input* input, Input::PlayerKeybindProfiles* keybindProfiles, Input::Joystick* joystick);
  void update(float dt);
  void applyRollingResistance();
};

#endif
