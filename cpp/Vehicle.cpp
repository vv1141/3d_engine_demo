#include "Vehicle.h"

const float Vehicle::epsilon = 0.00001f;

glm::vec3 Vehicle::getAxleVector(RigidBody* r){
  return r->getRotationMatrix() * glm::vec3(1.0f, 0.0f, 0.0f);
}

glm::vec3 Vehicle::getUpVector(RigidBody* r){
  return r->getRotationMatrix() * glm::vec3(0.0f, 1.0f, 0.0f);
}

glm::vec3 Vehicle::getForwardVector(RigidBody* r){
  return r->getRotationMatrix() * glm::vec3(0.0f, 0.0f, 1.0f);
}

float Vehicle::calculateSteeringSpeedFactor(){
  const float steeringSpeedFactor = 20.0f;
  const float minimumSteeringFactor = 0.2f;
  RigidBody* t1 = springs[0].tyre;
  RigidBody* t2 = springs[2].tyre;
  glm::vec3 w1 = t1->getAngularVelocity();
  glm::vec3 w2 = t2->getAngularVelocity();
  float f = glm::length(w1 + w2) / 100.0f;
  return glm::clamp(exp(-steeringSpeedFactor*f), minimumSteeringFactor, 1.0f);
}

void Vehicle::applyAngularImpulseToTyre(RigidBody* tyre, float impulse){
    glm::vec3 axleVector = getForwardVector(tyre);
    glm::vec3 drivingImpulse = impulse * axleVector;
    tyre->applyAngularImpulse(-drivingImpulse);
    hull->applyAngularImpulse(drivingImpulse);
}

void Vehicle::applyCounterRotationAngularImpulseToTyre(RigidBody* tyre, float impulse){
  if(glm::length2(tyre->getAngularVelocity()) > 0.0f){
    const float wLength = glm::length(tyre->getAngularVelocity());
    glm::vec3 wDirection = tyre->getAngularVelocity() / wLength;
    const float effectiveImpulse = std::min(wLength, impulse);
    glm::vec3 axleVector = getForwardVector(tyre);
    glm::vec3 brakeImpulse = glm::dot(effectiveImpulse * wDirection, axleVector) * axleVector;
    tyre->applyAngularImpulse(-brakeImpulse);
    hull->applyAngularImpulse(brakeImpulse);
    glm::vec3 wDirectionAfter = tyre->getAngularVelocity();
    if(glm::dot(wDirection, wDirectionAfter) <= 0.0f) tyre->setAngularVelocity(glm::vec3(0.0f));
  }
}

Vehicle::Vehicle(){
  steeringAngle = 0.0f;
  drivingImpulse = 6.0f * vehicleType->hullMass;
  brakingImpulse = 2.0f * vehicleType->hullMass;
  steeringSpeed = 0.5f * Utility::tau;
  steeringLimit = 0.5f;
  rollingResistance = 0.05f;
  internalRollingFriction = 0.01f;
  tyreMaxAngularVelocity = 10.0f;
}
Vehicle::~Vehicle(){
}
void Vehicle::setup(VehicleType* vehicleType, std::list<RigidBody>* rigidBodies, std::vector<CollisionPair>* collisionPairs, std::vector<std::unique_ptr<Constraint>>* constraints, float gravity){
  this->vehicleType = vehicleType;
  rigidBodies->emplace_back();
  rigidBodies->back().setMassAndInertiaTensor(VehicleType::hullMass, vehicleType->hullRotationalInertiaTensor);
  rigidBodies->back().setGeometryBox(&vehicleType->hullHitbox);
  rigidBodies->back().setIdentifier(RigidBody::Identifier::hull);
  hull = &(rigidBodies->back());

  float fw = vehicleType->frontAxleWidth * 0.5f;
  float rw = vehicleType->rearAxleWidth * 0.5f;
  float fv = vehicleType->frontAxleVerticalOffset;
  float rv = vehicleType->rearAxleVerticalOffset;
  float fh = vehicleType->frontAxleHorizontalOffset;
  float rh = vehicleType->rearAxleHorizontalOffset;
  float suspensionMinLimitFront = fv * 0.5f;
  float suspensionMaxLimitFront = fv * -0.5f;
  float suspensionMinLimitRear = rv * 0.5f;
  float suspensionMaxLimitRear = rv * -0.5f;
  glm::vec2 limitsFront(suspensionMinLimitFront, suspensionMaxLimitFront);
  glm::vec2 limitsRear(suspensionMinLimitRear, suspensionMaxLimitRear);

  rigidBodies->emplace_back();
  rigidBodies->back().setMassAndInertiaTensor(VehicleType::tyreMass, vehicleType->tyreRotationalInertiaTensor);
  rigidBodies->back().setGeometrySphere(vehicleType->tyreRadius, vehicleType->tyreWidth, hull->getObject());
  rigidBodies->back().setIdentifier(RigidBody::Identifier::tyre);
  rigidBodies->back().setOrientationFromDirection(glm::vec3(1.0f, 0.0f, 0.0f));
  springs.emplace_back(Spring{glm::vec3(fw, fv, fh), &(rigidBodies->back())});

  rigidBodies->emplace_back();
  rigidBodies->back().setMassAndInertiaTensor(VehicleType::tyreMass, vehicleType->tyreRotationalInertiaTensor);
  rigidBodies->back().setGeometrySphere(vehicleType->tyreRadius, vehicleType->tyreWidth, hull->getObject());
  rigidBodies->back().setIdentifier(RigidBody::Identifier::tyre);
  rigidBodies->back().setOrientationFromDirection(glm::vec3(1.0f, 0.0f, 0.0f));
  springs.emplace_back(Spring{glm::vec3(rw, rv, rh), &(rigidBodies->back())});

  rigidBodies->emplace_back();
  rigidBodies->back().setMassAndInertiaTensor(VehicleType::tyreMass, vehicleType->tyreRotationalInertiaTensor);
  rigidBodies->back().setGeometrySphere(vehicleType->tyreRadius, vehicleType->tyreWidth, hull->getObject());
  rigidBodies->back().setIdentifier(RigidBody::Identifier::tyre);
  rigidBodies->back().setOrientationFromDirection(glm::vec3(-1.0f, 0.0f, 0.0f));
  springs.emplace_back(Spring{glm::vec3(-fw, fv, fh), &(rigidBodies->back())});

  rigidBodies->emplace_back();
  rigidBodies->back().setMassAndInertiaTensor(VehicleType::tyreMass, vehicleType->tyreRotationalInertiaTensor);
  rigidBodies->back().setGeometrySphere(vehicleType->tyreRadius, vehicleType->tyreWidth, hull->getObject());
  rigidBodies->back().setIdentifier(RigidBody::Identifier::tyre);
  rigidBodies->back().setOrientationFromDirection(glm::vec3(-1.0f, 0.0f, 0.0f));
  springs.emplace_back(Spring{glm::vec3(-rw, rv, rh), &(rigidBodies->back())});

  setPosition(glm::vec3(0.0f));

  auto end = rigidBodies->end();
  std::advance(end, -5);
  for (auto it = rigidBodies->begin(); it != end; ++it) {
    RigidBody* other = &(*it);
    collisionPairs->emplace_back(other, hull);
    collisionPairs->emplace_back(other, springs[0].tyre);
    collisionPairs->emplace_back(other, springs[1].tyre);
    collisionPairs->emplace_back(other, springs[2].tyre);
    collisionPairs->emplace_back(other, springs[3].tyre);
  }

  constraints->emplace_back(std::make_unique<SuspensionConstraint>(
    hull, springs[0].tyre, glm::vec3(fw, fv, fh), limitsFront)
  );
  springs[0].constraint = constraints->back().get();
  constraints->emplace_back(std::make_unique<SuspensionConstraint>(
    hull, springs[1].tyre, glm::vec3(rw, rv, rh), limitsRear)
  );
  springs[1].constraint = constraints->back().get();
  constraints->emplace_back(std::make_unique<SuspensionConstraint>(
    hull, springs[2].tyre, glm::vec3(-fw, fv, fh), limitsFront)
  );
  springs[2].constraint = constraints->back().get();
  constraints->emplace_back(std::make_unique<SuspensionConstraint>(
    hull, springs[3].tyre, glm::vec3(-rw, rv, rh), limitsRear)
  );
  springs[3].constraint = constraints->back().get();
}

RigidBody* Vehicle::getTyre(int index){
  return springs[index].tyre;
}

std::vector<RigidBody*> Vehicle::getTyres(){
  std::vector<RigidBody*> tyres;
  for(int i = 0; i < springs.size(); i++){
    tyres.emplace_back(springs[i].tyre);
  }
  return tyres;
}

void Vehicle::setPosition(glm::vec3 position){
  hull->setPosition(position);
  const glm::mat4 m = hull->getModelMatrix();
  for(int i = 0; i < springs.size(); i++){
    springs[i].tyre->setPosition(glm::vec3(m * glm::vec4(springs[i].center, 1)));
  }
}

void Vehicle::setVelocity(glm::vec3 velocity){
  hull->setVelocity(velocity);
  for(int i = 0; i < springs.size(); i++){
    springs[i].tyre->setVelocity(velocity);
  }
}

void Vehicle::setAngularVelocity(glm::vec3 angularVelocity){
  hull->setVelocity(angularVelocity);
  for(int i = 0; i < springs.size(); i++){
    springs[i].tyre->setAngularVelocity(angularVelocity);
  }
}

void Vehicle::setOrientationFromDirection(glm::vec3 direction){
  hull->setOrientationFromDirection(direction);
  const glm::vec3 axle = getAxleVector(hull);
  for(int i = 0; i < springs.size(); i++){
    if(i == 0 || i == 1){
      springs[i].tyre->setOrientationFromDirection(-axle);
    } else {
      springs[i].tyre->setOrientationFromDirection(axle);
    }
  }
  const glm::mat4 m = hull->getModelMatrix();
  for(int i = 0; i < springs.size(); i++){
    springs[i].tyre->setPosition(glm::vec3(m * glm::vec4(springs[i].center, 1)));
  }
}

void Vehicle::rotate(glm::vec3 magnitude){
  hull->rotate(magnitude);
  for(int i = 0; i < springs.size(); i++){
    springs[i].tyre->rotate(magnitude);
  }
  const glm::mat4 m = hull->getModelMatrix();
  for(int i = 0; i < springs.size(); i++){
    springs[i].tyre->setPosition(glm::vec3(m * glm::vec4(springs[i].center, 1)));
  }
}

RigidBody* Vehicle::getHull(){
  return hull;
}

Vehicle::Spring* Vehicle::getSpring(int index){
  return &(springs[index]);
}

void Vehicle::applySpringForces(float dt){
  std::vector<glm::vec3> forces;
  const glm::mat4 m = hull->getModelMatrix();
  const glm::vec3 normal = hull->getRotationMatrix() * glm::vec3(0.0f, 1.0f, 0.0f);
  for(int i = 0; i < springs.size(); i++){
    const glm::vec3 springCenter = glm::vec3(m * glm::vec4(springs[i].center.x, springs[i].center.y, springs[i].center.z, 1));
    const float tyreDisplacementAlongNormal = glm::dot(springs[i].tyre->getPosition() - springCenter, normal);
    const float tyreVelocityAlongNormal = glm::dot(springs[i].tyre->getVelocity() - hull->getPointVelocity(springCenter), normal);
    float force = -VehicleType::springConstant * tyreDisplacementAlongNormal;
    force -= VehicleType::springDampingCoefficient * tyreVelocityAlongNormal;
    const glm::vec3 forceWorldSpace = force * normal;
    forces.push_back(forceWorldSpace);
  }
  for(int i = 0; i < springs.size(); i++){
    const glm::vec3 springCenter = glm::vec3(m * glm::vec4(springs[i].center.x, springs[i].center.y, springs[i].center.z, 1));
    glm::vec3 impulse = forces[i] * dt;
    springs[i].tyre->applyImpulse(impulse, springs[i].tyre->getPosition());
    hull->applyImpulse(-impulse, springCenter);
  }
}

void Vehicle::processLocalInput(Input* input, Input::PlayerKeybindProfiles* keybindProfiles, Input::Joystick* joystick){
  this->input.accelerate = input->combineToAnalogValue("accelerate", keybindProfiles, joystick);
  this->input.decelerate = input->combineToAnalogValue("decelerate", keybindProfiles, joystick);
  this->input.turnLeft = input->combineToAnalogValue("turnLeft", keybindProfiles, joystick);
  this->input.turnRight = input->combineToAnalogValue("turnRight", keybindProfiles, joystick);
}

void Vehicle::update(float dt){
  const float steeringInput = -input.turnLeft + input.turnRight;
  float targetAngle = steeringInput * steeringLimit;
  float angleDifference = targetAngle - steeringAngle;
  float speedFactor = calculateSteeringSpeedFactor();
  if(steeringAngle < targetAngle){
    steeringAngle += steeringSpeed * speedFactor * dt;
  }
  if(steeringAngle > targetAngle){
    steeringAngle -= steeringSpeed * speedFactor * dt;
  }
  if((targetAngle - steeringAngle < 0.0f && angleDifference > 0.0f) || (targetAngle - steeringAngle > 0.0f && angleDifference < 0.0f)) {
    steeringAngle = targetAngle;
  }
  steeringAngle = glm::clamp(steeringAngle, -steeringLimit, steeringLimit);
  float outerWheelAngle = 0.0f;
  if(std::abs(steeringAngle) > epsilon){
    float wheelbase = std::abs(vehicleType->frontAxleHorizontalOffset) + std::abs(vehicleType->rearAxleHorizontalOffset);
    if(steeringAngle < 0.0f) outerWheelAngle = atan(1.0f / (cos(steeringAngle)/sin(steeringAngle) - (vehicleType->frontAxleWidth / wheelbase)));
    else outerWheelAngle = -atan(1.0f / (cos(-steeringAngle)/sin(-steeringAngle) - (vehicleType->frontAxleWidth / wheelbase)));
  }

  glm::vec3 zeroAxle = hull->getRotationMatrix() * glm::vec3(1.0f, 0.0f, 0.0f);
  const glm::vec3 normal = hull->getRotationMatrix() * glm::vec3(0.0f, 1.0f, 0.0f);
  const glm::vec3 hullDirection = glm::cross(zeroAxle, normal);
  int inner, outer;
  if(steeringAngle < 0.0f){
    inner = 2;
    outer = 0;
  }else{
    inner = 0;
    outer = 2;
  }
  glm::vec3 steeringVector;
  glm::vec3 steeringVectorOuter;
  if(inner == 0){
    steeringVector = glm::rotate(zeroAxle, -steeringAngle, normal);
    steeringVectorOuter = glm::rotate(-zeroAxle, -outerWheelAngle, normal);
  }else{
    steeringVector = glm::rotate(-zeroAxle, -steeringAngle, normal);
    steeringVectorOuter = glm::rotate(zeroAxle, -outerWheelAngle, normal);
  }

  RigidBody* innerTyre = springs[inner].tyre;
  glm::vec3 axleInner = getForwardVector(innerTyre);
  glm::vec3 upInner = getUpVector(innerTyre);
  const glm::vec3 p1i = innerTyre->getAngularMomentum();
  const float angleInner = glm::orientedAngle(axleInner, steeringVector, normal);
  glm::vec3 w = innerTyre->getAngularVelocity();
  float direction = 1.0f;
  if(glm::dot(w, axleInner) < 0.0f) direction = -1.0f;
  innerTyre->rotateGlobal(angleInner * normal);
  if(std::abs(glm::dot(axleInner, normal)) > epsilon) {
    float tyreAngle = glm::orientedAngle(upInner, normal, axleInner);
    innerTyre->setOrientationFromDirection(-steeringVector, glm::rotate(normal, -tyreAngle, steeringVector));
  }
  float angularSpeed = glm::length(w);
  const glm::vec3 angularImpulseInner = innerTyre->getAngularMomentum() - p1i;

  RigidBody* outerTyre = springs[outer].tyre;
  glm::vec3 axleOuter = getForwardVector(outerTyre);
  glm::vec3 upOuter = getUpVector(outerTyre);
  const glm::vec3 p1o = outerTyre->getAngularMomentum();
  const float angleOuter = glm::orientedAngle(axleOuter, steeringVectorOuter, normal);
  w = outerTyre->getAngularVelocity();
  direction = 1.0f;
  if(glm::dot(w, axleOuter) < 0.0f) direction = -1.0f;
  outerTyre->rotateGlobal(angleOuter * normal);
  if(std::abs(glm::dot(axleOuter, normal)) > epsilon) {
    float tyreAngle = glm::orientedAngle(upOuter, normal, axleOuter);
    outerTyre->setOrientationFromDirection(-steeringVectorOuter, glm::rotate(normal, -tyreAngle, steeringVectorOuter));
  }
  angularSpeed = glm::length(w);
  const glm::vec3 angularImpulseOuter = outerTyre->getAngularMomentum() - p1o;

  SuspensionConstraint* ci = static_cast<SuspensionConstraint*>(springs[inner].constraint);
  SuspensionConstraint* co = static_cast<SuspensionConstraint*>(springs[outer].constraint);
  ci->setRotationAxis(steeringVector);
  co->setRotationAxis(steeringVectorOuter);

  glm::vec3 directionVector = hull->getRotationMatrix() * glm::vec3(0.0f, 0.0f, -1.0f);
  glm::vec3 directionNormal = hull->getRotationMatrix() * glm::vec3(0.0f, 1.0f, 0.0f);
  RigidBody* tyre;
  if(input.accelerate){
    tyre = springs[0].tyre;
    applyAngularImpulseToTyre(tyre, drivingImpulse * dt);
    tyre = springs[2].tyre;
    applyAngularImpulseToTyre(tyre, -drivingImpulse * dt);
  }
  if(input.decelerate){
    tyre = springs[0].tyre;
    applyAngularImpulseToTyre(tyre, -brakingImpulse * dt);
    tyre = springs[2].tyre;
    applyAngularImpulseToTyre(tyre, brakingImpulse * dt);
  }
  for(int i = 0; i < springs.size(); i++){
    RigidBody* tyre = springs[i].tyre;
    float w = glm::length(tyre->getAngularVelocity());
    if(w > tyreMaxAngularVelocity){
      applyCounterRotationAngularImpulseToTyre(tyre, w - tyreMaxAngularVelocity);
    }
  }
}

void Vehicle::applyRollingResistance(){
  for(int i = 0; i < springs.size(); i++){
    RigidBody* tyre = springs[i].tyre;
    if(tyre->getTyreGroundCollision()){
      applyCounterRotationAngularImpulseToTyre(tyre, internalRollingFriction * glm::length(tyre->getAngularVelocity()) + rollingResistance * std::abs(tyre->getCollisionImpulseMagnitude()));
    }
  }
}

