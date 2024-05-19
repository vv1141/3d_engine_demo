#include "VehicleType.h"

const float standardVehicleLength = 2.0f;
const float standardTyreRadius = 0.125f;
const float VehicleType::hullMass = 1000.0f;
const float VehicleType::hullRotationalInertia = 1.0f/6.0f * VehicleType::hullMass * standardVehicleLength*standardVehicleLength;
const float VehicleType::tyreMass = 125.0f;
const float VehicleType::totalMass = hullMass + 4*tyreMass;
const float VehicleType::springConstant = 30.0f * VehicleType::totalMass;
const float VehicleType::springDampingCoefficient = 5.0f * VehicleType::totalMass;

VehicleType::VehicleType(){
}
VehicleType::~VehicleType(){
}

void VehicleType::setup(
  float hullLength,
  float hullHeight,
  float hullWidth,
  float hitboxHorizontalOffset,
  float hitboxVerticalOffset,
  float tyreRadius,
  float tyreWidth,
  float frontAxleHorizontalOffset,
  float rearAxleHorizontalOffset,
  float frontAxleVerticalOffset,
  float rearAxleVerticalOffset,
  float frontAxleWidth,
  float rearAxleWidth
){
  this->hullLength = hullLength;
  this->hullHeight = hullHeight;
  this->hullWidth = hullWidth;
  this->hitboxHorizontalOffset = hitboxHorizontalOffset;
  this->hitboxVerticalOffset = hitboxVerticalOffset;
  this->tyreRadius = tyreRadius;
  this->tyreWidth = tyreWidth;
  this->frontAxleHorizontalOffset = frontAxleHorizontalOffset;
  this->rearAxleHorizontalOffset = rearAxleHorizontalOffset;
  this->frontAxleVerticalOffset = frontAxleVerticalOffset;
  this->rearAxleVerticalOffset = rearAxleVerticalOffset;
  this->frontAxleWidth = frontAxleWidth;
  this->rearAxleWidth = rearAxleWidth;

  const float m = (1.0f/12.0f) * hullMass;
  this->hullRotationalInertiaTensor = glm::mat3(
    m*(hullLength*hullLength + hullHeight*hullHeight), 0.0f, 0.0f,
    0.0f, m*(hullWidth*hullWidth + hullLength*hullLength), 0.0f,
    0.0f, 0.0f, m*(hullWidth*hullWidth + hullHeight*hullHeight)
  );
  const float m1 = (1.0f/12.0f) * tyreMass;
  const float m2 = 0.5f * tyreMass;
  const float r1 = 0.5f * tyreRadius;
  const float r2 = tyreRadius;
  const float h = tyreWidth;
  const float xz = m1*(3*(r2*r2 + r1*r1) + h*h);
  const float y = m2*(r2*r2 + r1*r1);
  this->tyreRotationalInertiaTensor = glm::mat3(
    xz, 0.0f, 0.0f,
    0.0f, xz, 0.0f,
    0.0f, 0.0f, y
  );
  this->tyreRotationalInertiaAroundAxle = y;
  Collision::constructHitbox(&hullHitbox, glm::vec3(hullWidth * 0.5f, hullHeight * 0.5f, hullLength * 0.5f));
}
