#ifndef H_VEHICLETYPE
#define H_VEHICLETYPE

#include "Collision.h"
#include "Object.h"

class VehicleType {
private:

public:

  static const float hullMass;
  static const float hullRotationalInertia;
  static const float rotationalInertia;
  static const float tyreMass;
  static const float totalMass;
  static const float tyreRotationalInertia;
  static const float springConstant;
  static const float springDampingCoefficient;

  float hullLength;
  float hullHeight;
  float hullWidth;
  float hitboxHorizontalOffset;
  float hitboxVerticalOffset;
  float tyreRadius;
  float tyreWidth;
  float frontAxleHorizontalOffset;
  float rearAxleHorizontalOffset;
  float frontAxleVerticalOffset;
  float rearAxleVerticalOffset;
  float frontAxleWidth;
  float rearAxleWidth;
  Collision::Polyhedron hullHitbox;

  glm::mat3 hullRotationalInertiaTensor;
  glm::mat3 tyreRotationalInertiaTensor;
  float tyreRotationalInertiaAroundAxle;

  VehicleType();
  ~VehicleType();

  void setup(
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
  );
};

#endif
