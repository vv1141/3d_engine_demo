#ifndef H_SUSPENSIONCONSTRAINT
#define H_SUSPENSIONCONSTRAINT

#include "Constraint.h"

class SuspensionConstraint: public Constraint {
private:
  // slider axis in local space of a
  glm::vec3 sliderAxis;
  glm::vec2 limits;
  // rotation axes in local space
  glm::vec3 rotationAxisA;
  glm::vec3 rotationAxisB;

  // orientation difference in local space of a
  /* glm::quat orientationDifference; */
  float inverseMassSum;

  //derived variables
  float biasFactorDt;

public:
  SuspensionConstraint();
  /* SuspensionConstraint(RigidBody* a, RigidBody* b, glm::vec3 r1, glm::vec3 r2, glm::vec3 o1, glm::vec3 o2, glm::vec3 sliderAxis, glm::vec2 limits, glm::vec3 rotationAxis); // rotation axis in world space */
  SuspensionConstraint(RigidBody* a, RigidBody* b, glm::vec3 r1, glm::vec2 limits);
  ~SuspensionConstraint();

  void setRotationAxis(glm::vec3 rotationAxis);
  void setupConstants(float dt);
  void applyConstraintImpulses(bool warmstart);
};

#endif
