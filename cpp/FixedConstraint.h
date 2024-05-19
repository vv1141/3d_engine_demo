#ifndef H_FIXEDCONSTRAINT
#define H_FIXEDCONSTRAINT

#include "Constraint.h"

class FixedConstraint : public Constraint {
private:

  glm::quat orientationDifference;
  float biasFactorDt;

public:

  FixedConstraint();
  FixedConstraint(RigidBody* a, RigidBody* b, glm::vec3 r1, glm::vec3 r2, glm::vec3 o1, glm::vec3 o2);
  ~FixedConstraint();

  void setupConstants(float dt);
  void applyConstraintImpulses(bool warmstart);

};

#endif
