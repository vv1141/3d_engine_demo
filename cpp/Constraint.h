#ifndef H_CONSTRAINT
#define H_CONSTRAINT

#include <glm/glm.hpp>

#include "RigidBody.h"

class Constraint {
public:

  static const int iterations;

protected:

  static const float biasFactor;

  RigidBody* a;
  RigidBody* b;
  glm::vec3 r1Local;
  glm::vec3 r2Local;

  glm::quat orientationFromDirection(glm::vec3 direction);
  glm::mat3 getSkewSymmetricMatrix(glm::vec3 v);

public:

  Constraint();
  virtual ~Constraint();

  virtual void setupConstants(float dt) = 0;
  virtual void applyConstraintImpulses(bool warmstart) = 0;

};

#endif
