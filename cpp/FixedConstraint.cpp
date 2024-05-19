#include "FixedConstraint.h"
#include "Debug.h"

FixedConstraint::FixedConstraint(){
}
FixedConstraint::FixedConstraint(RigidBody* a, RigidBody* b, glm::vec3 r1, glm::vec3 r2, glm::vec3 o1, glm::vec3 o2){
  this->a = a;
  this->b = b;
  this->r1Local = r1;
  this->r2Local = r2;
  this->orientationDifference = glm::conjugate(orientationFromDirection(o2)) * orientationFromDirection(o1);
}
FixedConstraint::~FixedConstraint(){
}

void FixedConstraint::setupConstants(float dt){
  biasFactorDt = biasFactor / dt;
}

void FixedConstraint::applyConstraintImpulses(bool warmstart){
  glm::vec3 r1 = a->getRotationMatrix() * r1Local;
  glm::vec3 r2 = b->getRotationMatrix() * r2Local;
  glm::mat3 r1x = getSkewSymmetricMatrix(r1);
  glm::mat3 r2x = getSkewSymmetricMatrix(r2);
  glm::vec3 x1 = a->getPosition();
  glm::vec3 v1 = a->getVelocity();
  glm::vec3 w1 = a->getAngularVelocity();
  glm::vec3 x2 = b->getPosition();
  glm::vec3 v2 = b->getVelocity();
  glm::vec3 w2 = b->getAngularVelocity();
  glm::mat3 I1 = a->getInverseInertiaTensorWorld();
  glm::mat3 I2 = b->getInverseInertiaTensorWorld();
  const float inverseMassSum = a->getInverseMass() + b->getInverseMass();
  glm::mat3 KTransInverse = glm::inverse(
    (r1x * I1 * glm::transpose(r1x)) +
    (r2x * I2 * glm::transpose(r2x)) +
    glm::mat3(
      inverseMassSum, 0.0f, 0.0f,
      0.0f, inverseMassSum, 0.0f,
      0.0f, 0.0f, inverseMassSum
    )
  );
  glm::quat orientationError = b->getOrientation() * this->orientationDifference * glm::conjugate(a->getOrientation());
  glm::vec3 bRot = 2.0f * biasFactorDt * glm::vec3(orientationError.x, orientationError.y, orientationError.z);
  glm::mat3 KRotInverse = glm::inverse(I1 + I2);

  // translation velocity constraint

  glm::vec3 bTrans = biasFactorDt * (x2 + r2 - x1 - r1);
  const glm::vec3 Jv = v2 + glm::cross(w2, r2) - v1 - glm::cross(w1, r1);
  glm::vec3 lambdaDt = KTransInverse * (-Jv - bTrans); // assume F_ext is 0, and it follows that v == v'
  a->applyImpulse(-lambdaDt, x1+r1);
  b->applyImpulse(lambdaDt, x2+r2);

  // rotation velocity constraint

  w1 = a->getAngularVelocity();
  w2 = b->getAngularVelocity();
  const glm::vec3 Jw = w2 - w1;
  lambdaDt = KRotInverse * (-Jw - bRot);
  a->applyAngularImpulse(-lambdaDt);
  b->applyAngularImpulse(lambdaDt);

  // translation position constraint

  glm::vec3 errorTrans = x2 + r2 - x1 - r1;
  glm::vec3 lambdaTrans = KTransInverse * -errorTrans;
  glm::vec3 v1Pseudo = a->getInverseMass() * -lambdaTrans;
  glm::vec3 w1Pseudo = I1 * glm::cross(lambdaTrans, r1);
  glm::vec3 v2Pseudo = b->getInverseMass() * lambdaTrans;
  glm::vec3 w2Pseudo = I2 * glm::cross(-lambdaTrans, r2);
  a->setPosition(x1 + v1Pseudo);
  a->getObject()->rotateGlobal(w1Pseudo);
  b->setPosition(x2 + v2Pseudo);
  b->getObject()->rotateGlobal(w2Pseudo);

  // rotation position constraint

  I1 = a->getInverseInertiaTensorWorld();
  I2 = b->getInverseInertiaTensorWorld();
  orientationError = b->getOrientation() * this->orientationDifference * glm::conjugate(a->getOrientation());
  // q = [sin(theta / 2) * v, cos(theta/2)]
  // assume theta is small (error is small) -> sin(x) = x
  glm::vec3 errorRotation = 2.0f * glm::vec3(orientationError.x, orientationError.y, orientationError.z);
  KRotInverse = glm::inverse(I1 + I2);
  glm::vec3 lambdaRot = KRotInverse * -errorRotation;
  w1Pseudo = I1 * -lambdaRot;
  w2Pseudo = I2 * lambdaRot;
  a->getObject()->rotateGlobal(w1Pseudo);
  b->getObject()->rotateGlobal(w2Pseudo);
}
