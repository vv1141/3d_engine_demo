#include "SuspensionConstraint.h"
#include "Debug.h"

SuspensionConstraint::SuspensionConstraint(){
}
SuspensionConstraint::SuspensionConstraint(RigidBody* a, RigidBody* b, glm::vec3 r1, glm::vec2 limits){
  this->a = a;
  this->b = b;
  this->r1Local = r1;
  this->r2Local = glm::vec3(0, 0, 0);
  this->sliderAxis = glm::vec3(0, 1, 0);
  this->limits = limits;
  glm::vec3 rotationAxis(1, 0, 0);
  this->rotationAxisA = a->getObject()->rotateVectorToLocalSpaceAndNormalize(rotationAxis);
  this->rotationAxisB = b->getObject()->rotateVectorToLocalSpaceAndNormalize(rotationAxis);
}
SuspensionConstraint::~SuspensionConstraint(){
}

void SuspensionConstraint::setRotationAxis(glm::vec3 rotationAxis){
  this->rotationAxisA = a->getObject()->rotateVectorToLocalSpaceAndNormalize(rotationAxis);
  this->rotationAxisB = b->getObject()->rotateVectorToLocalSpaceAndNormalize(rotationAxis);
}

void SuspensionConstraint::setupConstants(float dt){
  biasFactorDt = biasFactor / dt;
  inverseMassSum = a->getInverseMass() + b->getInverseMass();
}

void SuspensionConstraint::applyConstraintImpulses(bool warmstart){
  glm::vec3 r1 = a->getRotationMatrix() * r1Local;
  glm::vec3 r2 = b->getRotationMatrix() * r2Local;
  glm::vec3 x1 = a->getPosition();
  glm::vec3 v1 = a->getVelocity();
  glm::vec3 w1 = a->getAngularVelocity();
  glm::vec3 x2 = b->getPosition();
  glm::vec3 v2 = b->getVelocity();
  glm::vec3 w2 = b->getAngularVelocity();
  glm::mat3 I1 = a->getInverseInertiaTensorWorld();
  glm::mat3 I2 = b->getInverseInertiaTensorWorld();

  glm::vec3 axisWorld = a->getRotationMatrix() * sliderAxis;
  glm::vec3 u = x2 + r2 - x1 - r1;

  // limit constraints

  glm::vec3 r1ua = glm::cross(r1 + u, axisWorld);
  glm::vec3 r2a = glm::cross(r2, axisWorld);
  float d = glm::dot(u, axisWorld);
  float minLimitError = d - limits.x;
  float maxLimitError = limits.y - d;
  if(minLimitError <= 0.0f || maxLimitError <= 0.0f){
    float KMinInverse = inverseMassSum + glm::dot(r1ua, I1 * r1ua) + glm::dot(r2a, I2 * r2a);
    if(KMinInverse > 0.0f) KMinInverse = 1.0f / KMinInverse;
    float bMin = biasFactorDt * minLimitError;
    float bMax = biasFactorDt * maxLimitError;
    if(minLimitError <= 0.0f){

      // velocity constraints

      float JvMin = glm::dot(axisWorld, v2) + glm::dot(r2a, w2) - glm::dot(axisWorld, v1) - glm::dot(r1ua, w1);
      float lambdaMin = KMinInverse * (-JvMin - bMin);
      a->applyImpulse(-lambdaMin * axisWorld, x1);
      a->applyAngularImpulse(-lambdaMin * r1ua);
      b->applyImpulse(lambdaMin * axisWorld, x2);
      b->applyAngularImpulse(lambdaMin * r2a);

      // position constraints

      float lambdaMinError = KMinInverse * -minLimitError;
      glm::vec3 v1Pseudo = a->getInverseMass() * -lambdaMinError * axisWorld;
      glm::vec3 w1Pseudo = I1 * (-lambdaMinError * r1ua);
      a->setPosition(x1 + v1Pseudo);
      a->getObject()->rotateGlobal(w1Pseudo);
      glm::vec3 v2Pseudo = b->getInverseMass() * lambdaMinError * axisWorld;
      glm::vec3 w2Pseudo = I2 * (lambdaMinError * r2a);
      b->setPosition(x2 + v2Pseudo);
      b->getObject()->rotateGlobal(w2Pseudo);
    }
    if(maxLimitError <= 0.0f){

      // velocity constraints

      float JvMax = -glm::dot(axisWorld, v2) - glm::dot(r2a, w2) + glm::dot(axisWorld, v1) + glm::dot(r1ua, w1);
      float lambdaMax = KMinInverse * (-JvMax - bMax);
      a->applyImpulse(lambdaMax * axisWorld, x1);
      a->applyAngularImpulse(lambdaMax * r1ua);
      b->applyImpulse(-lambdaMax * axisWorld, x2);
      b->applyAngularImpulse(-lambdaMax * r2a);

      // position constraints

      float lambdaMaxError = KMinInverse * -maxLimitError;
      glm::vec3 v1Pseudo = a->getInverseMass() * lambdaMaxError * axisWorld;
      glm::vec3 w1Pseudo = I1 * (lambdaMaxError * r1ua);
      a->setPosition(x1 + v1Pseudo);
      a->getObject()->rotateGlobal(w1Pseudo);
      glm::vec3 v2Pseudo = b->getInverseMass() * -lambdaMaxError * axisWorld;
      glm::vec3 w2Pseudo = I2 * (-lambdaMaxError * r2a);
      b->setPosition(x2 + v2Pseudo);
      b->getObject()->rotateGlobal(w2Pseudo);
    }
  }

  // translation velocity constraint

  r1 = a->getRotationMatrix() * r1Local;
  r2 = b->getRotationMatrix() * r2Local;
  x1 = a->getPosition();
  v1 = a->getVelocity();
  w1 = a->getAngularVelocity();
  x2 = b->getPosition();
  v2 = b->getVelocity();
  w2 = b->getAngularVelocity();
  I1 = a->getInverseInertiaTensorWorld();
  I2 = b->getInverseInertiaTensorWorld();

  axisWorld = a->getRotationMatrix() * sliderAxis;
  u = x2 + r2 - x1 - r1;

  glm::vec3 n1 = Utility::getOneOrthogonalUnitVector(axisWorld);
  glm::vec3 n2 = glm::normalize(glm::cross(axisWorld, n1));
  glm::vec3 r2n1 = glm::cross(r2, n1);
  glm::vec3 r2n2 = glm::cross(r2, n2);

  glm::vec3 r1un1 = glm::cross(r1 + u, n1);
  glm::vec3 r1un2 = glm::cross(r1 + u, n2);
  glm::vec3 I1r1un1 = I1 * r1un1;
  glm::vec3 I1r1un2 = I1 * r1un2;
  glm::vec3 I2r2n1 = I2 * r2n1;
  glm::vec3 I2r2n2 = I2 * r2n2;
  glm::mat2 KTransInverse = glm::inverse(glm::mat2(
    inverseMassSum + glm::dot(r1un1, I1r1un1) + glm::dot(r2n1, I2r2n1),
    glm::dot(r1un1, I1r1un2) + glm::dot(r2n1, I2r2n2),
    glm::dot(r1un2, I1r1un1) + glm::dot(r2n2, I2r2n1),
    inverseMassSum + glm::dot(r1un2, I1r1un2) + glm::dot(r2n2, I2r2n2)
  ));
  glm::vec2 Jv(
    glm::dot(n1, v2) + glm::dot(w2, r2n1) - glm::dot(n1, v1) - glm::dot(w1, r1un1),
    glm::dot(n2, v2) + glm::dot(w2, r2n2) - glm::dot(n2, v1) - glm::dot(w1, r1un2)
  );
  glm::vec2 bTrans = biasFactorDt * glm::vec2(glm::dot(u, n1), glm::dot(u, n2));
  glm::vec2 lambdaDt = KTransInverse * (-Jv - bTrans);
  glm::vec3 impulseA = -n1 * lambdaDt.x - n2 * lambdaDt.y;
  glm::vec3 angularImpulseA = -r1un1 * lambdaDt.x - r1un2 * lambdaDt.y;
  glm::vec3 angularImpulseB = r2n1 * lambdaDt.x + r2n2 * lambdaDt.y;
  a->applyImpulse(impulseA, x1);
  a->applyAngularImpulse(angularImpulseA);
  b->applyImpulse(-impulseA, x2);
  b->applyAngularImpulse(angularImpulseB);

  // rotation velocity constraint for hinge joint

  glm::vec3 a1 = a->getRotationMatrix() * rotationAxisA;
  glm::vec3 a2 = b->getRotationMatrix() * rotationAxisB;
  glm::vec3 b2 = Utility::getOneOrthogonalUnitVector(a2);
  glm::vec3 c2 = glm::normalize(glm::cross(a2, b2));
  glm::vec3 b2a1 = glm::cross(b2, a1);
  glm::vec3 c2a1 = glm::cross(c2, a1);
  glm::vec3 I1b2a1 = I1 * b2a1;
  glm::vec3 I1c2a1 = I1 * c2a1;
  glm::vec3 I2b2a1 = I2 * b2a1;
  glm::vec3 I2c2a1 = I2 * c2a1;
  glm::mat2 KRotInverse = glm::inverse(glm::mat2(
    glm::dot(b2a1, I1b2a1) + glm::dot(b2a1, I2b2a1),
    glm::dot(c2a1, I1b2a1) + glm::dot(c2a1, I2b2a1),
    glm::dot(b2a1, I1c2a1) + glm::dot(b2a1, I2c2a1),
    glm::dot(c2a1, I1c2a1) + glm::dot(c2a1, I2c2a1)
  ));
  Jv = glm::vec2(
    -glm::dot(b2a1, w1) + glm::dot(b2a1, w2),
    -glm::dot(c2a1, w1) + glm::dot(c2a1, w2)
  );
  glm::vec2 errorRotation(glm::dot(a1, b2), glm::dot(a1, c2));
  glm::vec2 bRot = biasFactorDt * errorRotation;
  lambdaDt = KRotInverse * (-Jv - bRot);
  a->applyAngularImpulse(-b2a1 * lambdaDt.x - c2a1 * lambdaDt.y);
  b->applyAngularImpulse(b2a1 * lambdaDt.x + c2a1 * lambdaDt.y);

  // rotation position constraint

  glm::vec2 lambdaRotation = KRotInverse * -errorRotation;
  glm::vec3 w1Pseudo = I1 * (-b2a1 * lambdaRotation.x - c2a1 * lambdaRotation.y);
  glm::vec3 w2Pseudo = I2 * (b2a1 * lambdaRotation.x + c2a1 * lambdaRotation.y);
  a->getObject()->rotateGlobal(w1Pseudo);
  b->getObject()->rotateGlobal(w2Pseudo);

  // translation position constraint

  axisWorld = a->getRotationMatrix() * sliderAxis;
  u = x2 + r2 - x1 - r1;
  n1 = Utility::getOneOrthogonalUnitVector(axisWorld);
  n2 = glm::normalize(glm::cross(axisWorld, n1));
  r2n1 = glm::cross(r2, n1);
  r2n2 = glm::cross(r2, n2);
  r1un1 = glm::cross(r1 + u, n1);
  r1un2 = glm::cross(r1 + u, n2);
  I1r1un1 = I1 * r1un1;
  I1r1un2 = I1 * r1un2;
  I2r2n1 = I2 * r2n1;
  I2r2n2 = I2 * r2n2;
  KTransInverse = glm::inverse(glm::mat2(
    inverseMassSum + glm::dot(r1un1, I1r1un1) + glm::dot(r2n1, I2r2n1),
    glm::dot(r1un1, I1r1un2) + glm::dot(r2n1, I2r2n2),
    glm::dot(r1un2, I1r1un1) + glm::dot(r2n2, I2r2n1),
    inverseMassSum + glm::dot(r1un2, I1r1un2) + glm::dot(r2n2, I2r2n2)
  ));
  glm::vec2 errorTrans = glm::vec2(glm::dot(u, n1), glm::dot(u, n2));
  glm::vec2 lambdaTrans = KTransInverse * -errorTrans;
  impulseA = -n1 * lambdaTrans.x - n2 * lambdaTrans.y;
  angularImpulseA = -r1un1 * lambdaTrans.x - r1un2 * lambdaTrans.y;
  angularImpulseB = r2n1 * lambdaTrans.x + r2n2 * lambdaTrans.y;
  glm::vec3 v1Pseudo = a->getInverseMass() * impulseA;
  w1Pseudo = I1 * angularImpulseA;
  glm::vec3 v2Pseudo = b->getInverseMass() * -impulseA;
  w2Pseudo = I2 * angularImpulseB;
  a->setPosition(x1 + v1Pseudo);
  a->getObject()->rotateGlobal(w1Pseudo);
  b->setPosition(x2 + v2Pseudo);
  b->getObject()->rotateGlobal(w2Pseudo);
}
