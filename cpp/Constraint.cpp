#include "Constraint.h"

const int Constraint::iterations = 5;
const float Constraint::biasFactor = 0.2f;

glm::quat Constraint::orientationFromDirection(glm::vec3 direction){
  return glm::quatLookAt(glm::normalize(direction), glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f)));
}

glm::mat3 Constraint::getSkewSymmetricMatrix(glm::vec3 v){
  return glm::mat3(
    0.0f, v.z, -v.y,
    -v.z, 0.0f, v.x,
    v.y, -v.x, 0.0f
  );
}

Constraint::Constraint(){
}
Constraint::~Constraint(){
}
