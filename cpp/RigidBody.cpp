
#include "RigidBody.h"

#include "CollisionPair.h"
#include "Debug.h"

const float RigidBody::clampingCoefficient = 0.01f;
const float RigidBody::muStatic = 6.0f;
const float RigidBody::muDynamic = 5.0f;
const float RigidBody::muStaticSpin = 1.0f;
const float RigidBody::muDynamicSpin = 1.0f;
const float RigidBody::epsilon = 0.00001f;

RigidBody::RigidBody(){
  geometry = nullptr;
  identifier = Identifier::null;
  momentum = glm::vec3(0.0f, 0.0f, 0.0f);
  angularMomentum = glm::vec3(0.0f, 0.0f, 0.0f);
  inRest = false;
  tyreGroundCollision = false;
  relativeTyreGroundPointVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
  collisionImpulseMagnitude = 0.0f;
}

RigidBody::~RigidBody(){
}

void RigidBody::recalculateLinear(){
  object.setVelocity(inverseMass * momentum);
}

void RigidBody::recalculateAngular(){
  object.setAngularVelocity(inverseInertiaTensorWorld * angularMomentum);
}

void RigidBody::recalculateInertiaTensorWorld(){
  glm::mat3 rotation = getRotationMatrix();
  glm::mat3 rotationTranspose = glm::transpose(rotation);
  inertiaTensorWorld = rotation * inertiaTensor * rotationTranspose;
  inverseInertiaTensorWorld = rotation * inverseInertiaTensor * rotationTranspose;
}

float RigidBody::getVelocityDecay(glm::vec3 variable, float upperSpeed, float lowerSpeed, float minDecayFactor, float maxDecayFactor){
  float speedSquared = glm::length2(variable);
  if(speedSquared < 0.00001f){
    return 0.0f;
  }
  const float upperSpeedSquared = upperSpeed * upperSpeed;
  const float lowerSpeedSquared = lowerSpeed * lowerSpeed;
  float decayAlpha;
  if(speedSquared > upperSpeedSquared){
    decayAlpha = 1.0f;
  }
  else if(speedSquared < lowerSpeedSquared){
    decayAlpha = 0.0f;
  }
  else{
    decayAlpha = (speedSquared - lowerSpeedSquared) / (upperSpeedSquared - lowerSpeedSquared);
  }
  return (maxDecayFactor - minDecayFactor) * (1.0f - decayAlpha) + minDecayFactor;
}

bool RigidBody::isStableContactManifold(Collision::ContactManifold* contactManifold, glm::vec3 gravityVector){
  if(geometry->type == Geometry::Type::sphere){
    if(contactManifold->points.size() >= 1){
      float dp = glm::dot(contactManifold->normal, gravityVector);
      if(dp*dp - glm::length2(gravityVector) > -epsilon){
        return true;
      }
    }
  }
  else {
    if(contactManifold->points.size() == 4) {
      glm::vec3 triangleVertices0[3] = {contactManifold->points[0].position, contactManifold->points[1].position, contactManifold->points[2].position};
      if(!Utility::triangleHasArea(triangleVertices0, contactManifold->normal)) return false;
      glm::vec3 triangleVertices1[3] = {contactManifold->points[1].position, contactManifold->points[2].position, contactManifold->points[3].position};
      if(!Utility::triangleHasArea(triangleVertices1, contactManifold->normal)) return false;
      glm::vec3 triangleVertices2[3] = {contactManifold->points[0].position, contactManifold->points[2].position, contactManifold->points[3].position};
      if(!Utility::triangleHasArea(triangleVertices2, contactManifold->normal)) return false;
      glm::vec3 triangleVertices3[3] = {contactManifold->points[0].position, contactManifold->points[1].position, contactManifold->points[3].position};
      if(!Utility::triangleHasArea(triangleVertices3, contactManifold->normal)) return false;

      glm::vec3 intersectionPoint = Collision::linePlaneIntersection(object.getPosition(), object.getPosition() + gravityVector, contactManifold->normal, contactManifold->points[0].position);

      glm::vec3 manifoldCenter = (contactManifold->points[0].position + contactManifold->points[1].position + contactManifold->points[2].position + contactManifold->points[3].position) / 4.0f;
      if(Utility::onDistance(manifoldCenter, intersectionPoint, 0.01f)){
        return true;
      }
      if(Utility::pointInsideTriangle(intersectionPoint, triangleVertices0, contactManifold->normal) ||
         Utility::pointInsideTriangle(intersectionPoint, triangleVertices1, contactManifold->normal) ||
         Utility::pointInsideTriangle(intersectionPoint, triangleVertices2, contactManifold->normal) ||
         Utility::pointInsideTriangle(intersectionPoint, triangleVertices3, contactManifold->normal)){
        return true;
      }
    }
  }
  return false;
}

bool RigidBody::isStationaryOnSlope(glm::vec3 gravity, glm::vec3 normal){
  float gN = glm::dot(gravity, normal);
  return (glm::length(gravity) - gN < muDynamic * gN);
}

void RigidBody::setMassAndInertiaTensor(float mass, glm::mat3 inertiaTensor){
  this->mass = mass;
  this->inverseMass = 1.0f / mass;
  this->inertiaTensor = inertiaTensor;
  this->inverseInertiaTensor = glm::inverse(inertiaTensor);
  recalculateInertiaTensorWorld();
  recalculateLinear();
  recalculateAngular();
}

void RigidBody::setStatic(){
  this->object.setStatic(true);
  this->mass = -1.0f;
  this->inverseMass = 0.0f;
  this->inertiaTensor = glm::mat3(-1.0f);
  this->inverseInertiaTensor = glm::mat3(0.0f);
  recalculateInertiaTensorWorld();
  recalculateLinear();
  recalculateAngular();
}

bool RigidBody::isStatic(){
  return (inverseMass == 0.0f);
}

float RigidBody::getMass(){
  return mass;
}

float RigidBody::getInverseMass(){
  return inverseMass;
}

glm::mat3 RigidBody::getInverseInertiaTensorWorld(){
  return inverseInertiaTensorWorld;
}

void RigidBody::setGeometryBox(Collision::Polyhedron* collisionPolyhedron){
  geometry = std::make_unique<Box>();
  Box* box = static_cast<Box*>(geometry.get());
  box->setup(collisionPolyhedron);
}

void RigidBody::setGeometryCylinder(Collision::Polyhedron* collisionPolyhedron){
  geometry = std::make_unique<Cylinder>();
  Cylinder* cylinder = static_cast<Cylinder*>(geometry.get());
  cylinder->setup(collisionPolyhedron->maxHalfWidth.x, collisionPolyhedron->maxHalfWidth.y, collisionPolyhedron);
}

void RigidBody::setGeometrySphere(float radius, float fakeTyreWidth, Object* fakeTyreParent){
  geometry = std::make_unique<Sphere>();
  Sphere* sphere = static_cast<Sphere*>(geometry.get());
  sphere->setup(radius, fakeTyreWidth, fakeTyreParent);
}

void RigidBody::setGeometryPrimitiveGroup(){
  geometry = std::make_unique<PrimitiveGroup>();
}

void RigidBody::setGeometryMesh(int sectorCount, std::vector<Collision::Polyhedron>* collisionMesh){
  geometry = std::make_unique<Mesh>();
  Mesh* mesh = static_cast<Mesh*>(geometry.get());
  mesh->setup(sectorCount, collisionMesh);
}

Geometry* RigidBody::getGeometry(){
  return geometry.get();
}

void RigidBody::setIdentifier(Identifier identifier){
  this->identifier = identifier;
}

RigidBody::Identifier RigidBody::getIdentifier(){
  return identifier;
}

void RigidBody::addContactManifold(Collision::ContactManifold contactManifold){
  contactManifolds.emplace_back(contactManifold);
}

void RigidBody::clearContactManifolds(){
  contactManifolds.clear();
  supportManifold = SupportManifold{glm::vec3(0.0f, 0.0f, 0.0f), false};
}

std::vector<Collision::ContactManifold>* RigidBody::getContactManifolds(){
  return &contactManifolds;
}

void RigidBody::determineSupportManifold(glm::vec3 gravityVector){
  supportManifold = SupportManifold{glm::vec3(0.0f, 0.0f, 0.0f), false};
  std::vector<Collision::ContactManifold> joinableContactManifolds;
  for(int i = 0; i < contactManifolds.size(); i++){
    float dp = glm::dot(contactManifolds[i].normal, gravityVector);
    if(dp > 0.0f) joinableContactManifolds.emplace_back(contactManifolds[i]);
  }
  if(joinableContactManifolds.size() == 1){
    if(isStableContactManifold(&(joinableContactManifolds[0]), gravityVector)){
      supportManifold = SupportManifold{joinableContactManifolds[0].normal, true};
    }
  } else if(joinableContactManifolds.size() > 1){
    Collision::ContactManifold reducedManifold;
    if(Collision::joinContactManifolds(&joinableContactManifolds, &reducedManifold)){
      if(isStableContactManifold(&(reducedManifold), gravityVector)){
        supportManifold = SupportManifold{reducedManifold.normal, true};
      }
    }
  }
}

RigidBody::SupportManifold* RigidBody::getSupportManifold(){
  return &supportManifold;
}

bool RigidBody::isInRest(){
  return inRest;
}

void RigidBody::setPosition(glm::vec3 position){
  object.setPosition(position);
}

glm::vec3 RigidBody::getPosition(){
  return object.getPosition();
}

glm::vec3 RigidBody::getPosition(RigidBody* other){
  if(geometry->type == Geometry::Type::sphere && other != nullptr){
    Sphere* sphere = static_cast<Sphere*>(geometry.get());
    if(sphere->offset > 0.0f){
      glm::vec3 normal = sphere->fakeTyreParent->getRotationMatrix() * glm::vec3(0.0f, 1.0f, 0.0f);
      return object.getPosition() + sphere->offset * -normal;
    }
  }
  return object.getPosition();
}

void RigidBody::setVelocity(glm::vec3 velocity){
  momentum = mass * velocity;
  recalculateLinear();
}

glm::vec3 RigidBody::getVelocity(){
  return object.getVelocity();
}

glm::vec3 RigidBody::getPointVelocity(glm::vec3 point){
  return getVelocity() + glm::cross(getAngularVelocity(), point - getPosition());
}

void RigidBody::setOrientation(glm::quat orientation){
  object.setOrientation(orientation);
  recalculateInertiaTensorWorld();
}

void RigidBody::setOrientationFromDirection(glm::vec3 direction){
  object.setOrientationFromDirection(direction);
  recalculateInertiaTensorWorld();
}

void RigidBody::setOrientationFromDirection(glm::vec3 direction, glm::vec3 up){
  object.setOrientationFromDirection(direction, up);
  recalculateInertiaTensorWorld();
}

void RigidBody::rotate(glm::vec3 magnitude){
  object.rotate(magnitude);
  recalculateInertiaTensorWorld();
}

void RigidBody::rotateGlobal(glm::vec3 magnitude){
  object.rotateGlobal(magnitude);
  recalculateInertiaTensorWorld();
}

glm::quat RigidBody::getOrientation(){
  return object.getOrientation();
}

void RigidBody::setAngularVelocity(glm::vec3 angularVelocity){
  angularMomentum = inertiaTensorWorld * angularVelocity;
  recalculateAngular();
}

glm::vec3 RigidBody::getAngularVelocity(){
  return object.getAngularVelocity();
}

glm::vec3 RigidBody::getAngularMomentum(){
  return angularMomentum;
}

glm::mat4 RigidBody::getModelMatrix(){
  return object.getModelMatrix();
}

glm::mat4 RigidBody::getModelMatrixInverse(){
  return object.getModelMatrixInverse();
}

glm::mat3 RigidBody::getRotationMatrix(){
  return object.getRotationMatrix();
}

Object* RigidBody::getObject(){
  return &object;
}

void RigidBody::setFrameVariables(){
  tyreGroundCollision = false;
  relativeTyreGroundPointVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
  collisionImpulseMagnitude = 0.0f;
}

bool RigidBody::getTyreGroundCollision(){
  return tyreGroundCollision;
}

float RigidBody::getCollisionImpulseMagnitude(){
  return collisionImpulseMagnitude;
}

glm::vec3 RigidBody::getRelativeTyreGroundPointVelocity(){
  return relativeTyreGroundPointVelocity;
}

void RigidBody::applyImpulse(glm::vec3 impulse, glm::vec3 point){
  if(inverseMass > 0.0f){
    momentum += impulse;
    glm::vec3 angularImpulse = glm::cross(point - getPosition(), impulse);
    angularMomentum += angularImpulse;
    recalculateLinear();
    recalculateAngular();
  }
}

void RigidBody::applyGravity(glm::vec3 g, float dt){
  if(inverseMass > 0.0f){
    if(identifier == Identifier::hull || identifier == Identifier::tyre){
      applyImpulse(mass * g * dt, getPosition());
      return;
    }
    if(supportManifold.isStable && isStationaryOnSlope(g, supportManifold.normal)){
      momentum *= getVelocityDecay(getVelocity(), 2.0f, 0.5f, 0.999f, 0.995f);
      angularMomentum *= getVelocityDecay(getAngularVelocity(), 2.0f, 0.5f, 0.999f, 0.995f);
      recalculateLinear();
      recalculateAngular();
    }
    else{
      applyImpulse(mass * g * dt, getPosition());
    }
  }
}

void RigidBody::applyAngularImpulse(glm::vec3 angularImpulse){
  angularMomentum += angularImpulse;
  recalculateAngular();
}

void RigidBody::applyCollisionResponse(RigidBody* a, RigidBody* b, Collision::ContactManifold manifold){
  if(manifold.points.size() > 0){

    std::vector<Collision::ContactPoint> collidingPoints;
    std::vector<Impulse>                 impulses;
    float                                totalSeparation = 0.0f;
    float                                totalJ = 0.0f;

    glm::vec3 n = manifold.normal;

    for(int i = 0; i < manifold.points.size(); i++){
      glm::vec3 p = manifold.points[i].position;
      glm::vec3 vr = b->getPointVelocity(p) - a->getPointVelocity(p);
      if(glm::dot(vr, n) < 0.0f){
        collidingPoints.emplace_back(manifold.points[i]);
        totalSeparation += manifold.points[i].separation;
      }
    }

    if(collidingPoints.size() > 0){
      if(totalSeparation < 0.0f){

        bool isSphereA = (a->getGeometry()->type == Geometry::Type::sphere);
        bool isSphereB = (b->getGeometry()->type == Geometry::Type::sphere);

        for(int i = 0; i < collidingPoints.size(); i++){
          glm::vec3 p = collidingPoints[i].position;
          glm::vec3 vr = b->getPointVelocity(p) - a->getPointVelocity(p);
          float e = 0.0f;
          glm::vec3 ra = p - a->getPosition();
          glm::vec3 rb = p - b->getPosition();
          float denominator = (a->inverseMass + b->inverseMass + glm::dot(glm::cross(a->inverseInertiaTensorWorld * glm::cross(ra, n), ra) + glm::cross(b->inverseInertiaTensorWorld * glm::cross(rb, n), rb), n));
          if(!isnan(denominator)){
            float j = glm::dot(-(1.0f + e) * vr, n) / denominator;
            float relativeSeparation = 0.0f;
            if(totalSeparation < epsilon) {
              relativeSeparation = (1.0f / (float)collidingPoints.size());
            } else {
              relativeSeparation = collidingPoints[i].separation / totalSeparation;
            }
            if(a->inverseMass == 0.0){
              if(!isSphereB){
                j = glm::clamp(j, 0.0f, b->mass * clampingCoefficient);
              }
            }
            if(b->inverseMass == 0.0){
              if(!isSphereA){
                j = glm::clamp(j, 0.0f, a->mass * clampingCoefficient);
              }
            }
            j *= relativeSeparation;
            impulses.emplace_back(Impulse{j * n, p});
            totalJ += j;
          }
        }

        for(int i = 0; i < impulses.size(); i++){
          a->applyImpulse(-impulses[i].impulse, impulses[i].point);
          b->applyImpulse(impulses[i].impulse, impulses[i].point);
        }
        glm::vec3 averagePoint = glm::vec3(0.0f, 0.0f, 0.0f);
        for(int i = 0; i < collidingPoints.size(); i++){
          glm::vec3 p = collidingPoints[i].position;
          averagePoint += p;
        }
        averagePoint *= (1.0f / (float)collidingPoints.size());

        // Coulomb friction
        const float staticFriction = muStatic * totalJ;
        const float dynamicFriction = muDynamic * totalJ;
        const float staticFrictionSpin = muStaticSpin * totalJ;
        const float dynamicFrictionSpin = muDynamicSpin * totalJ;
        glm::vec3 vr = b->getPointVelocity(averagePoint) - a->getPointVelocity(averagePoint);
        if((a->identifier == Identifier::tyre &&
           b->identifier == Identifier::ground) ||
           (a->identifier == Identifier::ground &&
           b->identifier == Identifier::tyre)){
          a->tyreGroundCollision = true;
          b->tyreGroundCollision = true;
          a->relativeTyreGroundPointVelocity = vr;
          b->relativeTyreGroundPointVelocity = -vr;
          a->collisionImpulseMagnitude += totalJ;
          b->collisionImpulseMagnitude += totalJ;
        }
        glm::vec3 t = vr - glm::dot(vr, n) * n;
        t = glm::normalize(t);
        if(!Utility::isNan(t)){
          float tvr = glm::dot(vr, t);
          float jt = tvr / (a->inverseMass + b->inverseMass);
          glm::vec3 frictionImpulse = glm::vec3(0.0f, 0.0f, 0.0f);
          if(jt < staticFriction){
            frictionImpulse = -jt * t;
          }
          else{
            frictionImpulse = -dynamicFriction * t;
          }
          a->applyImpulse(-frictionImpulse, averagePoint);
          b->applyImpulse(frictionImpulse, averagePoint);
        }

        // spin friction
        glm::vec3 wr = b->getAngularVelocity() - a->getAngularVelocity();
        glm::vec3 nwr = glm::dot(wr, n) * n;
        glm::vec3 jw = glm::vec3(0.0f);
        if(a->inverseMass > 0.0f && b->inverseMass == 0.0f) jw = a->inertiaTensorWorld * nwr;
        if(b->inverseMass > 0.0f && a->inverseMass == 0.0f) jw = b->inertiaTensorWorld * nwr;
        glm::vec3 angularFrictionImpulse = glm::vec3(0.0f);
        if(glm::length2(jw) < staticFrictionSpin*staticFrictionSpin){
          angularFrictionImpulse = -jw;
        }
        else {
          float direction = 1.0f;
          if(glm::dot(wr, n) < 0.0f) direction = -1.0f;
          angularFrictionImpulse = direction * -dynamicFrictionSpin * n;
        }
        a->applyAngularImpulse(-angularFrictionImpulse);
        b->applyAngularImpulse(angularFrictionImpulse);
      }
    }

    float minSeparation = manifold.points[0].separation;
    for(int i = 1; i < manifold.points.size(); i++){
      if(manifold.points[i].separation < minSeparation){
        minSeparation = manifold.points[i].separation;
      }
    }

    const float fraction = 0.2f;
    const float slop = 0.001f;
    glm::vec3 correction = (std::max(-minSeparation - slop, 0.0f) / (a->inverseMass + b->inverseMass)) * fraction * n;
    a->setPosition(a->getPosition() - a->inverseMass * correction);
    b->setPosition(b->getPosition() + b->inverseMass * correction);
  }
}

void RigidBody::applyRestDetection(){
  const float restVelocityThreshold = 0.01f;
  const float restAngularVelocityThreshold = 0.01f;
  if(identifier == Identifier::hull || identifier == Identifier::tyre){
    inRest = false;
    return;
  }
  if(supportManifold.isStable &&
     glm::length2(getVelocity()) < restVelocityThreshold * restVelocityThreshold &&
     glm::length2(getAngularVelocity()) < restAngularVelocityThreshold * restAngularVelocityThreshold){
    inRest = true;
    momentum = glm::vec3(0.0f, 0.0f, 0.0f);
    angularMomentum = glm::vec3(0.0f, 0.0f, 0.0f);
    recalculateLinear();
    recalculateAngular();
  } else {
    inRest = false;
  }
}

void RigidBody::integrate(float dt){
  if(!inRest){
    object.integrate(dt);
    recalculateInertiaTensorWorld();
    recalculateAngular();
  }
}
