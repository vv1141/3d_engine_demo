
#include "World.h"

void World::addRenderObject(Model* model, Texture* texture, Object* object) {
  renderObjects.emplace_back(&modelTexturePairs, model, texture, object);
  RenderObject::sortRenderObjects(&renderObjects);
}

bool World::writeStateToFile(std::string path) {
  unsigned int size = rigidBodies.size() * (sizeof(rigidBodies.front().getPosition()) +
                                            sizeof(rigidBodies.front().getOrientation()) +
                                            sizeof(rigidBodies.front().getVelocity()) +
                                            sizeof(rigidBodies.front().getAngularVelocity()));
  char*        memBlock = new char[size];
  char*        memPointer = memBlock;

  for(auto it = rigidBodies.begin(); it != rigidBodies.end(); ++it) {
    Utility::writeValue(&memPointer, it->getPosition());
    Utility::writeValue(&memPointer, it->getOrientation());
    Utility::writeValue(&memPointer, it->getVelocity());
    Utility::writeValue(&memPointer, it->getAngularVelocity());
  }

  std::ofstream file(path, std::ios::out | std::ios::binary | std::ios::trunc);
  if(file.is_open()) {
    file.write(memBlock, size);
    file.close();
    delete[] memBlock;
  } else {
    std::cout << "error writing state: " + path << std::endl;
    return false;
  }
  std::cout << "state write to file: " + path << std::endl;
  return true;
}

bool World::readStateFromFile(std::string path) {
  std::streampos size;
  char*          memBlock;
  std::ifstream  file(path, std::ios::in | std::ios::binary | std::ios::ate);
  if(file.is_open()) {
    size = file.tellg();
    memBlock = new char[size];
    file.seekg(0, std::ios::beg);
    file.read(memBlock, size);
    file.close();
    char* memPointer = memBlock;

    glm::vec3 position;
    glm::quat orientation;
    glm::vec3 velocity;
    glm::vec3 angularVelocity;
    for(auto it = rigidBodies.begin(); it != rigidBodies.end(); ++it) {
      Utility::readValue(&memPointer, &position);
      Utility::readValue(&memPointer, &orientation);
      Utility::readValue(&memPointer, &velocity);
      Utility::readValue(&memPointer, &angularVelocity);
      it->setPosition(position);
      it->setOrientation(orientation);
      it->setVelocity(velocity);
      it->setAngularVelocity(angularVelocity);
    }

    delete[] memBlock;
  } else {
    std::cout << "error reading state: " + path << std::endl;
    return false;
  }
  std::cout << "state read from file: " + path << std::endl;
  return true;
}

bool World::readAssetFile(std::string path, bool readShaders) {
  char* memBlock;
  if(Utility::readFile(path, &memBlock, nullptr)) {
    char* memPointer = memBlock;

    if(!Utility::readFontFromMemoryBlock(&memPointer)) {
      delete[] memBlock;
      return false;
    }
    Utility::initTextRendering();
    std::vector<std::string> modelNames;
    modelNames.emplace_back("hull");
    modelNames.emplace_back("hull2");
    modelNames.emplace_back("tyre");
    modelNames.emplace_back("terrain");
    for(auto& name: modelNames) {
      models[name] = Model();
      textures[name] = Texture();
    }
    for(auto& name: modelNames) {
      models[name].readModelFromMemoryBlock(&memPointer);
      models[name].bufferData();
      if(!textures[name].loadTextures(&memPointer)) {
        delete[] memBlock;
        return false;
      }
    }
    Model::readCollisionMeshFromMemoryBlock(&terrainMesh, &memPointer);
    if(readShaders) {
      opaqueShader.loadShaders(&memPointer);
      shadowMappingShader.loadShaders(&memPointer);
      screenShader.loadShaders(&memPointer);
    }

    delete[] memBlock;
    return true;
  }
  return false;
}

World::World() {
}

World::~World() {
  glDeleteVertexArrays(1, &vertexArray);
}

bool World::init(sf::RenderWindow* renderWindow, Input* input) {
  this->renderWindow = renderWindow;
  this->input = input;
  Utility::initRenderState(renderWindow, &projectionViewMatrix);

  glGenVertexArrays(1, &vertexArray);

  depthMapResolution = 4096;
  shadowLevelCount = 5;
  zMult = 10.0f;
  zBias = 5.0f;
  fov = 70.0f;
  nearClippingPlane = 0.1f;
  farClippingPlane = 175.0f;
  multisamplingEnabled = true;
  multisamplingSampleCount = 4;

  bool useAssetFileShaders = true;
  if(!readAssetFile("res/assetdata", useAssetFileShaders)) return false;
  if(!useAssetFileShaders) {
    opaqueShader.loadShaders("SimpleShader.vert", "SimpleShader.frag");
    shadowMappingShader.loadShaders("ShadowMappingShader.vert", "ShadowMappingShader.frag");
    screenShader.loadShaders("ScreenShader.vert", "ScreenShader.frag");
  }
  opaqueShader.setup(shadowLevelCount);
  shadowMappingShader.setup(depthMapResolution, shadowLevelCount);
  glm::ivec2 windowSize((int)renderWindow->getSize().x, (int)renderWindow->getSize().y);
  screenShader.setup(windowSize, multisamplingEnabled, multisamplingSampleCount);

  isPaused = false;
  showControls = true;
  stateFile = "res/state";

  gravity = glm::vec3(0.0f, -9.81f, 0.0f);

  float vehicleCameraDistance = 7.0f;
  float vehicleCameraHeight = 3.0f;
  float vehicleCameraAngle = 1.0f;
  float orbitCameraInitialDistance = 5.0f;
  float orbitCameraMinDistance = 1.0f;
  float orbitCameraMaxDistance = 2500.0f;
  float orbitCameraMinAngleVertical = -1.4f;
  float orbitCameraMaxAngleVertical = 1.4f;

  float collisionBoxLength = 3.8f;
  float collisionBoxHeight = 1.16f;
  float collisionBoxWidth = 1.6f;
  Collision::constructHitbox(&cubeHitbox, glm::vec3(collisionBoxWidth * 0.5f, collisionBoxHeight * 0.5f, collisionBoxLength * 0.5f));
  const float mass = 100.0f;
  const float cubeRotationalInertia = 1.0f / 6.0f * mass * collisionBoxLength * collisionBoxLength;

  rigidBodies.emplace_back();
  rigidBodies.back().setGeometryMesh(24, &terrainMesh);
  rigidBodies.back().setIdentifier(RigidBody::Identifier::ground);
  rigidBodies.back().setStatic();
  rigidBodies.back().setPosition(glm::vec3(0, 0, 0));
  rigidBodies.back().getGeometry()->transformToWorldSpace(rigidBodies.back().getModelMatrix());
  addRenderObject(&(models["terrain"]), &(textures["terrain"]), rigidBodies.back().getObject());

  for(int i = 0; i < 2; i++) {
    for(int j = 0; j < 2; j++) {
      rigidBodies.emplace_back();
      rigidBodies.back().setMassAndInertiaTensor(mass, glm::mat3(cubeRotationalInertia));
      rigidBodies.back().setPosition(glm::vec3(
        Utility::randomDouble(0, 10),
        Utility::randomDouble(0, 10) + 10,
        Utility::randomDouble(0, 10)));
      rigidBodies.back().setGeometryBox(&cubeHitbox);
      addRenderObject(&(models["hull2"]), &(textures["hull2"]), rigidBodies.back().getObject());
      cubePointer = &rigidBodies.back();
      auto end = rigidBodies.end();
      std::advance(end, -1);
      for(auto it = rigidBodies.begin(); it != end; ++it) {
        RigidBody* other = &(*it);
        collisionPairs.emplace_back(cubePointer, other);
      }
    }
  }
  for(int i = 0; i < 2; i++) {
    for(int j = 0; j < 2; j++) {
      float r = Utility::randomDouble(0.5, 0.5);
      rigidBodies.emplace_back();
      rigidBodies.back().setMassAndInertiaTensor(mass, glm::mat3(cubeRotationalInertia));
      rigidBodies.back().setPosition(glm::vec3(-4, 5 + (i * 6) + j * 3, 2));
      rigidBodies.back().setGeometryBox(&cubeHitbox);
      addRenderObject(&(models["hull2"]), &(textures["hull2"]), rigidBodies.back().getObject());
      cubePointer = &rigidBodies.back();
      auto end = rigidBodies.end();
      std::advance(end, -1);
      for(auto it = rigidBodies.begin(); it != end; ++it) {
        RigidBody* other = &(*it);
        collisionPairs.emplace_back(cubePointer, other);
      }
    }
  }

  vehicleType.setup(
    3.8f,
    0.16f,
    1.6f,
    0.0f,
    1.0f, //hitbox offset
    0.325f,
    0.2f,
    -1.30997f,
    1.30997f,
    -0.557341f,
    -0.557341f,
    1.4f,
    1.4f);

  vehicles.emplace_back();
  vehicles.back().setup(&vehicleType, &rigidBodies, &collisionPairs, &constraints, glm::length(gravity));
  vehicles.back().setPosition(glm::vec3(0.0f, 5.0f, 0.0f));
  vehicles.back().setOrientationFromDirection(glm::vec3(0.0f, 0.0f, 1.0f));
  addRenderObject(&(models["hull"]), &(textures["hull"]), vehicles.back().getHull()->getObject());
  addRenderObject(&(models["tyre"]), &(textures["tyre"]), vehicles.back().getTyre(0)->getObject());
  addRenderObject(&(models["tyre"]), &(textures["tyre"]), vehicles.back().getTyre(1)->getObject());
  addRenderObject(&(models["tyre"]), &(textures["tyre"]), vehicles.back().getTyre(2)->getObject());
  addRenderObject(&(models["tyre"]), &(textures["tyre"]), vehicles.back().getTyre(3)->getObject());

  CollisionPair::checkMeshCollisionPairMaxOverlapValidity(&collisionPairs);

  players.emplace_back();
  localPlayer = &players.back();
  testProfiles.kbm = input->getDefaultKeybindProfileKbm();
  testProfiles.controller = input->getDefaultKeybindProfileDs4();
  localPlayer->setKeybindProfiles(testProfiles);
  localPlayer->setJoystick(&(input->joysticks[0]));
  localPlayer->setControlledVehicle(&vehicles.back());
  localPlayer->setFollowedObject(vehicles.back().getHull()->getObject());

  camera.setupVehicleCamera(localPlayer->getControlledVehicle(), vehicleCameraDistance, vehicleCameraHeight, vehicleCameraAngle);
  camera.setupOrbitCamera(localPlayer->getFollowedObject(), orbitCameraInitialDistance, orbitCameraMinDistance, orbitCameraMaxDistance, orbitCameraMinAngleVertical, orbitCameraMaxAngleVertical);
  return true;
}

void World::update(float dt) {
  if(input->keyHit("pause")) isPaused = !isPaused;
  if(input->keyHit("controls")) showControls = !showControls;
  if(input->keyHit("saveState")) writeStateToFile(stateFile);
  if(input->keyHit("loadState")) readStateFromFile(stateFile);
  if(input->keyHit("reset") && !isPaused) {
    vehicles.back().setPosition(glm::vec3(0.0f, 5.0f, 0.0f));
    vehicles.back().setOrientationFromDirection(glm::vec3(0.0f, 0.0f, 1.0f));
    vehicles.back().setVelocity(glm::vec3(0.0f));
    vehicles.back().setAngularVelocity(glm::vec3(0.0f));
    auto rigidBody = rigidBodies.begin();
    for(int i = 0; i < 2; i++) {
      for(int j = 0; j < 2; j++) {
        std::advance(rigidBody, 1);
        rigidBody->setPosition(glm::vec3(
          Utility::randomDouble(0, 10),
          Utility::randomDouble(0, 10) + 10,
          Utility::randomDouble(0, 10)));
        rigidBody->setOrientation(glm::vec3(0.0f));
        rigidBody->setVelocity(glm::vec3(0.0f));
        rigidBody->setAngularVelocity(glm::vec3(0.0f));
      }
    }
    for(int i = 0; i < 2; i++) {
      for(int j = 0; j < 2; j++) {
        std::advance(rigidBody, 1);
        rigidBody->setPosition(glm::vec3(-4, 5 + (i * 6) + j * 3, 2));
        rigidBody->setOrientation(glm::vec3(0.0f));
        rigidBody->setVelocity(glm::vec3(0.0f));
        rigidBody->setAngularVelocity(glm::vec3(0.0f));
      }
    }
  }
  if(isPaused) {
    camera.processInput(dt, input, isPaused);
  } else {
    // use semi-implicit Euler integration by applying velocity- and angular velocity changes before position- and orientation changes
    // apply all external forces before applying constraint impulses
    // loop over all collision responses and constraints multiple times as they may invalidate each other (sequential impulse method)

    localPlayer->processLocalInput(input);
    camera.processInput(dt, input, isPaused);

    for(auto it = rigidBodies.begin(); it != rigidBodies.end(); ++it) {
      it->setFrameVariables();
      it->applyGravity(gravity, dt);
    }
    for(auto it = vehicles.begin(); it != vehicles.end(); ++it) {
      it->applySpringForces(dt);
      it->update(dt);
    }
    for(auto it = rigidBodies.begin(); it != rigidBodies.end(); ++it) {
      if(it->getGeometry()->type != Geometry::Type::mesh) {
        it->integrate(dt);
      }
    }
    for(auto it = constraints.begin(); it != constraints.end(); ++it) {
      (*it)->setupConstants(dt);
    }
    int constraintIterations = Constraint::iterations;
    if(constraints.size() == 0) constraintIterations = 1;

    for(int i = 0; i < constraintIterations; i++) {
      CollisionPair::clearDataCaches();
      for(auto it = rigidBodies.begin(); it != rigidBodies.end(); ++it) {
        it->clearContactManifolds();
        if(it->getGeometry()->type != Geometry::Type::mesh) {
          it->getGeometry()->transformToWorldSpace(it->getModelMatrix());
        }
      }
      std::vector<CollisionPair::Contact> objectContacts;
      for(auto it = collisionPairs.begin(); it != collisionPairs.end(); ++it) {
        CollisionPair::Contact contact = it->getContact();
        if(contact.contactManifold.points.size() > 0) {
          objectContacts.emplace_back(contact);
          contact.A->addContactManifold(contact.contactManifold);
          contact.contactManifold.normal *= -1.0f;
          contact.B->addContactManifold(contact.contactManifold);
        }
      }
      CollisionPair::sortContacts(&objectContacts, gravity);
      for(auto it = objectContacts.begin(); it != objectContacts.end(); ++it) {
        RigidBody::applyCollisionResponse(it->A, it->B, it->contactManifold);
      }
      for(auto it = constraints.begin(); it != constraints.end(); ++it) {
        (*it)->applyConstraintImpulses(false);
      }
    }
    for(auto it = vehicles.begin(); it != vehicles.end(); ++it) {
      it->applyRollingResistance();
    }
    for(auto it = rigidBodies.begin(); it != rigidBodies.end(); ++it) {
      if(!it->isStatic()) {
        it->determineSupportManifold(gravity);
        it->applyRestDetection();
      }
    }
  }
  if(vehicles.back().getHull()->getPosition().y < -5.0f) {
    vehicles.back().setPosition(glm::vec3(0.0f, 5.0f, 0.0f));
    vehicles.back().setOrientationFromDirection(glm::vec3(0.0f, 0.0f, 1.0f));
    vehicles.back().setVelocity(glm::vec3(0.0f));
    vehicles.back().setAngularVelocity(glm::vec3(0.0f));
  }
}

void World::updateCameraPosition(double alpha) {
  camera.updatePosition(alpha, isPaused);
}

void World::renderGeometry(double alpha) {
  renderWindow->setActive(true);

  glBindVertexArray(vertexArray);
  glDisable(GL_FRAMEBUFFER_SRGB);

  std::vector<Light::DirectionalLight> directionalLights;
  directionalLights.push_back(Light::DirectionalLight{glm::vec3(-0.7f, -0.8f, 0.0f), glm::vec3(1.0f, 1.0f, 1.0f), 1.0f});
  std::vector<Light::PointLight> pointLights;

  glm::mat4              view = camera.getViewMatrix();
  std::vector<glm::vec2> clippingPlanes = shadowMappingShader.calculateClippingPlanes(nearClippingPlane, farClippingPlane, shadowLevelCount);
  std::vector<glm::mat4> projectionMatrices = shadowMappingShader.calculateProjectionMatrices(renderWindow, fov, clippingPlanes);
  std::vector<glm::mat4> projectionViewMatrices;
  for(auto it = projectionMatrices.begin(); it != projectionMatrices.end(); ++it) {
    projectionViewMatrices.push_back((*it) * view);
  }
  std::vector<glm::mat4> projectionViewInverseMatrices;
  for(auto it = projectionViewMatrices.begin(); it != projectionViewMatrices.end(); ++it) {
    projectionViewInverseMatrices.push_back(glm::inverse(*it));
  }

  glm::mat4 projection = glm::perspective(
    glm::radians(fov),
    (float)renderWindow->getSize().x / (float)renderWindow->getSize().y,
    nearClippingPlane,
    farClippingPlane);
  projectionViewMatrix = projection * view;

  glm::vec3 lightDir = glm::normalize(directionalLights.back().direction);
  glm::mat4 depthViewMatrix = glm::lookAt(glm::vec3(0, 0, 0), lightDir, glm::vec3(0, 1, 0));

  std::vector<glm::mat4> depthViewProjections;
  for(auto it = projectionViewInverseMatrices.begin(); it != projectionViewInverseMatrices.end(); ++it) {
    depthViewProjections.push_back(shadowMappingShader.calculateDepthViewProjection(directionalLights.back().direction, *it, zMult, zBias));
  }

  shadowMappingShader.render(&renderObjects, alpha, depthViewProjections);

  for(auto it = clippingPlanes.begin(); it != clippingPlanes.end(); ++it) {
    glm::vec4 vecView(0.0f, 0.0f, it->y, 1.0f);
    glm::vec4 vecClip = projection * vecView;
    it->y = -vecClip.z;
  }

  glm::mat4 biasMatrix(
    0.5, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.5, 0.5, 0.5, 1.0);
  for(auto it = depthViewProjections.begin(); it != depthViewProjections.end(); ++it) {
    *it = biasMatrix * (*it);
  }

  opaqueShader.render(renderWindow,
                      screenShader.framebuffer,
                      &directionalLights,
                      &pointLights,
                      view,
                      shadowMappingShader.depthTextures,
                      &renderObjects,
                      alpha,
                      projectionViewMatrix,
                      depthViewProjections,
                      clippingPlanes);

  screenShader.render(renderWindow, true);
}

void World::renderUi(int fps) {
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  renderWindow->setActive(false);
  renderWindow->pushGLStates();
  renderWindow->resetGLStates();

  Utility::renderText(glm::vec2(5, 5), std::to_string(fps) + " FPS", sf::Color(255, 255, 255, 200));
  if(isPaused) Utility::renderText(glm::vec2(100, 5), "PAUSED", sf::Color(255, 255, 255, 200));
  if(showControls) {
    Utility::renderRectangle(glm::vec2(10, 30), glm::vec2(320, 345), sf::Color(0, 0, 0, 100));
    Utility::renderText(glm::vec2(20, 30), "Keyboard", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(20, 60), "Steer", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(200, 60), "[A, D]", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(20, 80), "Accelerate", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(200, 80), "[W]", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(20, 100), "Reverse", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(200, 100), "[S]", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(20, 120), "Next camera", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(200, 120), "[C]", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(20, 140), "Reset", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(200, 140), "[R]", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(20, 160), "Pause", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(200, 160), "[P]", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(20, 180), "Show/hide controls", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(200, 180), "[F1]", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(20, 200), "Save state", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(200, 200), "[F2]", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(20, 220), "Load state", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(200, 220), "[F3]", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(20, 240), "Quit", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(200, 240), "[Esc]", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(20, 280), "Controller", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(20, 310), "Steer", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(200, 310), "[Left joystick]", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(20, 330), "Accelerate", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(200, 330), "[Rt]", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(20, 350), "Reverse", sf::Color(255, 255, 255, 200));
    Utility::renderText(glm::vec2(200, 350), "[Lt]", sf::Color(255, 255, 255, 200));
  }
  renderWindow->popGLStates();
}
