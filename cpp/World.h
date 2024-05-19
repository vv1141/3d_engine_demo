#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <list>
#include <memory>

#include <GL/glew.h>
#include <SFML/OpenGL.hpp>
#include <SFML/Graphics.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/rotate_vector.hpp>

#include "Input.h"
#include "Collision.h"
#include "CollisionPair.h"
#include "RigidBody.h"
#include "Shader.h"
#include "Model.h"
#include "Texture.h"
#include "RenderObject.h"
#include "Vehicle.h"
#include "VehicleType.h"
#include "Light.h"
#include "Player.h"
#include "Camera.h"
#include "Utility.h"
#include "Debug.h"

class World {
public:

private:

  int depthMapResolution;
  int shadowLevelCount;
  float zMult;
  float zBias;
  float fov;
  float nearClippingPlane;
  float farClippingPlane;
  bool multisamplingEnabled;
  int multisamplingSampleCount;

  sf::RenderWindow* renderWindow;
  Input* input;
  sf::Vector2i mousePosition;
  Camera camera;
  bool isPaused;
  bool showControls;
  std::string stateFile;

  GLuint vertexArray;
  OpaqueShader opaqueShader;
  ShadowMappingShader shadowMappingShader;
  ScreenShader screenShader;

  std::list<Player> players;
  Player* localPlayer;
  std::list<RigidBody> rigidBodies;
  std::vector<CollisionPair> collisionPairs;
  std::vector<std::unique_ptr<Constraint>> constraints;
  std::list<Vehicle> vehicles;
  std::list<RenderObject::ModelTexturePair> modelTexturePairs;
  std::vector<RenderObject> renderObjects;
  Model cubeModel;
  Model hullModel;
  Model frontRightTyreModel;
  Model frontLeftTyreModel;
  Model rearRightTyreModel;
  Model rearLeftTyreModel;
  Model terrainModel;
  Texture cubeTexture;
  Texture hullTexture;
  Texture tyreTexture;
  Texture terrainTexture;

  Collision::Polyhedron cubeHitbox;
  Collision::Polyhedron wheelPolyhedron;
  std::vector<Collision::Polyhedron> terrainMesh;
  RigidBody* cubePointer;
  VehicleType vehicleType;
  glm::vec3 lightPosition;
  glm::mat4 projectionViewMatrix;
  Input::PlayerKeybindProfiles testProfiles;
  float vehicleCameraDistance;
  float vehicleCameraHeight;
  float vehicleCameraAngle;
  float orbitCameraInitialDistance;
  float orbitCameraMinDistance;
  float orbitCameraMaxDistance;
  float orbitCameraMinAngleVertical;
  float orbitCameraMaxAngleVertical;
  glm::vec3 gravity;

  void addRenderObject(Model* model, Texture* texture, Object* object);
  void addTransparentRenderObject(Model* model, Texture* texture, Object* object);

  bool writeStateToFile(std::string path);
  bool readStateFromFile(std::string path);

public:

  World();
  ~World();

  void init(sf::RenderWindow* renderWindow, Input* input);
  void update(float dt);
  void updateCameraPosition(double alpha);
  void renderGeometry(double alpha);
  void renderUi(int fps);
};

#endif
