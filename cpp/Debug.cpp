
#include "Debug.h"

void Debug::renderFaceNormal(Collision::Face* face) {
  Collision::HalfEdge* currentEdge = face->startEdge;
  glm::vec3            average(0, 0, 0);
  int                  vertices = 0;
  do {
    average = average + *(currentEdge->startVertex);
    vertices++;
    currentEdge = currentEdge->next;
  } while(currentEdge != face->startEdge);
  average = (1.0f / (float)vertices) * average;
  Utility::renderLine(
    Utility::getScreenCoordinates(average),
    Utility::getScreenCoordinates(average + face->normal),
    sf::Color(0, 255, 255));
}

void Debug::renderPolyhedronWireframe(Collision::Polyhedron* polyhedron, sf::Color colour) {
  for(int i = 0; i < polyhedron->faces.size(); i++) {
    Collision::HalfEdge* currentEdge = polyhedron->faces[i].startEdge;
    do {
      Utility::renderLine(
        Utility::getScreenCoordinates(*(currentEdge->startVertex)),
        Utility::getScreenCoordinates(*(currentEdge->twin->startVertex)),
        colour);
      currentEdge = currentEdge->next;
    } while(currentEdge != polyhedron->faces[i].startEdge);
  }
}

void Debug::renderHalfEdge(Collision::HalfEdge* edge) {
  Utility::renderLine(
    Utility::getScreenCoordinates(*(edge->startVertex)),
    Utility::getScreenCoordinates(*(edge->twin->startVertex)),
    sf::Color(0, 255, 0));
  Utility::renderLine(
    Utility::getScreenCoordinates(*(edge->startVertex)) + glm::vec2(5, 5),
    Utility::getScreenCoordinates(*(edge->twin->startVertex)) + glm::vec2(5, 5),
    sf::Color(255, 150, 0));
  Utility::renderMarker(Utility::getScreenCoordinates(*(edge->startVertex)) + glm::vec2(-3, -3), sf::Color(0, 255, 0));
  Utility::renderMarker(Utility::getScreenCoordinates(*(edge->twin->startVertex)) + glm::vec2(2, 2), sf::Color(255, 150, 0));
  renderFaceNormal(edge->face);
}

void Debug::log(int value) {
  std::cout << std::to_string(value) << std::endl;
}

void Debug::log(double value) {
  std::cout << std::to_string(value) << std::endl;
}

void Debug::log(glm::vec3 value) {
  std::cout << std::to_string(value.x) << ", " << std::to_string(value.y) << ", " << std::to_string(value.z) << std::endl;
}

void Debug::log(std::string value) {
  std::cout << value << std::endl;
}

void Debug::log(void* value) {
  std::cout << value << std::endl;
}

void Debug::log(std::string message, int value) {
  std::cout << message << std::to_string(value) << std::endl;
}

void Debug::log(std::string message, double value) {
  std::cout << message << std::to_string(value) << std::endl;
}

void Debug::log(std::string message, glm::vec3 value) {
  std::cout << message << std::to_string(value.x) << ", " << std::to_string(value.y) << ", " << std::to_string(value.z) << std::endl;
}

void Debug::log(std::string message, std::string value) {
  std::cout << message << value << std::endl;
}

void Debug::log(std::string message, void* value) {
  std::cout << message << value << std::endl;
}
