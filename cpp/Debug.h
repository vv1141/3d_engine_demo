#ifndef DEBUG_H
#define DEBUG_H

#include <glm/glm.hpp>
#include <iostream>

#include "Collision.h"
#include "Utility.h"

class Debug {
private:
  static void renderFaceNormal(Collision::Face* face);

public:
  static void renderPolyhedronWireframe(Collision::Polyhedron* polyhedron, sf::Color colour);
  static void renderHalfEdge(Collision::HalfEdge* edge);
  static void log(int value);
  static void log(double value);
  static void log(glm::vec3 value);
  static void log(std::string value);
  static void log(void* value);
  static void log(std::string message, int value);
  static void log(std::string message, double value);
  static void log(std::string message, glm::vec3 value);
  static void log(std::string message, std::string value);
  static void log(std::string message, void* value);
};

#endif
