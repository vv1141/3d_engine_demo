
#include "Utility.h"
#include "Debug.h"

const float Utility::tau = 6.283185307;
const float Utility::epsilon = 0.00001f;

bool Utility::onDistance(glm::vec3 a, glm::vec3 b, float distance) {
  return distanceSquared(a, b) <= distance * distance;
}

double Utility::distanceSquared(glm::vec3 a, glm::vec3 b) {
  return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

glm::vec3 Utility::getOneOrthogonalVector(glm::vec3 vector) {
  glm::vec3 vAbs = glm::vec3(std::abs(vector.x), std::abs(vector.y), std::abs(vector.z));
  int       minAxis = 0;
  if(vAbs.y <= vAbs.x && vAbs.y <= vAbs.z) minAxis = 1;
  if(vAbs.z <= vAbs.x && vAbs.z <= vAbs.y) minAxis = 2;
  if(minAxis == 0) {
    return glm::vec3(0.0f, -vector.z, vector.y);
  } else if(minAxis == 1) {
    return glm::vec3(vector.z, 0.0f, -vector.x);
  }
  return glm::vec3(-vector.y, vector.x, 0.0f);
}

glm::vec3 Utility::getOneOrthogonalUnitVector(glm::vec3 vector) {
  return glm::normalize(getOneOrthogonalVector(vector));
}

bool Utility::triangleHasArea(glm::vec3* triangleVertices, glm::vec3 triangleNormal) {
  glm::vec3 edge = triangleVertices[1] - triangleVertices[0];
  glm::vec3 edgeNormal = glm::cross(triangleNormal, edge);
  if(std::abs(glm::dot(edgeNormal, (triangleVertices[2] - triangleVertices[0]))) > epsilon) {
    return true;
  }
  return false;
}

bool Utility::pointInsideTriangle(glm::vec3 point, glm::vec3* triangleVertices, glm::vec3 triangleNormal) {
  int signs = 0;
  for(int i = 0; i < 3; i++) {
    int j = i + 1;
    if(j == 3) {
      j = 0;
    }
    glm::vec3 edge = triangleVertices[j] - triangleVertices[i];
    glm::vec3 edgeNormal = glm::cross(triangleNormal, edge);
    if(glm::dot(edgeNormal, (point - triangleVertices[i])) > 0.0f) {
      signs++;
    }
  }
  if(signs == 0 || signs == 3) {
    return true;
  }
  return false;
}

glm::vec3 Utility::getTriangleNormal(std::vector<glm::vec3>* vertices) {
  // assume vertices are in counter-clockwise order
  const std::vector<glm::vec3>& verticesRef = *vertices;
  if(verticesRef.size() < 3) {
    Debug::log("Error: Not a triangle: less than three vertices");
    return glm::vec3(0.0f);
  }
  const glm::vec3 normal = glm::normalize(glm::cross(verticesRef[1] - verticesRef[0], verticesRef[2] - verticesRef[0]));
  if(isNan(normal)) {
    Debug::log("Error: Normal calculation failed; vertices:");
    for(int i = 0; i < verticesRef.size(); i++) {
      Debug::log(verticesRef[i]);
    }
    return glm::vec3(0.0f);
  }
  return normal;
}

bool Utility::verticesFormPlane(std::vector<glm::vec3>* vertices, glm::vec3* normal) {
  const std::vector<glm::vec3>& verticesRef = *vertices;
  const int                     c = verticesRef.size();
  if(c < 3) {
    Debug::log("Error: Vertices do not form a plane: less than three vertices");
    return false;
  }
  if(c > 3) {
    glm::vec3 pointOnPlane = verticesRef[0];
    for(int i = 1; i < c; i++) {
      glm::vec3   point = verticesRef[i];
      float       pointToPlaneDistance = std::abs(glm::dot(*normal, (point - pointOnPlane)));
      const float slack = 0.001f;
      if(pointToPlaneDistance > slack) {
        Debug::log("Error: Vertices do not form a plane: all vertices are not on the same plane; vertices:");
        for(int i = 0; i < c; i++) {
          Debug::log(verticesRef[i]);
        }
        return false;
      }
    }
  }
  return true;
}

float Utility::getFraction(float value, float lowerThreshold, float upperThreshold) {
  if(value <= lowerThreshold) {
    return 0.0f;
  }
  if(value >= upperThreshold) {
    return 1.0f;
  }
  return (value - lowerThreshold) / (upperThreshold - lowerThreshold);
}

glm::vec3 Utility::vectorRejection(glm::vec3 vector, glm::vec3 normal) {
  return vector - (glm::dot(vector, normal) * normal);
}

bool Utility::isNan(glm::vec3 vec) {
  return isnan(vec.x) || isnan(vec.y) || isnan(vec.z);
}

sf::RenderWindow* Utility::renderWindow = nullptr;
glm::mat4*        Utility::projectionViewMatrix = nullptr;
glm::vec2         Utility::screenSize = glm::vec2();

void Utility::initRenderState(sf::RenderWindow* renderWindow, glm::mat4* projectionViewMatrix) {
  Utility::renderWindow = renderWindow;
  Utility::projectionViewMatrix = projectionViewMatrix;
  Utility::screenSize = glm::vec2(renderWindow->getSize().x, renderWindow->getSize().y);
}

void Utility::renderLine(glm::vec2 startPosition, glm::vec2 endPosition, sf::Color colour) {
  sf::Vertex line[] = {
    sf::Vertex(sf::Vector2f(startPosition.x, startPosition.y)),
    sf::Vertex(sf::Vector2f(endPosition.x, endPosition.y))};
  line[0].color = colour;
  line[1].color = colour;
  renderWindow->draw(line, 2, sf::Lines);
}

void Utility::renderMarker(glm::vec2 position, sf::Color colour) {
  sf::CircleShape marker(4, 8);
  marker.setFillColor(colour);
  marker.setPosition(position.x, position.y);
  renderWindow->draw(marker);
}

void Utility::renderCircle(glm::vec2 position, float size, sf::Color colour) {
  sf::CircleShape circle(size, size);
  circle.setFillColor(colour);
  circle.setPosition(position.x, position.y);
  renderWindow->draw(circle);
}

void Utility::renderRectangle(glm::vec2 position, glm::vec2 size, sf::Color colour) {
  sf::RectangleShape rectangle(sf::Vector2f(size.x, size.y));
  rectangle.setFillColor(colour);
  rectangle.setPosition(position.x, position.y);
  renderWindow->draw(rectangle);
}

void Utility::renderRectangle(glm::vec2 position, glm::vec2 size, sf::Color colour, sf::Texture texture) {
  sf::RectangleShape rectangle(sf::Vector2f(size.x, size.y));
  rectangle.setTexture(&texture);
  rectangle.setTextureRect(sf::IntRect(0, 0, size.x, size.y));
  rectangle.setFillColor(colour);
  rectangle.setPosition(position.x, position.y);
  renderWindow->draw(rectangle);
}

sf::Font Utility::font = sf::Font();
sf::Text Utility::text = sf::Text();

void Utility::initTextRendering(std::string fontPath) {
  Utility::font.loadFromFile(fontPath);
  Utility::text.setFont(Utility::font);
  Utility::text.setCharacterSize(18);
  Utility::text.setFillColor(sf::Color::White);
}

void Utility::renderText(glm::vec2 position, std::string string) {
  Utility::text.setPosition(position.x, position.y);
  Utility::text.setString(string);
  Utility::text.setFillColor(sf::Color::White);
  renderWindow->draw(Utility::text);
}

void Utility::renderText(glm::vec2 position, std::string string, sf::Color colour) {
  Utility::text.setPosition(position.x, position.y);
  Utility::text.setString(string);
  Utility::text.setFillColor(colour);
  renderWindow->draw(Utility::text);
}

std::mt19937 Utility::generator = std::mt19937();

void Utility::initPRNG() {
  std::random_device rd;
  Utility::generator = std::mt19937(rd());
}

int Utility::randomInt(int min, int max) {
  std::uniform_int_distribution<> distribution(min, max);
  return distribution(Utility::generator);
}

double Utility::randomDouble(double min, double max) {
  std::uniform_real_distribution<> distribution(min, max);
  return distribution(Utility::generator);
}

glm::vec2 Utility::getScreenCoordinates(glm::vec3 position) {
  glm::vec3 transformedWorldPosition = (*projectionViewMatrix) * glm::vec4(position, 1.0f);
  float     x = transformedWorldPosition.x / transformedWorldPosition.z;
  float     y = transformedWorldPosition.y / transformedWorldPosition.z;
  float     px = (x + 1.0f) * (float)screenSize.x * 0.5f;
  float     py = (y + 1.0f) * (float)screenSize.y * 0.5f;
  return glm::vec2(
    std::floor(px),
    std::floor((float)screenSize.y - py));
}
