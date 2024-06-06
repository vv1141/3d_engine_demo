#ifndef TEXTURE_H
#define TEXTURE_H

#include <string>

#include <GL/glew.h>
#include <SFML/OpenGL.hpp>
#include <SFML/Graphics.hpp>

class Texture {
public:
  enum Flags {
    diffuse = 0x01,
    normal = 0x02,
    depth = 0x04
  };

private:
  static int uniqueIdGenerator;
  int        id;
  GLuint     diffuseMapId;
  GLuint     normalMapId;
  GLuint     depthMapId;
  int        textures;

  bool loadTexture(std::string path, GLuint* textureId, bool sRgb);
  bool create1x1GreyTexture(GLuint* textureId);
  bool create1x1NormalTexture(GLuint* textureId);

public:
  Texture();
  ~Texture();

  bool       loadTextures(std::string path, int flags = 0);
  void       bindTextures(GLuint diffuseMapSampler, GLuint normalMapSampler, GLuint depthMapSampler, int flags);
  int        getId();
  static int getTextureCount();
};

#endif
