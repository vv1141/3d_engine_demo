#include "Texture.h"

int Texture::uniqueIdGenerator = 0;

bool Texture::loadTexture(std::string path, GLuint* textureId, bool sRgb) {
  sf::Image image;
  if(!image.loadFromFile(path)) {
    return false;
  }

  glGenTextures(1, textureId);
  glBindTexture(GL_TEXTURE_2D, *textureId);

  const unsigned char* pixels = image.getPixelsPtr();
  if(sRgb) {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB_ALPHA, image.getSize().x, image.getSize().y, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
  } else {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.getSize().x, image.getSize().y, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
  }

  // trilinear filtering
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glGenerateMipmap(GL_TEXTURE_2D);
  return true;
}

bool Texture::create1x1GreyTexture(GLuint* textureId) {
  const unsigned char pixels[] = {100, 100, 100, 100};
  glGenTextures(1, textureId);
  glBindTexture(GL_TEXTURE_2D, *textureId);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
  return true;
}

bool Texture::create1x1NormalTexture(GLuint* textureId) {
  const unsigned char pixels[] = {128, 128, 255, 255};
  glGenTextures(1, textureId);
  glBindTexture(GL_TEXTURE_2D, *textureId);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
  return true;
}

Texture::Texture() {
  id = uniqueIdGenerator;
  uniqueIdGenerator++;
}

Texture::~Texture() {
  if(textures & Flags::diffuse) glDeleteTextures(1, &diffuseMapId);
  if(textures & Flags::normal) glDeleteTextures(1, &normalMapId);
  if(textures & Flags::depth) glDeleteTextures(1, &depthMapId);
}

bool Texture::loadTextures(std::string path, int flags) {
  this->textures = flags;
  bool success = true;
  if(flags & Flags::diffuse) {
    if(!loadTexture(path + "/diffuse.png", &diffuseMapId, true)) { // diffuse maps are considered to be in sRGB space
      success = false;
    }
  } else {
    create1x1GreyTexture(&diffuseMapId);
  }
  if(flags & Flags::normal) {
    if(!loadTexture(path + "/normal.png", &normalMapId, false)) { // normal maps are considered to be in linear colour space (although possibly generated from manually drawn heightmaps)
      success = false;
    }
  } else {
    create1x1NormalTexture(&normalMapId);
  }
  if(flags & Flags::depth) {
    if(!loadTexture(path + "/depth.png", &depthMapId, false)) { // depth maps are considered to be in linear colour space (although possibly manually drawn)
      success = false;
    }
  } else {
    create1x1GreyTexture(&depthMapId);
  }
  return success;
}

void Texture::bindTextures(GLuint diffuseMapSampler, GLuint normalMapSampler, GLuint depthMapSampler, int flags) {
  if(flags & Flags::diffuse) {
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, diffuseMapId);
    glUniform1i(diffuseMapSampler, 0);
  }
  if(flags & Flags::normal) {
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, normalMapId);
    glUniform1i(normalMapSampler, 1);
  }
  if(flags & Flags::depth) {
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, depthMapId);
    glUniform1i(depthMapSampler, 2);
  }
}

int Texture::getId() {
  return id;
}

int Texture::getTextureCount() {
  return uniqueIdGenerator;
}
