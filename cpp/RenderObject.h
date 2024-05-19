#ifndef RENDER_OBJECT_H
#define RENDER_OBJECT_H

#include <list>
#include <algorithm>

#include "Model.h"
#include "Texture.h"
#include "Object.h"

class RenderObject {
public:

  // a collection of RenderObjects is orderable by model and texture for efficient rendering by avoiding unnecessary binding of model and texture

  struct ModelTexturePair {
    Model*   model;
    Texture* texture;
  };

private:

  ModelTexturePair* getModelTexturePair(std::list<ModelTexturePair>* modelTexturePairs, Model* model, Texture* texture);
  int getSortIdentifier() const;

public:

  ModelTexturePair* modelTexturePair;
  Object*           object;

  RenderObject(std::list<ModelTexturePair>* modelTexturePairs, Model* model, Texture* texture, Object* object);
  ~RenderObject();

  static void sortRenderObjects(std::vector<RenderObject>* renderObjects);

};

#endif