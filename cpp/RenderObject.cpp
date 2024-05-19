#include "RenderObject.h"

RenderObject::ModelTexturePair* RenderObject::getModelTexturePair(std::list<ModelTexturePair>* modelTexturePairs, Model* model, Texture* texture){
  for (auto it = modelTexturePairs->begin(); it != modelTexturePairs->end(); ++it) {
    if(it->model == model && it->texture == texture){
      return &(*it);
    }
  }
  return nullptr;
}

int RenderObject::getSortIdentifier() const {
  return (modelTexturePair->model->getId() * Texture::getTextureCount()) + modelTexturePair->texture->getId();
}

RenderObject::RenderObject(std::list<ModelTexturePair>* modelTexturePairs, Model* model, Texture* texture, Object* object){
  modelTexturePair = getModelTexturePair(modelTexturePairs, model, texture);
  if(modelTexturePair == nullptr){
    modelTexturePairs->emplace_back(ModelTexturePair{model, texture});
    modelTexturePair = &modelTexturePairs->back();
  }
  this->object = object;
}

RenderObject::~RenderObject(){
}

void RenderObject::sortRenderObjects(std::vector<RenderObject>* renderObjects){
  std::sort(
    renderObjects->begin(), renderObjects->end(), [](RenderObject const& a, RenderObject const& b) { return a.getSortIdentifier() < b.getSortIdentifier(); }
  );
}
