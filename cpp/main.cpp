#include <GL/glew.h>
#include <SFML/OpenGL.hpp>
#include <SFML/Graphics.hpp>

#include <iostream>
#include <string>

#include "Input.h"
#include "Utility.h"
#include "World.h"

int main() {
  sf::ContextSettings settings;
  settings.depthBits = 24;
  settings.stencilBits = 8;
  settings.antialiasingLevel = 0;
  settings.sRgbCapable = true;
  sf::RenderWindow renderWindow;

  sf::VideoMode screen = sf::VideoMode::getDesktopMode();
  renderWindow.create(sf::VideoMode(1280, 720), "3d_engine", sf::Style::Titlebar | sf::Style::Close, settings);
  renderWindow.setActive(true);
  glEnable(GL_MULTISAMPLE);
  glViewport(0, 0, renderWindow.getSize().x, renderWindow.getSize().y);
  renderWindow.setVerticalSyncEnabled(false);
  settings = renderWindow.getSettings();
  std::cout << "depth bits: " << settings.depthBits << std::endl;
  std::cout << "stencil bits: " << settings.stencilBits << std::endl;
  std::cout << "sRGB capable: " << settings.sRgbCapable << std::endl;
  std::cout << "OpenGL version: " << settings.majorVersion << "." << settings.minorVersion << std::endl;

  glewExperimental = GL_TRUE;
  GLenum err = glewInit();
  if(GLEW_OK != err) {
    std::cout << "GLEW Error: " << glewGetErrorString(err) << std::endl;
  }
  std::cout << "GLEW version: " << glewGetString(GLEW_VERSION) << std::endl;

  Utility::initPRNG();

  Input input;
  input.init();

  World world;
  if(!world.init(&renderWindow, &input)) {
    return 0;
  }

  const double dt = 0.001;

  sf::Clock timer;
  timer.restart();

  double accumulator = 0.0;
  double currentTime = timer.getElapsedTime().asSeconds();

  int       fps = 0;
  sf::Clock fpsUpdateTimer;
  fpsUpdateTimer.restart();
  double totalFrameTime = 0.0;
  int    frameCounter = 0;

  bool running = true;
  while(running) {
    double newTime = timer.getElapsedTime().asSeconds();
    double frameTime = newTime - currentTime;
    totalFrameTime += frameTime;
    frameCounter++;

    if(fpsUpdateTimer.getElapsedTime().asSeconds() >= 0.5f) {
      fpsUpdateTimer.restart();
      fps = int(1.0 / (totalFrameTime / frameCounter));
      totalFrameTime = 0.0;
      frameCounter = 0;
    }

    if(frameTime > 0.25) {
      frameTime = 0.25;
    }

    currentTime = newTime;
    accumulator += frameTime;

    while(accumulator >= dt) {
      input.update(&renderWindow);
      if(input.keyHit("quit") || input.getClosedEvent()) {
        running = false;
      }

      world.update(dt);
      accumulator -= dt;
    }

    const float alpha = accumulator / dt;

    world.updateCameraPosition(alpha);
    world.renderGeometry(alpha);
    world.renderUi(fps);
    renderWindow.display();
  }

  return 0;
}
