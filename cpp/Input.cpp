#include "Input.h"
#include <iostream>

void Input::testMouseHit(int i) {
  bool keydown = MouseDownMem[i];
  bool hit = 0;

  if(MouseDown[i] && !keydown) {
    hit = 1;
    keydown = 1;
  }
  if(!MouseDown[i]) keydown = 0;

  MouseDownMem[i] = keydown;
  MouseHit[i] = hit;
}

bool Input::sameJoystickIdentification(unsigned int joystickNumber, sf::Joystick::Identification identification) {
  const sf::Joystick::Identification id = sf::Joystick::getIdentification(joystickNumber);
  if(id.name != identification.name) return false;
  if(id.vendorId != identification.vendorId) return false;
  if(id.productId != identification.productId) return false;
  return true;
}

Input::Input() {
  for(int i = 0; i < 3; i++) {
    MouseDown[i] = false;
    MouseDownMem[i] = false;
    MouseHit[i] = false;
  }
}

Input::~Input() {
}

void Input::init() {
  joysticks.reserve(100);

  // kbm

  keybindProfiles.emplace_back();
  keybindProfiles.back().name = "default_kbm";
  keybindProfiles.back().keybindings["quit"] = Keybind(sf::Keyboard::Escape);
  keybindProfiles.back().keybindings["controls"] = Keybind(sf::Keyboard::F1);
  keybindProfiles.back().keybindings["pause"] = Keybind(sf::Keyboard::P);
  keybindProfiles.back().keybindings["reset"] = Keybind(sf::Keyboard::R);
  keybindProfiles.back().keybindings["saveState"] = Keybind(sf::Keyboard::F2);
  keybindProfiles.back().keybindings["loadState"] = Keybind(sf::Keyboard::F3);

  keybindProfiles.back().keybindings["accelerate"] = Keybind(sf::Keyboard::W);
  keybindProfiles.back().keybindings["decelerate"] = Keybind(sf::Keyboard::S);
  keybindProfiles.back().keybindings["turnLeft"] = Keybind(sf::Keyboard::A);
  keybindProfiles.back().keybindings["turnRight"] = Keybind(sf::Keyboard::D);

  keybindProfiles.back().keybindings["toggleCamera"] = Keybind(sf::Keyboard::C);
  keybindProfiles.back().keybindings["moveCameraCloser"] = Keybind(Keybind::MouseWheelDelta::forward);
  keybindProfiles.back().keybindings["moveCameraFurther"] = Keybind(Keybind::MouseWheelDelta::backward);
  keybindProfiles.back().keybindings["rotateCamera"] = Keybind(sf::Mouse::Left);

  // controller

  keybindProfiles.emplace_back();
  keybindProfiles.back().name = "default_ds4";
  keybindProfiles.back().keybindings["accelerate"] = Keybind(Keybind::ControllerAxisDs4::r2);
  keybindProfiles.back().keybindings["decelerate"] = Keybind(Keybind::ControllerAxisDs4::l2);
  keybindProfiles.back().keybindings["turnLeft"] = Keybind(Keybind::ControllerAxisDs4::lLeft);
  keybindProfiles.back().keybindings["turnRight"] = Keybind(Keybind::ControllerAxisDs4::lRight);

  setCustomKeyBinds();

  for(auto itProfile = keybindProfiles.begin(); itProfile != keybindProfiles.end(); ++itProfile) {
    for(auto const& it: itProfile->keybindings) {
      itProfile->keybindings[it.first].setup(&frameTag, &mouseWheelDelta);
    }
  }
}

void Input::setCustomKeyBinds() {
}

Input::KeybindProfile* Input::getDefaultKeybindProfileKbm() {
  for(auto it = keybindProfiles.begin(); it != keybindProfiles.end(); ++it) {
    if(it->name == "default_kbm") return &(*it);
  }
  return nullptr;
}

Input::KeybindProfile* Input::getDefaultKeybindProfileDs4() {
  for(auto it = keybindProfiles.begin(); it != keybindProfiles.end(); ++it) {
    if(it->name == "default_ds4") return &(*it);
  }
  return nullptr;
}

void Input::update(sf::RenderWindow* renderWindow) {
  mouseWheelDelta = 0;
  closedEvent = false;
  backspacePressed = false;
  deletePressed = false;
  leftArrowPressed = false;
  rightArrowPressed = false;

  frameTag++;

  sf::Event event;

  while(renderWindow->pollEvent(event)) {
    if(event.type == sf::Event::Closed) {
      closedEvent = true;
    } else if(event.type == sf::Event::MouseWheelScrolled) {
      mouseWheelDelta = event.mouseWheelScroll.delta;
    }
    if(event.type == sf::Event::KeyPressed) {
      if(event.key.code == sf::Keyboard::BackSpace) backspacePressed = true;
      if(event.key.code == sf::Keyboard::Delete) deletePressed = true;
      if(event.key.code == sf::Keyboard::Left) leftArrowPressed = true;
      if(event.key.code == sf::Keyboard::Right) rightArrowPressed = true;
    }
    if(event.type == sf::Event::Resized) {
      renderWindow->setActive(true);
      glViewport(0, 0, event.size.width, event.size.height);
      renderWindow->setActive(false);
    }
  }

  windowActive = renderWindow->hasFocus();

  if(windowActive) {
    sf::Vector2i newMousePosition = sf::Mouse::getPosition(*renderWindow);
    if(newMousePosition.x > 0 &&
       newMousePosition.y > 0 &&
       newMousePosition.x < renderWindow->getSize().x &&
       newMousePosition.y < renderWindow->getSize().y) {
      mouseInWindow = true;
    } else {
      mouseInWindow = false;
    }
    mouseVelocity.x = newMousePosition.x - mousePosition.x;
    mouseVelocity.y = newMousePosition.y - mousePosition.y;
    mousePosition.x = newMousePosition.x;
    mousePosition.y = newMousePosition.y;
  }

  if(sf::Mouse::isButtonPressed(sf::Mouse::Left))
    MouseDown[0] = true;
  else
    MouseDown[0] = false;
  testMouseHit(0);

  if(sf::Mouse::isButtonPressed(sf::Mouse::Middle))
    MouseDown[1] = true;
  else
    MouseDown[1] = false;
  testMouseHit(1);

  if(sf::Mouse::isButtonPressed(sf::Mouse::Right))
    MouseDown[2] = true;
  else
    MouseDown[2] = false;
  testMouseHit(2);

  for(unsigned int i = 0; i < sf::Joystick::Count; i++) {
    if(sf::Joystick::isConnected(i)) {
      bool matchFound = false;
      for(unsigned int j = 0; j < joysticks.size(); j++) {
        if(sameJoystickIdentification(i, joysticks[j].identification)) {
          joysticks[j].isConnected = true;
          joysticks[j].number = i;
          joysticks[j].identification = sf::Joystick::getIdentification(i);
          matchFound = true;
          break;
        }
      }
      if(!matchFound) {
        Input::Joystick joystick;
        joystick.isConnected = true;
        joystick.number = i;
        joystick.identification = sf::Joystick::getIdentification(i);
        joysticks.emplace_back(joystick);
      }
    }
  }
  for(unsigned int i = 0; i < joysticks.size(); i++) {
    if(joysticks[i].isConnected) {
      bool matchFound = false;
      for(unsigned int j = 0; j < sf::Joystick::Count; j++) {
        if(sf::Joystick::isConnected(j)) {
          if(sameJoystickIdentification(j, joysticks[i].identification)) {
            matchFound = true;
            break;
          }
        }
      }
      if(!matchFound) {
        joysticks[i].isConnected = false;
      }
    }
  }
}

bool Input::getLeftMouseHit() {
  if(mouseInWindow && windowActive) return MouseHit[0];
  return false;
}

bool Input::getRightMouseHit() {
  if(mouseInWindow && windowActive) return MouseHit[2];
  return false;
}

bool Input::getMiddleMouseHit() {
  if(mouseInWindow && windowActive) return MouseHit[1];
  return false;
}

bool Input::getLeftMouseDown() {
  if(mouseInWindow && windowActive) return MouseDown[0];
  return false;
}

bool Input::getRightMouseDown() {
  if(mouseInWindow && windowActive) return MouseDown[2];
  return false;
}

bool Input::getMiddleMouseDown() {
  if(mouseInWindow && windowActive) return MouseDown[1];
  return false;
}

int Input::getMouseWheelDelta() {
  if(mouseInWindow && windowActive) return mouseWheelDelta;
  return 0;
}

sf::Vector2i Input::getMouseVelocity() {
  return mouseVelocity;
}

int Input::getMouseX() {
  return (int)mousePosition.x;
}

int Input::getMouseY() {
  return (int)mousePosition.y;
}

sf::Vector2i Input::getMousePosition() {
  return mousePosition;
}

bool Input::keyHit(std::string keybind) {
  PlayerKeybindProfiles defaultProfile;
  defaultProfile.kbm = &(keybindProfiles[0]);
  defaultProfile.controller = nullptr;
  return keyHit(keybind, &defaultProfile, nullptr);
}

bool Input::keyHit(std::string keybind, PlayerKeybindProfiles* profile, Input::Joystick* joystick) {
  bool kbmHit = false;
  bool controllerHit = false;
  if(windowActive) {
    if(profile->kbm != nullptr &&
       profile->kbm->keybindings.find(keybind) != profile->kbm->keybindings.end()) {
      kbmHit = profile->kbm->keybindings.at(keybind).hit();
    }
    if(profile->controller != nullptr &&
       profile->controller->keybindings.find(keybind) != profile->controller->keybindings.end() &&
       joystick != nullptr) {
      controllerHit = profile->controller->keybindings.at(keybind).hit(joystick->number);
    }
    return kbmHit || controllerHit;
  }
  return false;
}

bool Input::keyDown(std::string keybind, PlayerKeybindProfiles* profile, Input::Joystick* joystick) {
  bool kbmDown = false;
  bool controllerDown = false;
  if(windowActive) {
    if(profile->kbm != nullptr &&
       profile->kbm->keybindings.find(keybind) != profile->kbm->keybindings.end()) {
      kbmDown = profile->kbm->keybindings.at(keybind).down();
    }
    if(profile->controller != nullptr &&
       profile->controller->keybindings.find(keybind) != profile->controller->keybindings.end() &&
       joystick != nullptr) {
      if(joystick->isConnected) {
        controllerDown = profile->controller->keybindings.at(keybind).down(joystick->number);
      }
    }
    return kbmDown || controllerDown;
  }
  return false;
}

bool Input::keyDown(std::string keybind) {
  PlayerKeybindProfiles defaultProfile;
  defaultProfile.kbm = &(keybindProfiles[0]);
  defaultProfile.controller = nullptr;
  return keyDown(keybind, &defaultProfile, nullptr);
}

float Input::axisValue(std::string keybind, PlayerKeybindProfiles* profile, Input::Joystick* joystick) {
  if(windowActive) {
    if(profile->controller != nullptr &&
       profile->controller->keybindings.find(keybind) != profile->controller->keybindings.end() &&
       joystick != nullptr) {
      if(joystick->isConnected) {
        return profile->controller->keybindings.at(keybind).axisValue(joystick->number);
      }
    }
  }
  return 0.0f;
}

bool Input::getClosedEvent() {
  return closedEvent;
}

float Input::combineToAnalogValue(std::string keybind, Input::PlayerKeybindProfiles* keybindProfiles, Input::Joystick* joystick) {
  bool b = keyDown(keybind, keybindProfiles, joystick);
  if(b) return 1.0f;
  return axisValue(keybind, keybindProfiles, joystick);
}

bool Input::combineToDigitalValue(std::string keybind, Input::PlayerKeybindProfiles* keybindProfiles, Input::Joystick* joystick) {
  bool b = keyDown(keybind, keybindProfiles, joystick);
  if(b) return true;
  float f = axisValue(keybind, keybindProfiles, joystick);
  if(f > 0.0f) return true;
  return false;
}
