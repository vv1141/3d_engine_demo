
#include "Keybind.h"

std::string Keybind::getKeyboardKeyName(const sf::Keyboard::Key key) {
  switch(key) {
    case sf::Keyboard::Unknown:
      return "Unknown";
    case sf::Keyboard::A:
      return "A";
    case sf::Keyboard::B:
      return "B";
    case sf::Keyboard::C:
      return "C";
    case sf::Keyboard::D:
      return "D";
    case sf::Keyboard::E:
      return "E";
    case sf::Keyboard::F:
      return "F";
    case sf::Keyboard::G:
      return "G";
    case sf::Keyboard::H:
      return "H";
    case sf::Keyboard::I:
      return "I";
    case sf::Keyboard::J:
      return "J";
    case sf::Keyboard::K:
      return "K";
    case sf::Keyboard::L:
      return "L";
    case sf::Keyboard::M:
      return "M";
    case sf::Keyboard::N:
      return "N";
    case sf::Keyboard::O:
      return "O";
    case sf::Keyboard::P:
      return "P";
    case sf::Keyboard::Q:
      return "Q";
    case sf::Keyboard::R:
      return "R";
    case sf::Keyboard::S:
      return "S";
    case sf::Keyboard::T:
      return "T";
    case sf::Keyboard::U:
      return "U";
    case sf::Keyboard::V:
      return "V";
    case sf::Keyboard::W:
      return "W";
    case sf::Keyboard::X:
      return "X";
    case sf::Keyboard::Y:
      return "Y";
    case sf::Keyboard::Z:
      return "Z";
    case sf::Keyboard::Num0:
      return "Num0";
    case sf::Keyboard::Num1:
      return "Num1";
    case sf::Keyboard::Num2:
      return "Num2";
    case sf::Keyboard::Num3:
      return "Num3";
    case sf::Keyboard::Num4:
      return "Num4";
    case sf::Keyboard::Num5:
      return "Num5";
    case sf::Keyboard::Num6:
      return "Num6";
    case sf::Keyboard::Num7:
      return "Num7";
    case sf::Keyboard::Num8:
      return "Num8";
    case sf::Keyboard::Num9:
      return "Num9";
    case sf::Keyboard::Escape:
      return "Escape";
    case sf::Keyboard::LControl:
      return "LControl";
    case sf::Keyboard::LShift:
      return "LShift";
    case sf::Keyboard::LAlt:
      return "LAlt";
    case sf::Keyboard::LSystem:
      return "LSystem";
    case sf::Keyboard::RControl:
      return "RControl";
    case sf::Keyboard::RShift:
      return "RShift";
    case sf::Keyboard::RAlt:
      return "RAlt";
    case sf::Keyboard::RSystem:
      return "RSystem";
    case sf::Keyboard::Menu:
      return "Menu";
    case sf::Keyboard::LBracket:
      return "LBracket";
    case sf::Keyboard::RBracket:
      return "RBracket";
    case sf::Keyboard::SemiColon:
      return "SemiColon";
    case sf::Keyboard::Comma:
      return "Comma";
    case sf::Keyboard::Period:
      return "Period";
    case sf::Keyboard::Quote:
      return "Quote";
    case sf::Keyboard::Slash:
      return "Slash";
    case sf::Keyboard::BackSlash:
      return "BackSlash";
    case sf::Keyboard::Tilde:
      return "Tilde";
    case sf::Keyboard::Equal:
      return "Equal";
    case sf::Keyboard::Dash:
      return "Dash";
    case sf::Keyboard::Space:
      return "Space";
    case sf::Keyboard::Return:
      return "Return";
    case sf::Keyboard::BackSpace:
      return "BackSpace";
    case sf::Keyboard::Tab:
      return "Tab";
    case sf::Keyboard::PageUp:
      return "PageUp";
    case sf::Keyboard::PageDown:
      return "PageDown";
    case sf::Keyboard::End:
      return "End";
    case sf::Keyboard::Home:
      return "Home";
    case sf::Keyboard::Insert:
      return "Insert";
    case sf::Keyboard::Delete:
      return "Delete";
    case sf::Keyboard::Add:
      return "Add";
    case sf::Keyboard::Subtract:
      return "Subtract";
    case sf::Keyboard::Multiply:
      return "Multiply";
    case sf::Keyboard::Divide:
      return "Divide";
    case sf::Keyboard::Left:
      return "Left";
    case sf::Keyboard::Right:
      return "Right";
    case sf::Keyboard::Up:
      return "Up";
    case sf::Keyboard::Down:
      return "Down";
    case sf::Keyboard::Numpad0:
      return "Numpad0";
    case sf::Keyboard::Numpad1:
      return "Numpad1";
    case sf::Keyboard::Numpad2:
      return "Numpad2";
    case sf::Keyboard::Numpad3:
      return "Numpad3";
    case sf::Keyboard::Numpad4:
      return "Numpad4";
    case sf::Keyboard::Numpad5:
      return "Numpad5";
    case sf::Keyboard::Numpad6:
      return "Numpad6";
    case sf::Keyboard::Numpad7:
      return "Numpad7";
    case sf::Keyboard::Numpad8:
      return "Numpad8";
    case sf::Keyboard::Numpad9:
      return "Numpad9";
    case sf::Keyboard::F1:
      return "F1";
    case sf::Keyboard::F2:
      return "F2";
    case sf::Keyboard::F3:
      return "F3";
    case sf::Keyboard::F4:
      return "F4";
    case sf::Keyboard::F5:
      return "F5";
    case sf::Keyboard::F6:
      return "F6";
    case sf::Keyboard::F7:
      return "F7";
    case sf::Keyboard::F8:
      return "F8";
    case sf::Keyboard::F9:
      return "F9";
    case sf::Keyboard::F10:
      return "F10";
    case sf::Keyboard::F11:
      return "F11";
    case sf::Keyboard::F12:
      return "F12";
    case sf::Keyboard::F13:
      return "F13";
    case sf::Keyboard::F14:
      return "F14";
    case sf::Keyboard::F15:
      return "F15";
    case sf::Keyboard::Pause:
      return "Pause";
    default:
      return "Unknown";
  }
}

std::string Keybind::getMouseButtonName(const sf::Mouse::Button button) {
  switch(button) {
    case sf::Mouse::Left:
      return "Left Mouse Button";
    case sf::Keyboard::Right:
      return "Right Mouse Button";
    case sf::Mouse::Middle:
      return "Middle Mouse Button";
    case sf::Mouse::XButton1:
      return "Extra Mouse Button 1";
    case sf::Mouse::XButton2:
      return "Extra Mouse Button 2";
    default:
      return "Unknown";
  }
}

std::string Keybind::getControllerButtonNameDs4(ControllerButtonDs4 button) {
  switch(button) {
    case ControllerButtonDs4::cross:
      return "Cross";
    case ControllerButtonDs4::square:
      return "Square";
    case ControllerButtonDs4::triangle:
      return "Triangle";
    case ControllerButtonDs4::circle:
      return "Circle";
    default:
      return "Unknown";
  }
}

std::string Keybind::getControllerAxisNameDs4(ControllerAxisDs4 axis) {
  switch(axis) {
    case ControllerAxisDs4::lLeft:
      return "Left Stick X-Axis Left";
    case ControllerAxisDs4::lRight:
      return "Left Stick X-Axis Right";
    case ControllerAxisDs4::lUp:
      return "Left Stick Y-Axis Up";
    case ControllerAxisDs4::lDown:
      return "Left Stick Y-Axis Down";
    case ControllerAxisDs4::rLeft:
      return "Right Stick X-Axis Left";
    case ControllerAxisDs4::rRight:
      return "Right Stick X-Axis Right";
    case ControllerAxisDs4::rUp:
      return "Right Stick Y-Axis Up";
    case ControllerAxisDs4::rDown:
      return "Right Stick Y-Axis Down";
    case ControllerAxisDs4::l2:
      return "Left Trigger";
    case ControllerAxisDs4::r2:
      return "Right Trigger";
    case ControllerAxisDs4::dpadLeft:
      return "Dpad Left";
    case ControllerAxisDs4::dpadRight:
      return "Dpad Right";
    case ControllerAxisDs4::dpadUp:
      return "Dpad Up";
    case ControllerAxisDs4::dpadDown:
      return "Dpad Down";
    default:
      return "Unknown";
  }
}

Keybind::Keybind() {
  type = Type::unbound;
  name = "Unbound";
  previouslyDown = false;
}

Keybind::Keybind(sf::Keyboard::Key keyboardBind) {
  setKeyboardBind(keyboardBind);
  previouslyDown = false;
}

Keybind::Keybind(sf::Mouse::Button mouseButtonBind) {
  setMouseButtonBind(mouseButtonBind);
  previouslyDown = false;
}

Keybind::Keybind(MouseWheelDelta mouseWheelDelta) {
  setMouseWheelBind(mouseWheelDelta);
  previouslyDown = false;
}

Keybind::Keybind(ControllerButtonDs4 button) {
  setControllerButtonBindDs4(button);
  previouslyDown = false;
}

Keybind::Keybind(ControllerAxisDs4 axis) {
  setControllerAxisBindDs4(axis);
  previouslyDown = false;
}

Keybind::~Keybind() {
}

void Keybind::setKeyboardBind(sf::Keyboard::Key keyboardBind) {
  type = Type::keyboard;
  name = getKeyboardKeyName(keyboardBind);
  this->keyboardBind = keyboardBind;
}

void Keybind::setMouseButtonBind(sf::Mouse::Button mouseButtonBind) {
  type = Type::mouseButton;
  name = getMouseButtonName(mouseButtonBind);
  this->mouseButtonBind = mouseButtonBind;
}

void Keybind::setMouseWheelBind(MouseWheelDelta mouseWheelDelta) {
  type = Type::mouseWheel;
  mouseWheelBind = mouseWheelDelta;
  if(mouseWheelDelta == MouseWheelDelta::backward) {
    name = "Mouse wheel backward";
  } else {
    name = "Mouse wheel forward";
  }
}

void Keybind::setControllerButtonBindDs4(ControllerButtonDs4 button) {
  type = Type::controllerButtonDs4;
  name = getControllerButtonNameDs4(button);
  this->controllerButtonBindDs4 = button;
}

void Keybind::setControllerAxisBindDs4(ControllerAxisDs4 axis) {
  type = Type::controllerAxisDs4;
  name = getControllerAxisNameDs4(axis);
  this->controllerAxisBindDs4 = axis;
}

void Keybind::setup(int* frameTag, int* updatedMouseWheelDelta) {
  this->updatedFrameTag = frameTag;
  this->updatedMouseWheelDelta = updatedMouseWheelDelta;
}

void Keybind::unbind() {
  type = Type::unbound;
  name = "Unbound";
}

std::string Keybind::getBindName() {
  return name;
}

bool Keybind::down() {
  return down(-1);
}

bool Keybind::down(unsigned int joystickNumber) {
  if(type == Type::keyboard) {
    return sf::Keyboard::isKeyPressed(keyboardBind);
  }
  if(type == Type::mouseButton) {
    return sf::Mouse::isButtonPressed(mouseButtonBind);
  }
  if(type == Type::mouseWheel) {
    if(*updatedMouseWheelDelta < 0) {
      if(mouseWheelBind == MouseWheelDelta::backward) return true;
    } else if(*updatedMouseWheelDelta > 0) {
      if(mouseWheelBind == MouseWheelDelta::forward) return true;
    }
  }
  if(joystickNumber < sf::Joystick::Count) {
    if(type == Type::controllerButtonDs4) {
      return sf::Joystick::isButtonPressed(joystickNumber, (unsigned int)controllerButtonBindDs4);
    }
  }
  return false;
}

bool Keybind::hit() {
  return hit(-1);
}

bool Keybind::hit(unsigned int joystickNumber) {
  if(frameTag == *updatedFrameTag) {
    return recordedHit;
  }

  frameTag = *updatedFrameTag;
  recordedHit = false;

  if(down(joystickNumber)) {
    if(previouslyDown) return false;
    previouslyDown = true;
    recordedHit = true;
    return true;
  }
  previouslyDown = false;
  return false;
}

float Keybind::axisValue(unsigned int joystickNumber) {
  if(joystickNumber < sf::Joystick::Count) {
    if(type == Type::controllerAxisDs4) {
      float value;
      switch(controllerAxisBindDs4) {
        case ControllerAxisDs4::lLeft:
          value = sf::Joystick::getAxisPosition(joystickNumber, sf::Joystick::X);
          if(value < 0.0f) return -value * 0.01f;
          return 0.0f;
        case ControllerAxisDs4::lRight:
          value = sf::Joystick::getAxisPosition(joystickNumber, sf::Joystick::X);
          if(value > 0.0f) return value * 0.01f;
          return 0.0f;
        case ControllerAxisDs4::lUp:
          value = sf::Joystick::getAxisPosition(joystickNumber, sf::Joystick::Y);
          if(value < 0.0f) return -value * 0.01f;
          return 0.0f;
        case ControllerAxisDs4::lDown:
          value = sf::Joystick::getAxisPosition(joystickNumber, sf::Joystick::Y);
          if(value > 0.0f) return value * 0.01f;
          return 0.0f;
        case ControllerAxisDs4::rLeft:
          value = sf::Joystick::getAxisPosition(joystickNumber, sf::Joystick::U);
          if(value < 0.0f) return -value * 0.01f;
          return 0.0f;
        case ControllerAxisDs4::rRight:
          value = sf::Joystick::getAxisPosition(joystickNumber, sf::Joystick::U);
          if(value > 0.0f) return value * 0.01f;
          return 0.0f;
        case ControllerAxisDs4::rUp:
          value = sf::Joystick::getAxisPosition(joystickNumber, sf::Joystick::V);
          if(value < 0.0f) return -value * 0.01f;
          return 0.0f;
        case ControllerAxisDs4::rDown:
          value = sf::Joystick::getAxisPosition(joystickNumber, sf::Joystick::V);
          if(value > 0.0f) return value * 0.01f;
          return 0.0f;
        case ControllerAxisDs4::l2:
          return 0.005f * (100.0f + sf::Joystick::getAxisPosition(joystickNumber, sf::Joystick::Z));
        case ControllerAxisDs4::r2:
          return 0.005f * (100.0f + sf::Joystick::getAxisPosition(joystickNumber, sf::Joystick::R));
        case ControllerAxisDs4::dpadLeft:
          return float(sf::Joystick::getAxisPosition(joystickNumber, sf::Joystick::PovX) < 0.0f);
        case ControllerAxisDs4::dpadRight:
          return float(sf::Joystick::getAxisPosition(joystickNumber, sf::Joystick::PovX) > 0.0f);
        case ControllerAxisDs4::dpadUp:
          return float(sf::Joystick::getAxisPosition(joystickNumber, sf::Joystick::PovY) < 0.0f);
        case ControllerAxisDs4::dpadDown:
          return float(sf::Joystick::getAxisPosition(joystickNumber, sf::Joystick::PovY) > 0.0f);
        default:
          return 0.0f;
      }
    }
  }
  return 0.0f;
}
