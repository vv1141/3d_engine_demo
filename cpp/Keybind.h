#ifndef KEYBIND_H
#define KEYBIND_H

#include <SFML/Window.hpp>

class Keybind {
public:

  enum class MouseWheelDelta {
    forward,
    backward
  };

  enum class ControllerButtonDs4 {
    cross = 0,
    circle,
    triangle,
    square,
    l1,
    r1,
    l2,
    r2,
    share,
    options,
    ps,
    l3,
    r3
  };
  enum class ControllerAxisDs4 {
    lLeft,
    lRight,
    lUp,
    lDown,
    rLeft,
    rRight,
    rUp,
    rDown,
    l2,
    r2,
    dpadLeft,
    dpadRight,
    dpadUp,
    dpadDown
  };

private:

  enum class Type {
    unbound,
    keyboard,
    mouseButton,
    mouseWheel,
    controllerButtonDs4,
    controllerAxisDs4
  };

  Type                type;
  std::string         name;
  sf::Keyboard::Key   keyboardBind;
  sf::Mouse::Button   mouseButtonBind;
  MouseWheelDelta     mouseWheelBind;
  int*                updatedMouseWheelDelta;
  ControllerButtonDs4 controllerButtonBindDs4;
  ControllerAxisDs4   controllerAxisBindDs4;

  std::string         getKeyboardKeyName(const sf::Keyboard::Key key);
  std::string         getMouseButtonName(const sf::Mouse::Button button);
  std::string         getControllerButtonNameDs4(ControllerButtonDs4 button);
  std::string         getControllerAxisNameDs4(ControllerAxisDs4 axis);

  bool                previouslyDown;
  bool                recordedHit;
  int                 frameTag;
  int*                updatedFrameTag;

public:

  Keybind();
  Keybind(sf::Keyboard::Key keyboardBind);
  Keybind(sf::Mouse::Button mouseButtonBind);
  Keybind(MouseWheelDelta mouseWheelDelta);
  Keybind(ControllerButtonDs4 button);
  Keybind(ControllerAxisDs4 axis);
  ~Keybind();

  void        setKeyboardBind(sf::Keyboard::Key keyboardBind);
  void        setMouseButtonBind(sf::Mouse::Button mouseButtonBind);
  void        setMouseWheelBind(MouseWheelDelta mouseWheelDelta);
  void        setControllerButtonBindDs4(ControllerButtonDs4 button);
  void        setControllerAxisBindDs4(ControllerAxisDs4 axis);
  void        setup(int* frameTag, int* updatedMouseWheelDelta);
  void        unbind();
  std::string getBindName();
  bool        down();
  bool        down(unsigned int joystickNumber);
  bool        hit();
  bool        hit(unsigned int joystickNumber);
  float       axisValue(unsigned int joystickNumber);

};

#endif
