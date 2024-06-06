#ifndef INPUT_H
#define INPUT_H

#include <GL/glew.h>
#include <SFML/OpenGL.hpp>
#include <SFML/Graphics.hpp>

#include <vector>

#include "Keybind.h"

class Input {
public:
  struct KeybindProfile {
    std::string                    name;
    std::map<std::string, Keybind> keybindings;
  };

  struct PlayerKeybindProfiles {
    KeybindProfile* kbm;
    KeybindProfile* controller;
  };

  struct Joystick {
    bool                         isConnected;
    unsigned int                 number;
    sf::Joystick::Identification identification;

    bool keyDown(std::string keybind);
  };

private:
  bool windowActive;
  bool mouseInWindow;

  int          mouseWheelDelta;
  sf::Vector2i mousePosition;
  sf::Vector2i mouseVelocity;

  bool       closedEvent;
  sf::Uint32 stringInput;
  bool       backspacePressed;
  bool       deletePressed;
  bool       leftArrowPressed;
  bool       rightArrowPressed;

  int                         frameTag;
  std::vector<KeybindProfile> keybindProfiles;

  bool MouseDown[3];
  bool MouseDownMem[3];
  bool MouseHit[3];

  void testMouseHit(int i);
  bool sameJoystickIdentification(unsigned int joystickNumber, sf::Joystick::Identification identification);

public:
  std::vector<Input::Joystick> joysticks;

  Input();
  ~Input();

  void            init();
  void            setCustomKeyBinds();
  KeybindProfile* getDefaultKeybindProfileKbm();
  KeybindProfile* getDefaultKeybindProfileDs4();
  void            update(sf::RenderWindow* renderWindow);

  bool         getLeftMouseHit();
  bool         getRightMouseHit();
  bool         getMiddleMouseHit();
  bool         getLeftMouseDown();
  bool         getRightMouseDown();
  bool         getMiddleMouseDown();
  int          getMouseWheelDelta();
  sf::Vector2i getMouseVelocity();
  int          getMouseX();
  int          getMouseY();
  sf::Vector2i getMousePosition();

  bool keyHit(std::string keybind);
  bool keyHit(std::string keybind, PlayerKeybindProfiles* profile, Input::Joystick* joystick);
  bool keyDown(std::string keybind);
  bool keyDown(std::string keybind, PlayerKeybindProfiles* profile, Input::Joystick* joystick);

  float axisValue(std::string keybind, PlayerKeybindProfiles* profile, Input::Joystick* joystick);

  bool getClosedEvent();

  float combineToAnalogValue(std::string keybind, Input::PlayerKeybindProfiles* keybindProfiles, Input::Joystick* joystick);
  bool  combineToDigitalValue(std::string keybind, Input::PlayerKeybindProfiles* keybindProfiles, Input::Joystick* joystick);
};

#endif
