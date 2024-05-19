#ifndef PLAYER_H
#define PLAYER_H

#include "Vehicle.h"
#include "Input.h"

class Player {
private:

  Input::PlayerKeybindProfiles  keybindProfiles;
  Input::Joystick*              joystick;
  Vehicle*                      controlledVehicle;
  Object*                       followedObject;

public:

  Player();
  ~Player();

  void                          setKeybindProfiles(Input::PlayerKeybindProfiles keybindProfiles);
  Input::PlayerKeybindProfiles* getKeybindProfiles();
  void                          setJoystick(Input::Joystick* joystick);
  void                          setControlledVehicle(Vehicle* vehicle);
  Vehicle*                      getControlledVehicle();
  void                          setFollowedObject(Object* object);
  Object*                       getFollowedObject();

  void processLocalInput(Input* input);

};

#endif
