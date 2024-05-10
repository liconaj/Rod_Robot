#ifndef COMMANDS_H
#define COMMANDS_H

#include <SerialCommands.h>
#include "RodActuator.h"
#include "NamedActuators.h"

bool sendingCommand = false;

void endCmd(SerialCommands* sender) {
  sender->GetSerial()->println("!EOC");
}

void nextCommand() {
  if (!sendingCommand) return;
  for (NamedActuator actuator: namedActuators) {
    if (!actuator.object->Next()) {
      return;
    }
  }
  Serial.println("!EOC");
  sendingCommand = false;
}

void cmdUnrecognized(SerialCommands* sender, const char* cmd) {
  sendingCommand = true;

  sender->GetSerial()->print("Unrecognized command [");
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}

void cmdMoveActuator(SerialCommands* sender) {
  sendingCommand = true;
  uint8_t speed = 255;
  char* arg;
  while ((arg = sender->Next()) != NULL)
  {
    char id = arg[0];
    float value = atof(arg + 1);
    if (id == 'F') {
      speed = value;
      continue;
    }
    RodActuator *actuator = getActuatorByName(id);
    actuator->Move(value);
  }

  for (NamedActuator actuator : namedActuators)
  {
    actuator.object->SetSpeed(speed);
  }
}

void cmdGetActuatorPositions(SerialCommands* sender) {
  sendingCommand = true;

  int decimalPlaces = 5;

  String positions = "";
  for (int i=0; i<N_ACTUATORS; ++i) {
    String name = (String) namedActuators[i].name;
    float position = namedActuators[i].object->GetPosition();
    positions += name + ":" + String(position);
    if (i<N_ACTUATORS-1) {{
      positions += " ";
    }}
  }
  sender->GetSerial()->println(positions);
}

void cmdStartAutoTuning(SerialCommands* sender) {
  sendingCommand = true;

  char* arg;
  while ((arg = sender->Next()) != NULL ) {
    char id = arg[0];
    Serial.print("Starting auto tuning in actuator ");
    Serial.println(id);
    RodActuator* actuator = getActuatorByName(id);
    actuator->AutoTune();
  }
}

void cmdSetPIDConstants(SerialCommands* sender) {
  sendingCommand = true;

  float kp = 0;
  float ki = 0;
  float kd = 0;

  char* arg;
  while ((arg = sender->Next()) != NULL ) {
    char id = arg[0];
    float value = atof(arg+1);
    if (id == 'P') kp = value;
    else if (id == 'I') ki = value;
    else if (id == 'D') kd = value;
  }

  for (NamedActuator actuator : namedActuators) {
    actuator.object->SetPIDConstants(kp, ki, kd);
  }

  sender->GetSerial()->print("Kp:");
  sender->GetSerial()->print(kp);
  sender->GetSerial()->print(" Kd:");
  sender->GetSerial()->print(kd);
  sender->GetSerial()->print(" Ki:");
  sender->GetSerial()->println(ki);
}

void cmdDelay(SerialCommands* sender) {
  sendingCommand = true;

  // sender->GetSerial()->println("Starting delay");
  char* arg = sender->Next(); 
  if (arg != NULL) {
    char parameter = arg[0];
    float value = atof(arg+1);
    if (parameter == 'P') {
      delay(value);
    }
  }
}

void cmdGetGCodes(SerialCommands* sender) {
  sendingCommand = true;

  sender->GetSerial()->println("G1: Move actuator");
  sender->GetSerial()->println("G4: Pause robot by a determined P<time>");
}

void cmdGetMCodes(SerialCommands* sender) {
  sendingCommand = true;

  sender->GetSerial()->println("M114: Report actual position of actuators");
  sender->GetSerial()->println("M301: Set PID constants with P<value> I<value> D<value>");
  sender->GetSerial()->println("M303: Starts autotuning");
}


// G-Code Definitions
SerialCommand GHelp("G", &cmdGetGCodes);
SerialCommand G4("G4", &cmdDelay);
SerialCommand G1("G1", &cmdMoveActuator);

// M-Code Definitions
SerialCommand MHelp("M", &cmdGetMCodes);
SerialCommand M114("M114", &cmdGetActuatorPositions);
SerialCommand M303("M303", &cmdStartAutoTuning);
SerialCommand M301("M301", &cmdSetPIDConstants);

#endif