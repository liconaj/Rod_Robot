#include "Commands.h"

// SerialCommands Setup
char serialCommandBuffer[32];
SerialCommands serialCommands(&Serial, serialCommandBuffer, sizeof(serialCommandBuffer));

void addSerialCommands() {
  serialCommands.SetDefaultHandler(cmdUnrecognized);

  serialCommands.AddCommand(&GHelp);
  serialCommands.AddCommand(&G1);
  serialCommands.AddCommand(&G4);

  serialCommands.AddCommand(&MHelp);
  serialCommands.AddCommand(&M114);
  serialCommands.AddCommand(&M303);
  serialCommands.AddCommand(&M301);
}


void setup() {
  Serial.begin(250000);
  addSerialCommands();
}

void loop() {
  serialCommands.ReadSerial();

  updateNamedActuators();
  nextCommand();
}


// LAS CONSTANTES SON Kp=10 Kd=200