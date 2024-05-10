#ifndef NAMEDACTUATORS_H
#define NAMEDACTUATORS_H

#include "RodActuator.h"

#define A_ENA 20
#define A_ENB 21
#define A_IN1 2
#define A_IN2 3

#define B_ENA 10
#define B_ENB 10
#define B_IN1 11
#define B_IN2 11

#define C_ENA 12
#define C_ENB 12
#define C_IN1 13
#define C_IN2 13

enum Actuators {
  A, B, C,
  N_ACTUATORS
};

struct NamedActuator {
  char name;
  RodActuator* object;
};


// Posiciones
volatile long angPosition[N_ACTUATORS] = {0, 0, 0};

//
// Interrupt Handlers
//
static void A_interruptHandler() {
  if(digitalRead(A_ENA) != digitalRead(A_ENB)) angPosition[A]++;
  else angPosition[A]--;
}
static void B_interruptHandler() {
  if(digitalRead(B_ENA) != digitalRead(B_ENB)) angPosition[B]++;
  else angPosition[B]--;
}

static void C_interruptHandler() {
  if(digitalRead(C_ENA) != digitalRead(C_ENB)) angPosition[C]++;
  else angPosition[C]--;
}

NamedActuator namedActuators[N_ACTUATORS] = {
  {'A', new RodActuator(A_ENA, A_ENB, A_IN1, A_IN2, angPosition[A], A_interruptHandler)},
  {'B', new RodActuator(B_ENA, B_ENB, B_IN1, B_IN2, angPosition[B], B_interruptHandler)},
  {'C', new RodActuator(C_ENA, C_ENB, C_IN1, C_IN2, angPosition[C], C_interruptHandler)},
};


RodActuator* getActuatorByName(char name) {
  for (int i = 0; i < N_ACTUATORS; ++i) {
    if (namedActuators[i].name == name) {
      return namedActuators[i].object;
    }
  }
  return nullptr;
}

void updateNamedActuators() {
  for (NamedActuator actuator: namedActuators) {
    actuator.object->Update();
  }
}

#endif