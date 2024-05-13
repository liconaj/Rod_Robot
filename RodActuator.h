#ifndef RODACTUATOR_H
#define RODACTUATOR_H

#include <Arduino.h>
#include <QuickPID.h>

class RodActuator
{
  
public:

  RodActuator(uint8_t ENA, uint8_t ENB, uint8_t IN1, uint8_t IN2, volatile long &angPosition, void (*interruptHandler)());

  void Stop();
  void Update();
  void SetPIDConstants(float Kp, float Ki, float Kd);
  void Move(float desiredPosition);
  float GetPosition();
  int GetTimeout();
  void SetSpeed(uint8_t maxVelocity);
  void SetLeftLimit(float leftLimit);
  void SetRightLimit(float rightLimit);
  void SetRelativeSystem();
  void SetGlobalSystem();
  void SetTimeout(float timeout);
  bool Next();

private:

  bool isStable(float threshold);
  volatile long &angPosition;
  uint8_t IN1;  // Pin velocidad adelante
  uint8_t IN2;  // Pin velocidad atrás
  uint8_t ENA;  // Pin encoder A
  uint8_t ENB;  // Pin encoder B

  float ticksRev = 7.0; // ticks in angPosition that makes a rev
  float gearRatio = 235.0;  // reducion gear of gearRatio:1
  float R = 5.24;
  float positionFactor;

  uint8_t speed = 2;
  unsigned long timeout = 5000;
  unsigned long timer = millis();
  float position = 0;  // Posición lineal actual
  float lastPosition = 0;
  float desiredPosition = 0; // Posición lineal deseada
  float lastAngvelocity = 0;
  float angVelocity = 0; // Velocidad angular de los motores (interfaz para el PWM)

  bool next = true;
  bool move = false;
  float Kp, Kd, Ki;
  float outputMin = -100;
  float outputMax = 100;
  float outputSpan;
  QuickPID PID; // Controlador PID

  float _leftLimit = -2000;
  float _rightLimit = 2000;

  void updateVelocity();
  void updatePosition();
};

#endif