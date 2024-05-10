#ifndef RODACTUATOR_H
#define RODACTUATOR_H

#include <Arduino.h>
#include <QuickPID.h>
#include <sTune.h>

class RodActuator
{
  
public:

  RodActuator(uint8_t ENA, uint8_t ENB, uint8_t IN1, uint8_t IN2, volatile long &angPosition, void (*interruptHandler)());
  // Destructor de la clase
  void Stop();
  void Update();
  void AutoTune();
  void SetPIDConstants(float Kp, float Ki, float Kd);
  void GetPIDConstants(float &Kp, float &Ki, float &Kd);
  void Move(float* desiredPosition);
  float GetPosition();
  void SetSpeed(uint8_t maxVelocity);
  void SetLeftLimit(float leftLimit);
  void SetRightLimit(float rightLimit);
  void SetRelativeSystem();
  void SetGlobalSystem();
  bool Next();

private:

  bool isStable(float threshold);
  volatile long &_angPosition;
  uint8_t _IN1;  // Pin velocidad adelante
  uint8_t _IN2;  // Pin velocidad atrás
  uint8_t _ENA;  // Pin encoder A
  uint8_t _ENB;  // Pin encoder B

  float ticksRev = 7.0; // ticks in angPosition that makes a rev
  float gearRatio = 235.0;  // reducion gear of gearRatio:1
  float R = 12.0;
  float positionFactor;

  uint8_t _speed = 255;
  int _moveTimeout = 100;
  int _moveTimer = 0;
  float _position = 0;  // Posición lineal actual
  float _lastPosition = 0;
  float * _desiredPosition = 0; // Posición lineal deseada
  float _lastAngvelocity = 0;
  float _angVelocity = 0; // Velocidad angular de los motores (interfaz para el PWM)

  bool _next = true;
  bool _toMove = false;
  float _Kp, _Kd, _Ki;
  float _outputMin = -10;
  float _outputMax = 10;
  float _outputSpan;
  QuickPID _PID; // Controlador PID
  bool _toTune = false;
  sTune _tuner = sTune(&_position, &_angVelocity, _tuner.Mixed_PID, _tuner.directIP, _tuner.printSUMMARY);

  float _leftLimit = -200;
  float _rightLimit = 200;

  void updateVelocity();
  void updatePosition();
  void updateTuner();
};

#endif