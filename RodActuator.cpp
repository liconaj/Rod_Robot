#include <Arduino.h>
#include "RodActuator.h"

#define DEBUG false

RodActuator::RodActuator(uint8_t ENA, uint8_t ENB, uint8_t IN1, uint8_t IN2, volatile long &angPosition, void (*interruptHandler)()) :
  ENA(ENA), ENB(ENB), IN1(IN1), IN2(IN2), angPosition(angPosition)
{
  pinMode(ENA, INPUT);
  pinMode(ENB, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENA), interruptHandler, CHANGE);

  // Calculate position factor
  // 1 Revolución en el encoder son 7 cambios
  // La relación de engranages de 235:1
  // El valor de _position está en revoluciones
  // 1/(7*235) = 6.0790273556231e-4
  positionFactor = 2.0 * R * PI / (ticksRev * gearRatio);

  // Inicializar control PID
  PID = QuickPID(&position, &angVelocity, &desiredPosition);
  PID.SetOutputLimits(outputMin, outputMax);
  //PID.SetDerivativeMode(QuickPID::dMode::dOnError);
  //PID.SetProportionalMode(QuickPID::pMode::pOnError);
  //PID.SetAntiWindupMode(QuickPID::iAwMode::iAwCondition);
  PID.SetMode(QuickPID::Control::automatic);
}

void RodActuator::Update() {
  updatePosition();

  if (move) {
    PID.Compute();
    updateVelocity();
  }

  lastPosition = position;
  lastAngvelocity = angVelocity;

  if (!next && move) {
    unsigned long currentTime = millis(); 
    if (currentTime - timer > timeout) {
      next = true;
    }
  }
}

void RodActuator::Stop() {
  angVelocity = 0;
  next = true;
  move = false;
  updateVelocity();
  updatePosition();
}


void RodActuator::Move(float desiredPosition) {
  next = false;
  timer = millis();
  move = true;
  RodActuator::desiredPosition = desiredPosition;
}

float RodActuator::GetPosition() {
  return position;
}


void RodActuator::SetPIDConstants(float Kp, float Ki, float Kd) {
  Kp = Kp;
  Ki = Ki;
  Kd = Kd;
  PID.SetTunings(Kp, Ki, Kd);
}

bool RodActuator::Next() {
  return next;
}

void RodActuator::SetSpeed(uint8_t speed) {
  RodActuator::speed = speed;
}

void RodActuator::SetTimeout(float timeout) {
  RodActuator::timeout = timeout;
}

int RodActuator::GetTimeout() {
  return timeout;
}

// =========================================
// Private methods
// =========================================
bool RodActuator::isStable(float threshold=1) {
  return abs(position - desiredPosition) <= threshold;
}

void RodActuator::updateVelocity() {
  uint8_t vel1 = 0;
  uint8_t vel2 = 0;
  float minInput = 0.5;
  uint8_t minPWM = 50;
  if (angVelocity >= minInput) {
    //vel1 = abs((float)_angVelocity / (float)_outputMax) * (float)_speed;
    vel1 = map(abs(angVelocity), minInput, outputMax, minPWM, RodActuator::speed);
    
  }
  else if (angVelocity <= -minInput) {
    // vel2 = abs((float)_angVelocity / (float)_outputMax) * (float)_speed;
    vel2 = map(abs(angVelocity), minInput, outputMax, minPWM, RodActuator::speed);
  }
  if (DEBUG && (move && !next)) {
    Serial.print("Velocity: ");
    Serial.print(angVelocity);
    Serial.print(" ");
    Serial.print(vel1);
    Serial.print(" ");
    Serial.print(vel2);
    Serial.print("  Position: ");
    Serial.print(position);
    Serial.println();
  }
  analogWrite(IN1, vel1);
  analogWrite(IN2, vel2);
}

void RodActuator::updatePosition() {
  position = angPosition * positionFactor;
}
