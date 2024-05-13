#include <Arduino.h>
#include "RodActuator.h"

RodActuator::RodActuator(uint8_t ENA, uint8_t ENB, uint8_t IN1, uint8_t IN2, volatile long &angPosition, void (*interruptHandler)()) :
  _ENA(ENA), _ENB(ENB), _IN1(IN1), _IN2(IN2), _angPosition(angPosition)
{
  pinMode(_ENA, INPUT);
  pinMode(_ENB, INPUT);
  pinMode(_IN1, OUTPUT);
  pinMode(_IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(_ENA), interruptHandler, CHANGE);

  // Calculate position factor
  positionFactor = R * 2.0*PI / (ticksRev * gearRatio); 

  // Inicializar control PID
  _outputSpan = _outputMax - _outputMin;
  _PID = QuickPID(&_position, &_angVelocity, &_desiredPosition);
  _PID.SetSampleTimeUs(_outputSpan * 1000 - 1);
  _PID.SetOutputLimits(_outputMin, _outputMax);
  _PID.SetDerivativeMode(QuickPID::dMode::dOnError);
  _PID.SetProportionalMode(QuickPID::pMode::pOnErrorMeas);
  _PID.SetAntiWindupMode(QuickPID::iAwMode::iAwCondition);
  _PID.SetMode(QuickPID::Control::automatic);
}

void RodActuator::Update() {
  updatePosition();
  // updateVelocity();
  if (_toTune) {
    updateTuner();
  } else {
    _PID.Compute();
  }
  if (_toMove) {
    updateVelocity();
    if (isStable(1) && !_next) {
      _next = true;
    }
    if (_angVelocity == 0.0 && _lastAngvelocity == 0.0) {
       _toMove = false;
       _next = true;
    }
  }
  _lastPosition = _position;
  _lastAngvelocity = _angVelocity;
  //if (!_next) {
  //  _timer++;
  //}
  //if (_timer>_timeout) {
  //  _next = true;
  //  _timer = 0;
  //}
}

void RodActuator::Stop() {
  _angVelocity = 0;
  _toMove = false;
  updateVelocity();
  updatePosition();
}

void RodActuator::AutoTune() {
  _toTune = true;
  
  int positionLimit = 2000;
  const float inputSpan = 1000;
  const float outputStart = 0;
  const float outputStep = _outputMax;
  uint32_t testTimeSec = 5;
  uint32_t settleTimeSec = 1;
  const uint16_t samples = 500;

  _next = false;
  _tuner.Configure(inputSpan, _outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  _tuner.SetEmergencyStop(positionLimit);
}

void RodActuator::Move(float desiredPosition) {
  _toMove = true;
  _next = false;
  _desiredPosition = desiredPosition;
  //_PID.SetSetpoint(&desiredPosition);
}

float RodActuator::GetPosition() {
  return _position;
}


void RodActuator::SetPIDConstants(float Kp, float Ki, float Kd) {
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
  _PID.SetTunings(Kp, Ki, Kd);
}


void RodActuator::GetPIDConstants(float &Kp, float &Ki, float &Kd) {
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
}

bool RodActuator::Next() {
  return _next;
}

void RodActuator::SetSpeed(uint8_t speed) {
  _speed = speed;
}

// =========================================
// Private methods
// =========================================
bool RodActuator::isStable(float threshold=1) {
  return abs(_position - _desiredPosition) <= threshold;
}

void RodActuator::updateVelocity() {
  uint8_t vel1 = 0;
  uint8_t vel2 = 0;
  uint8_t minPWM = 200;
  if (_angVelocity > 0.1) {
    //vel1 = abs((float)_angVelocity / (float)_outputMax) * (float)_speed;
    vel1 = map(abs(_angVelocity), 0.1, _outputMax, minPWM, _speed);
    
  }
  else if (_angVelocity < -0.1) {
    // vel2 = abs((float)_angVelocity / (float)_outputMax) * (float)_speed;
    vel2 = map(abs(_angVelocity), 0.1, _outputMax, minPWM, _speed);
  }
  if (_toTune || _toMove) {
    Serial.print("Velocity: ");
    Serial.print(_angVelocity);
    Serial.print(" ");
    Serial.print(vel1);
    Serial.print(" ");
    Serial.print(vel2);
    Serial.print("  Position: ");
    Serial.print(_position);
    Serial.println();
  }
  analogWrite(_IN1, vel1);
  analogWrite(_IN2, vel2);
}

void RodActuator::updatePosition() {
  // 1 Revolución en el encoder son 7 cambios
  // La relación de engranages de 235:1
  // El valor de _position está en revoluciones
  // 1/(7*235) = 6.0790273556231e-4
  _position = (int)(_angPosition * positionFactor);
}

void RodActuator::updateTuner() {
  switch (_tuner.Run()) {
    case _tuner.sample: // active once per sample during tuning
      updateVelocity();
      break;
    case _tuner.tunings: // active just once per sample during test
      _toTune = false;
      _tuner.GetAutoTunings(&_Kp, &_Ki, &_Kd);
      _PID.SetTunings(_Kp, _Ki, _Kd);
      Stop();
      _next = true;
      break;
  }
}