#pragma once

const uint8_t LEFT_SIDE_LIMIT_SWITCH_PIN = 18;
const uint8_t RIGHT_SIDE_LIMIT_SWITCH_PIN = 19;
const uint8_t RAISE_PULLEY_PIN = 22;
const uint8_t LOWER_PULLEY_PIN = 23;
const uint8_t POWER_MAGNET_PIN = 24;

const int PULLEY_LOWER_PERIOD = 5500;

void InitializePulleyMotorControl();
void LowerPulley();
void RaisePulley();
void StopRaisingPulley();
void PowerOnMagnet();
void PowerOffMagnet();