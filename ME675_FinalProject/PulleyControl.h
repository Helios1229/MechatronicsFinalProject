#pragma once

const uint8_t RAISE_PULLEY_PIN = 22;
const uint8_t LOWER_PULLEY_PIN = 23;
const uint8_t POWER_MAGNET_PIN = 24;

void InitializePulleyMotorControl();
void LowerPulley();
void RaisePulley();
void PowerOnMagnet();
void PowerOffMagnet();