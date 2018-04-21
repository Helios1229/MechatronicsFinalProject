#pragma once
#include <stdint.h>
#include "LineFollower.h"

const uint8_t RAISE_PULLEY_PIN = 22;
const uint8_t LOWER_PULLEY_PIN = 23;
const uint8_t POWER_MAGNET_PIN = 24;

void FiniteStateMachineProcess();
void StartPathFollowing(LineDetectionStructure lineDetectionStruct);
void LineFollowing(LineDetectionStructure lineDetectionStruct, float objectDistanceX, float objectDistanceY);