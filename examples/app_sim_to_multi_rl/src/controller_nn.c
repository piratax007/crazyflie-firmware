/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2024 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * out_of_tree_controller.c - App layer application of an out of tree controller.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "MYCONTROLLER"
#include "debug.h"

#include "math3d.h"
#include "controller_nn.h"
#include <math.h>

#define MAX_THRUST 0.15f;
#define A 2.130295e-11f;
#define B 1.032633e-6f;
#define C 5.484560e-4f;

static float maxThrustFactor = 0.70f;
static bool relVel = true;
static bool relOmega = true;
static bool relXYZ = true;
static uint16_t freq = 500;

static control_t_n control_n;
static struct mat33 rot;
static float state_array[18];

static uint32_t usec_eval;


// We still need an appMain() function, but we will not really use it. Just let it quietly sleep.
void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

void controllerOutOfTreeInit() {
  control_n.thrust_0 = 0.0f;
  control_n.thrust_1 = 0.0f;
  control_n.thrust_2 = 0.0f;
  control_n.thrust_3 = 0.0f;
}

float scale(float v) {
  return 0.5f * (v + 1);
}

float clip(float v, float min, float max) {
  if (v < min) return min;
  if (v > max) return max;
  return v;
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
}
