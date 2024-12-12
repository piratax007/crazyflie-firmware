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
 * nn_controller.c - App layer application of an out of tree controller.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "math3d.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "NNCONTROLLER"
#include "debug.h"

#include "nn_controller.h"
#include "log.h"

static control_t_n control_n;
struct vec euler_angles;
static float state_array[12];

bool print_flag = true;


// We still need an appMain() function, but we will not really use it. Just let it quietly sleep.
void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

// Call the PID controller in this example to make it possible to fly. When you implement you own controller, there is
// no need to include the pid controller.
#include "controller_pid.h"

void controllerOutOfTreeInit() {
  control_n.rpm_0 = 0.0f;
  control_n.rpm_1 = 0.0f;
  control_n.rpm_2 = 0.0f;
  control_n.rpm_3 = 0.0f;

  // Call the PID controller instead in this example to make it possible to fly
  controllerPidInit();
}

bool controllerOutOfTreeTest() {
  return true;
}

void controllerOutOfTree(
    control_t *control,
    const setpoint_t *setpoint,
    const sensorData_t *sensors,
    const state_t *state,
    const uint32_t tick
    ) {
  // Implement your controller here...
  struct quat q = mkquat(
      state->attitudeQuaternion.x,
      state->attitudeQuaternion.y,
      state->attitudeQuaternion.z,
      state->attitudeQuaternion.w
  );

  euler_angles = quat2rpy(q);

  float omega_roll = radians(sensors->gyro.x);
  float omega_pitch = radians(sensors->gyro.y);
  float omega_yaw = radians(sensors->gyro.z);

  state_array[0] = state->position.x;
  state_array[1] = state->position.y;
  state_array[2] = state->position.z;
  state_array[3] = euler_angles.x;
  state_array[4] = euler_angles.y;
  state_array[5] = euler_angles.z;
  state_array[6] = state->velocity.x;
  state_array[7] = state->velocity.y;
  state_array[8] = state->velocity.z;
  state_array[9] = omega_roll;
  state_array[10] = omega_pitch;
  state_array[11] = omega_yaw;

  neuralNetworkComputation(&control_n, state_array);
  

  // Call the PID controller instead in this example to make it possible to fly
  controllerPid(control, setpoint, sensors, state, tick);
}

// PARAM_GROUP_START(ctrlNN)

// PARAM_GROUP_STOP(ctrlNN)

LOG_GROUP_START(ctrlNN)
LOG_ADD(LOG_FLOAT, ob_x, &state_array[0])
LOG_ADD(LOG_FLOAT, ob_y, &state_array[1])
LOG_ADD(LOG_FLOAT, ob_z, &state_array[2])

LOG_ADD(LOG_FLOAT, ob_roll, &state_array[3])
LOG_ADD(LOG_FLOAT, ob_pitch, &state_array[4])
LOG_ADD(LOG_FLOAT, ob_yaw, &state_array[5])

LOG_ADD(LOG_FLOAT, ob_vx, &state_array[6])
LOG_ADD(LOG_FLOAT, ob_vy, &state_array[7])
LOG_ADD(LOG_FLOAT, ob_vz, &state_array[8])

LOG_ADD(LOG_FLOAT, ob_wx, &state_array[0])
LOG_ADD(LOG_FLOAT, ob_wy, &state_array[0])
LOG_ADD(LOG_FLOAT, ob_wz, &state_array[0])

LOG_GROUP_STOP(ctrlNN)