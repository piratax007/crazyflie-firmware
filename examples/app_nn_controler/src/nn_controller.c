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
#define DEBUG_MODULE "NNCONTROLLER"
#include "debug.h"

#include "nn_controller.h"
#include "log.h"
#include "param.h"
#include "led.h"

static control_t_n control_n;
struct vec euler_angles;
static float state_array[12];
static float activateNN = 0.0f;

int PWM_0, PWM_1, PWM_2, PWM_3;


// We still need an appMain() function, but we will not really use it. Just let it quietly sleep.
void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

void controllerOutOfTreeInit() {
  control_n.rpm_0 = 0.0f;
  control_n.rpm_1 = 0.0f;
  control_n.rpm_2 = 0.0f;
  control_n.rpm_3 = 0.0f;

  counter = 0;
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
  
  control->controlMode = controlModeNN;
  if (!RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
    return;
  }

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

  rpm2pwm(&control_n, &PWM_0, &PWM_1, &PWM_2, &PWM_3);

  if (counter < 10000) {
    counter++;

    control->motorPwm[0] = 0;
    control->motorPwm[1] = 0;
    control->motorPwm[2] = 0;
    control->motorPwm[3] = 0;
  } else {
    control->motorPwm[0] = PWM_0;
    control->motorPwm[1] = PWM_1;
    control->motorPwm[2] = PWM_2;
    control->motorPwm[3] = PWM_3;
  }
}

void rpm2pwm(control_t_n *control_n, int *PWM_0, int *PWM_1, int *PWM_2, int *PWM_3) {
  const float a = 6.24e-10f;
  const float b = 2.14e-5f;

  *PWM_0 = 65535 * (a * (control_n->rpm_0 * (float)control_n->rpm_0) + b * (float)(control_n->rpm_0));
  *PWM_1 = 65535 * (a * (control_n->rpm_1 * (float)control_n->rpm_1) + b * (float)(control_n->rpm_1));
  *PWM_2 = 65535 * (a * (control_n->rpm_2 * (float)control_n->rpm_2) + b * (float)(control_n->rpm_2));
  *PWM_3 = 65535 * (a * (control_n->rpm_3 * (float)control_n->rpm_3) + b * (float)(control_n->rpm_3));
}

PARAM_GROUP_START(ctrlNN)
/**
 * @brief Activation signal for the NN controller
 */
PARAM_ADD(PARAM_FLOAT, activateNN, &activateNN)
PARAM_GROUP_STOP(ctrlNN)

// LOG_GROUP_START(ctrlNN)
// LOG_ADD(LOG_FLOAT, ob_x, &state_array[0])
// LOG_ADD(LOG_FLOAT, ob_y, &state_array[1])
// LOG_ADD(LOG_FLOAT, ob_z, &state_array[2])

// LOG_ADD(LOG_FLOAT, ob_roll, &state_array[3])
// LOG_ADD(LOG_FLOAT, ob_pitch, &state_array[4])
// LOG_ADD(LOG_FLOAT, ob_yaw, &state_array[5])

// LOG_ADD(LOG_FLOAT, ob_vx, &state_array[6])
// LOG_ADD(LOG_FLOAT, ob_vy, &state_array[7])
// LOG_ADD(LOG_FLOAT, ob_vz, &state_array[8])

// LOG_ADD(LOG_FLOAT, ob_wx, &state_array[9])
// LOG_ADD(LOG_FLOAT, ob_wy, &state_array[10])
// LOG_ADD(LOG_FLOAT, ob_wz, &state_array[11])

// LOG_ADD(LOG_FLOAT, nn_rpm_0, &control_n.rpm_0)
// LOG_ADD(LOG_FLOAT, nn_rpm_1, &control_n.rpm_1)
// LOG_ADD(LOG_FLOAT, nn_rpm_2, &control_n.rpm_2)
// LOG_ADD(LOG_FLOAT, nn_rpm_3, &control_n.rpm_3)

// LOG_GROUP_STOP(ctrlNN)