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
 * You should have received A copy of the GNU General Public License
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
static int activateNN = 0;

int PWM_NN_0, PWM_NN_1, PWM_NN_2, PWM_NN_3;


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
  if (!RATE_DO_EXECUTE(RATE_200_HZ, tick)) {
    return;
  }

  struct quat q = mkquat(
      state->attitudeQuaternion.x,
      state->attitudeQuaternion.y,
      state->attitudeQuaternion.z,
      state->attitudeQuaternion.w
  );

  euler_angles = quat2rpy(q);

  float omega_roll = sensors->gyro.x;
  float omega_pitch = sensors->gyro.y;
  float omega_yaw = sensors->gyro.z;

  state_array[0] = state->position.x;
  state_array[1] = state->position.y - 3.705f;
  state_array[2] = state->position.z;
  state_array[3] = radians(euler_angles.x);
  state_array[4] = radians(euler_angles.y);
  state_array[5] = radians(euler_angles.z);
  state_array[6] = state->velocity.x;
  state_array[7] = state->velocity.y;
  state_array[8] = state->velocity.z;
  state_array[9] = radians(omega_roll);
  state_array[10] = radians(omega_pitch);
  state_array[11] = radians(omega_yaw);

  if (activateNN == 0) {
    control->motorPwm[0] = 0;
    control->motorPwm[1] = 0;
    control->motorPwm[2] = 0;
    control->motorPwm[3] = 0;
  } else {
    neuralNetworkComputation(&control_n, state_array);

    rpm2pwm(&control_n, &PWM_NN_0, &PWM_NN_1, &PWM_NN_2, &PWM_NN_3);

    control->motorPwm[0] = PWM_NN_0;
    control->motorPwm[1] = PWM_NN_1;
    control->motorPwm[2] = PWM_NN_2;
    control->motorPwm[3] = PWM_NN_3;
  }
}

void rpm2pwm(control_t_n *control_n, int *PWM_NN_0, int *PWM_NN_1, int *PWM_NN_2, int *PWM_NN_3) {
  // const float a = 6.24e-10f;
  // const float b = 2.14e-5f;

  // *PWM_NN_0 = 65535 * (a * (control_n->rpm_0 * (float)control_n->rpm_0) + b * (float)(control_n->rpm_0));
  // *PWM_NN_1 = 65535 * (a * (control_n->rpm_1 * (float)control_n->rpm_1) + b * (float)(control_n->rpm_1));
  // *PWM_NN_2 = 65535 * (a * (control_n->rpm_2 * (float)control_n->rpm_2) + b * (float)(control_n->rpm_2));
  // *PWM_NN_3 = 65535 * (a * (control_n->rpm_3 * (float)control_n->rpm_3) + b * (float)(control_n->rpm_3));

  // const float a = 1.11984693e-07f;
  // const float b = 1.42493452e-03f;
  // const float c = -1.92966300e+00f;

  // *PWM_NN_0 = 65536 * (a * (control_n->rpm_0 * control_n->rpm_0) + b * control_n->rpm_0 + c) / 100;
  // *PWM_NN_1 = 65536 * (a * (control_n->rpm_1 * control_n->rpm_1) + b * control_n->rpm_1 + c) / 100;
  // *PWM_NN_2 = 65536 * (a * (control_n->rpm_2 * control_n->rpm_2) + b * control_n->rpm_2 + c) / 100;
  // *PWM_NN_3 = 65536 * (a * (control_n->rpm_3 * control_n->rpm_3) + b * control_n->rpm_3 + c) / 100;

  // const float a = 4070.3f;
  const float b = 0.2685f;

  *PWM_NN_0 = (control_n->rpm_0 - 4070.3f) / b;
  *PWM_NN_1 = (control_n->rpm_1 - 4070.3f) / b;
  *PWM_NN_2 = (control_n->rpm_2 - 4070.3f) / b;
  *PWM_NN_3 = (control_n->rpm_3 - 4070.3f) / b;
}

PARAM_GROUP_START(nn_controller)
PARAM_ADD(PARAM_INT8, activateNN, &activateNN)
PARAM_GROUP_STOP(nn_controller)

LOG_GROUP_START(ctrlNN)
LOG_ADD(LOG_FLOAT, ob_x, &state_array[0])
LOG_ADD(LOG_FLOAT, ob_y, &state_array[1])
LOG_ADD(LOG_FLOAT, ob_z, &state_array[2])

LOG_ADD(LOG_FLOAT, ob_roll, &state_array[3])
LOG_ADD(LOG_FLOAT, ob_pitch, &state_array[4])
LOG_ADD(LOG_FLOAT, ob_yaw, &state_array[5])

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

LOG_ADD(LOG_INT32, motor_pwm_0, &PWM_NN_0)
LOG_ADD(LOG_INT32, motor_pwm_1, &PWM_NN_1)
LOG_ADD(LOG_INT32, motor_pwm_2, &PWM_NN_2)
LOG_ADD(LOG_INT32, motor_pwm_3, &PWM_NN_3)

LOG_GROUP_STOP(ctrlNN)