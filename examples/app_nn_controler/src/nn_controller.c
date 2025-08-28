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
#include "usec_time.h"
#include "led.h"

static control_t_n control_n;
struct vec euler_angles;
static float state_array[12];
static int activateNN = 0;
static uint32_t activationTime;

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
  if (!RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
    return;
  }

  struct quat q = mkquat(
      state->attitudeQuaternion.x,
      state->attitudeQuaternion.y,
      state->attitudeQuaternion.z,
      state->attitudeQuaternion.w
  );

  euler_angles = quat2rpy_xyz(q);

  float omega_roll = sensors->gyro.x;
  float omega_pitch = sensors->gyro.y;
  float omega_yaw = sensors->gyro.z;

  state_array[0] = state->position.x;
  state_array[1] = state->position.y - 3.705f;
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

  if (activateNN == 0) {
    control->motorPwm[0] = 0;
    control->motorPwm[1] = 0;
    control->motorPwm[2] = 0;
    control->motorPwm[3] = 0;
  } else {
    uint64_t startTime = usecTimestamp();
    neuralNetworkComputation(&control_n, state_array);
    activationTime = (uint32_t)(usecTimestamp() - startTime);

    rpm2pwm(&control_n, &PWM_NN_0, &PWM_NN_1, &PWM_NN_2, &PWM_NN_3);

    control->motorPwm[0] = PWM_NN_0;
    control->motorPwm[1] = PWM_NN_1;
    control->motorPwm[2] = PWM_NN_2;
    control->motorPwm[3] = PWM_NN_3;
  }
}

void rpm2pwm(control_t_n *control_n, int *PWM_NN_0, int *PWM_NN_1, int *PWM_NN_2, int *PWM_NN_3) {
  const float a = 4070.3f;
  const float b = 0.2685f;

  *PWM_NN_0 = (control_n->rpm_0 - a) / b;
  *PWM_NN_1 = (control_n->rpm_1 - a) / b;
  *PWM_NN_2 = (control_n->rpm_2 - a) / b;
  *PWM_NN_3 = (control_n->rpm_3 - a) / b;
}

static inline float clampf_pm1(float v) {
  return (v > 1.0f) ? 1.0f : (v < -1.0f) ? -1.0f : v;
}

struct vec quat2rpy_xyz(struct quat q_xyzw) {
  float x = q_xyzw.x;
  float y = q_xyzw.y;
  float z = q_xyzw.z;
  float w = q_xyzw.w;

  float s = w*w + x*x + y*y + z*z;
  if (s > 0.0f) {
    if (fabsf(1.0f - s) > 1e-6f) {
      float inv = 1.0f / sqrtf(s);
      w *= inv;
      x *= inv;
      y *= inv;
      z *= inv;
    }
  }

  struct vec e;

  float sinr_cosp = 2.0f * (w*x + y*z);
  float cosr_cosp = 1.0f - 2.0f * (x*x + y*y);
  e.x = atan2f(sinr_cosp, cosr_cosp);

  float sinp = 2.0f * (w*y - z*x);
  e.y = asinf(clampf_pm1(sinp));

  float siny_cosp = 2.0f * (w*z + x*y);
  float cosy_cosp = 1.0f - 2.0f * (y*y + z*z);
  e.z = atan2f(siny_cosp, cosy_cosp);

  return e;
}

PARAM_GROUP_START(nn_controller)
PARAM_ADD(PARAM_INT8, activateNN, &activateNN)
PARAM_GROUP_STOP(nn_controller)

LOG_GROUP_START(ctrlNN)
LOG_ADD(LOG_UINT32, activationTime, &activationTime)

LOG_ADD(LOG_FLOAT, ob_x, &state_array[0])
LOG_ADD(LOG_FLOAT, ob_y, &state_array[1])
LOG_ADD(LOG_FLOAT, ob_z, &state_array[2])

LOG_ADD(LOG_INT32, motor_pwm_0, &control_n.rpm_0)
LOG_ADD(LOG_INT32, motor_pwm_1, &control_n.rpm_1)
LOG_ADD(LOG_INT32, motor_pwm_2, &control_n.rpm_2)
LOG_ADD(LOG_INT32, motor_pwm_3, &control_n.rpm_3)

LOG_GROUP_STOP(ctrlNN)