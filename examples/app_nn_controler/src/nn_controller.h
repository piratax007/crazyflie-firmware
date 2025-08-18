//
// Created by fausto on 2024-12-10.
//

#ifndef __NN_CONTROLLER_H__
#define __NN_CONTROLLER_H__

#include "stabilizer_types.h"
#include "nn_compute.h"
#include "controller.h"

void rpm2pwm(control_t_n *control_n, int *PWM_0, int *PWM_1, int *PWM_2, int *PWM_3);

#endif //__NN_CONTROLLER_H__
