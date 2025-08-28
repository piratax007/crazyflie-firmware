#ifndef __CONTROLLER_NN_H__
#define __CONTROLLER_NN_H__

#include "stabilizer_types.h"
#include "network_evaluate.h"
#include "controller.h"

void thrusts2PWM(control_t_n *control_n, 
    int *PWM_0, int *PWM_1, int *PWM_2, int *PWM_3);

float scale(float v);

float clip(float v, float min, float max);

void controllerNNEnableBigQuad(void);

#endif //__CONTROLLER_NN_H__