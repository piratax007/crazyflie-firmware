//
// Created by fausto on 2024-12-10.
//

#ifndef __NN_CONTROLLER_H__
#define __NN_CONTROLLER_H__

#include "stabilizer_types.h"
#include "nn_compute.h"
#include "controller.h"

void rpm2pwm(control_t_n *control_n, int *PWM_0, int *PWM_1, int *PWM_2, int *PWM_3);

struct quat quat_normalize(struct quat q);
struct quat quat_conjugate(struct quat q);
struct quat quat_multiply(struct quat q1, struct quat q2);
struct quat quat_error_xyzw(struct quat q_target, struct quat q_current, bool ensure_pos_w);

struct vec vec_cross_product(struct vec v1, struct vec v2);
struct vec rotate_vector_by_quaternion(struct vec v, struct quat q);

#endif //__NN_CONTROLLER_H__
