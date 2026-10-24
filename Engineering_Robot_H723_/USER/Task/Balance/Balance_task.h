//
// Created by 24319 on 2026/8/12.
//

#ifndef CTRBOARD_H7_ALL_BALANCE_TASK_H
#define CTRBOARD_H7_ALL_BALANCE_TASK_H

#include "stdint.h"
#include "usart_task.h"

#define GIMBAL_PID_KP       0.2f//0.7f//3.5f
#define GIMBAL_PID_KI       0.0f
#define GIMBAL_PID_KD       0.0043f//0.08f//0.15f
#define GIMBAL_PID_MAX_OUT  1.57f
#define GIMBAL_PID_INT_LIM  1.0f
#define GIMBAL_TARGET_PITCH 0.0f
#define GIMBAL_PITCH_FF_GAIN 1.0f
#define GIMBAL_DEG_TO_RAD    0.01745329252f

void BalanceTask_Entry(void const * argument);

#endif
