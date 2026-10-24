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
#define GIMBAL_PID_MAX_SPEED 5.0f
#define GIMBAL_PID_INT_LIM  1.0f
#define GIMBAL_TARGET_PITCH 0.0f
#define GIMBAL_CONTROL_DT_MAX 0.05f

void BalanceTask_Entry(void const * argument);

#endif
