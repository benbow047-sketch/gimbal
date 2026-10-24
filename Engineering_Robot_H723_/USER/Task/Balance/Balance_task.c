//
// Created by 24319 on 2026/8/12.
//

#include "Balance_task.h"
#include "DMmotor_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "msg_freertos.h"
#include "PID.h"
#include "drv_dwt.h"
#include <stdio.h>
#include <string.h>

/* -------------------------------- 线程间通讯Topics ------------------------------- */
static subscriber_t *sub_imu_rpy;
static publisher_t  *pub_gimbal_cmd;
static imu_rpy_data_t imu_data;
static gimbal_cmd_t   gimbal_out;
/* -------------------------------- 线程间通讯Topics ------------------------------- */

/* -------------------------------- PID控制器 ------------------------------- */
static pid_obj_t *gimbal_pitch_pid;
static pid_config_t gimbal_pitch_config = INIT_PID_CONFIG(
    GIMBAL_PID_KP, GIMBAL_PID_KI, GIMBAL_PID_KD,
    GIMBAL_PID_INT_LIM, GIMBAL_PID_MAX_OUT,
    PID_Integral_Limit | PID_Trapezoid_Intergral
);
/* -------------------------------- PID控制器 ------------------------------- */

/* -------------------------------- 调试监测 ------------------------------- */
static uint32_t balance_task_dwt = 0;
static float balance_task_dt = 0;
static float balance_task_delta = 0;
static float balance_task_start_dt = 0;
/* -------------------------------- 调试监测 ------------------------------- */

static void Balance_topic_init(void)
{
    sub_imu_rpy   = sub_register("imu_rpy", sizeof(imu_rpy_data_t));
    pub_gimbal_cmd = pub_register("gimbal_cmd", sizeof(gimbal_cmd_t));
}

/* -------------------------------- 线程入口 ------------------------------- */
void BalanceTask_Entry(void const * argument)
{
    Balance_topic_init();

    gimbal_pitch_pid = pid_register(&gimbal_pitch_config);

    memset(&imu_data, 0, sizeof(imu_data));
    memset(&gimbal_out, 0, sizeof(gimbal_out));

    balance_task_dt = dwt_get_delta(&balance_task_dwt);
    balance_task_start_dt = dwt_get_time_ms();

    for(;;)
    {
        balance_task_delta = dwt_get_time_ms() - balance_task_start_dt;
        balance_task_start_dt = dwt_get_time_ms();
        balance_task_dt = dwt_get_delta(&balance_task_dwt);

        /* 订阅IMU姿态数据 */
        if (sub_get_msg(sub_imu_rpy, &imu_data))
        {
            /* 以Pitch角度为反馈,目标为水平(0度),PID输出云台电机目标角度 */
            float pid_out = pid_calculate(gimbal_pitch_pid, imu_data.pitch, GIMBAL_TARGET_PITCH);
            float ff_out = -(imu_data.pitch - GIMBAL_TARGET_PITCH) *
                           GIMBAL_DEG_TO_RAD * GIMBAL_PITCH_FF_GAIN;
            float motor_target_angle = pid_out + ff_out;

            if (motor_target_angle > GIMBAL_PID_MAX_OUT)
                motor_target_angle = GIMBAL_PID_MAX_OUT;
            else if (motor_target_angle < -GIMBAL_PID_MAX_OUT)
                motor_target_angle = -GIMBAL_PID_MAX_OUT;

            /* 限制输出范围并赋值 */
            gimbal_out.pitch_motor_angle = motor_target_angle;
            gimbal_out.yaw_motor_angle   = 0.0f;  // 暂不控制偏航

            /* 发布云台电机控制命令 */
            pub_push_msg(pub_gimbal_cmd, &gimbal_out);
        }

        vTaskDelay(1);
    }
}

