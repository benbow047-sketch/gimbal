/**
  ******************************************************************************
  * @file    DMmotor_task.c
  * @brief   云台平衡控制线程 — FDCAN2控制两个云台电机(Motor8/Motor9)
  ******************************************************************************
  */
#include <stdio.h>
#include <math.h>
#include "DMmotor_task.h"
#include "drv_dwt.h"
#include "PID.h"
#include "msg_freertos.h"
#include "dm_motor_ctrl.h"
#include "FreeRTOS.h"
#include "task.h"

/* -------------------------------- 线程间通讯Topics ------------------------------- */
static subscriber_t *sub_gimbal_cmd;
static gimbal_cmd_t  gimbal_cmd_recv;
static publisher_t  *pub_gimbal_fb;
static gimbal_cmd_t  gimbal_fb;
/* -------------------------------- 线程间通讯Topics ------------------------------- */

/* -------------------------------- 调试监测 ------------------------------- */
static uint32_t DMmotor_task_dwt = 0;
static float DMmotor_task_dt = 0;
static float DMmotor_task_delta = 0;
static float DMmotor_task_start_dt = 0;
/* -------------------------------- 调试监测 ------------------------------- */

/* 兼容旧代码的外部引用 — 由keyboard.c / cmd_task.c / algorithm_task.c 引用 */
struct arm_cmd_msg arm_cmd = {
    .ctrl_mode = ARM_DISABLE,
    .last_mode = ARM_DISABLE
};
Auto_ctrl_mode auto_ctrl_mode = AUTO_WAIT;

void arm_cmd_state_machine(void)
{
    /* 云台模式下机械臂未启用，空函数保持兼容 */
}

void arm_cmd_enable(void)
{
    arm_cmd.ctrl_mode = ARM_ENABLE;
}

void arm_cmd_disable(void)
{
    arm_cmd.ctrl_mode = ARM_DISABLE;
}
/* ----------------------------------------------------------- */

static void DMmotor_topic_init(void)
{
    sub_gimbal_cmd = sub_register("gimbal_cmd", sizeof(gimbal_cmd_t));
    pub_gimbal_fb  = pub_register("gimbal_fb", sizeof(gimbal_cmd_t));
}

void DMmotorTask_Entry(void const * argument)
{
    DMmotor_topic_init();

    /* FDCAN2使能云台电机 Motor8(俯仰) Motor9(偏航) */
    dm_motor_enable(&hfdcan2, &motor[Motor8]);
    dm_motor_enable(&hfdcan2, &motor[Motor9]);

    memset(&gimbal_cmd_recv, 0, sizeof(gimbal_cmd_recv));
    memset(&gimbal_fb, 0, sizeof(gimbal_fb));

    DMmotor_task_dt = dwt_get_delta(&DMmotor_task_dwt);
    DMmotor_task_start_dt = dwt_get_time_ms();

    for(;;)
    {
        DMmotor_task_delta = dwt_get_time_ms() - DMmotor_task_start_dt;
        DMmotor_task_start_dt = dwt_get_time_ms();
        DMmotor_task_dt = dwt_get_delta(&DMmotor_task_dwt);

        /* 接收云台控制命令 */
        if (sub_get_msg(sub_gimbal_cmd, &gimbal_cmd_recv))
        {
            pos_ctrl(&hfdcan2, motor[Motor8].id,
                     gimbal_cmd_recv.pitch_motor_angle,
                     GIMBAL_MOTOR_SPEED);

            /* 回传实际角度 */
            gimbal_fb.pitch_motor_angle = motor[Motor8].para.pos;
            gimbal_fb.yaw_motor_angle   = motor[Motor9].para.pos;
            pub_push_msg(pub_gimbal_fb, &gimbal_fb);
        }

        vTaskDelay(1);
    }
}
