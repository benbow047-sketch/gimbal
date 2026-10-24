#ifndef CTRBOARD_H7_ALL_DMMOTOR_TASK_H
#define CTRBOARD_H7_ALL_DMMOTOR_TASK_H
#include "cmsis_os.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include "fdcan.h"
#include "cmd_task.h"
#include "rc_sbus.h"
#define DEG_TO_RAD(x) ((x) * (M_PI / 180.0f))
#define RAD_TO_DEG(x) ((x) * (180.0f / M_PI))
#define NUM_INITIAL_READINGS 10

// 鍏彿鐢垫満鐨勮搴﹂檺鍒?
#define MOTOR_6_MIN_LIMIT (-1.57f)
#define MOTOR_6_MAX_LIMIT (1.57f)

// 云台俯仰电机(Motor8)角度限制
#define GIMBAL_PITCH_MIN (-1.57f)
#define GIMBAL_PITCH_MAX (1.57f)
// 云台偏航电机(Motor9)角度限制
#define GIMBAL_YAW_MIN   (-3.14f)
#define GIMBAL_YAW_MAX   (3.14f)

// 云台电机运行速度
#define GIMBAL_MOTOR_SPEED 5.0f

// 浜斿彿鐢垫満鐨勮搴﹂檺鍒?
#define MOTOR_5_MIN_LIMIT (-1.65f)  //
#define MOTOR_5_MAX_LIMIT (1.65f)

// 鍥涘彿鐢垫満鐨勮搴﹂檺鍒?
#define MOTOR_4_MIN_LIMIT (-3.14f)
#define MOTOR_4_MAX_LIMIT 3.14f

// 涓夊彿鐢垫満鐨勮搴﹂檺鍒?
#define MOTOR_3_MIN_LIMIT -0.05f
#define MOTOR_3_MAX_LIMIT 2.65f

// 璁＄畻浜屽彿鐢垫満鐨勮搴﹂檺鍒?
#define MOTOR_2_MIN_LIMIT (-2.1f)
#define MOTOR_2_MAX_LIMIT (0.05f)

// 璁＄畻涓�鍙风數鏈虹殑瑙掑害闄愬埗
#define MOTOR_1_MIN_LIMIT (-2.4f)
#define MOTOR_1_MAX_LIMIT 2.4f

void arm_cmd_state_machine(void);
void arm_cmd_enable(void);
void arm_cmd_disable(void);

typedef struct {
    float motor_min_limit;
    float motor_max_limit;
    float initial_offset_rad;

    float current_angle_rad;
    float last_angle_rad;
    int calibrated;
} DMmotorControl;

typedef enum
{
    ARM_DISABLE,
    ARM_ENABLE,
    ARM_INIT,
} arm_mode_e;

struct arm_cmd_msg
{
    arm_mode_e ctrl_mode;
    arm_mode_e last_mode;
};

/** 鍏宠妭鐢垫満瑙掑害閫熷害澶勭悊鐩稿叧鍑芥暟 **/
#define POS_DEADBAND_RAD        0.0002f
#define VEL_DEADBAND_RAD_S      0.009f
#define MEDIAN_WIN_SIZE         5u  // 5绐楀彛涓�兼护娉?

typedef struct
{
    float buf[MEDIAN_WIN_SIZE];
    uint8_t index;
    uint8_t inited;
} median_filter5_t;

/** 鍏宠妭鐢垫満瑙掑害閫熷害澶勭悊鐩稿叧鍑芥暟 **/


// 闄愬箙鍑芥暟
float clamp_radians(float radians, float min_limit, float max_limit);

void DMcontrol_motor_1(hcan_t* hcan, DMmotorControl* motor_control, float target_angle);

void DMcontrol_motor_2(hcan_t* hcan, DMmotorControl* motor_control, float target_angle);

void DMcontrol_motor_3(hcan_t* hcan, DMmotorControl* motor_control, float target_angle);

void DMcontrol_motor_4(hcan_t* hcan, DMmotorControl* motor_control, float target_angle);

void DMcontrol_motor_5(hcan_t* hcan, DMmotorControl* motor_control, float target_angle);

void DMcontrol_motor_6(hcan_t* hcan, DMmotorControl* motor_control, float target_angle);

void DMcontrol_motor_7(hcan_t* hcan,Gripper_mode_e Gripper_ctrl);




/** 云台控制命令 */
typedef struct __attribute__((__packed__))
{
    float pitch_motor_angle;
    float yaw_motor_angle;
} gimbal_cmd_t;

void DMmotorTask_Entry(void const * argument);
#endif //CTRBOARD_H7_ALL_DMMOTOR_TASK_H

