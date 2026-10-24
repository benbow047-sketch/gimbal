//
// Created by 刘嘉俊 on 25-4-9.
//

#ifndef CTRBOARD_H7_ALL_USART_TASK_H
#define CTRBOARD_H7_ALL_USART_TASK_H

#include "stdint.h"

void usart_rx_semaphore_init(void);
void usart_tx_semaphore_init(void);
void USART7_DebugPrintf(const char *format, ...);
void IoT_data_unpack(uint8_t *data, uint16_t len);

/* ------------------------------ IMU姿态角数据包 ------------------------------ */
#define IMU_RX_BUF_SIZE     132         // 44字节/帧(4包)，3帧缓冲          // 11字节/帧，3帧缓冲
#define IMU_PACKET_SIZE     11          // 固定数据包长度
#define IMU_HEADER1         0x55        // 帧头1
#define IMU_HEADER2         0x53        // 帧头2

/**
 * @brief IMU原始数据包（11字节：0x55 0x53 RollL RollH PitchL PitchH YawL YawH VL VH SUM）
 */
typedef struct __attribute__((__packed__))
{
    uint8_t  header1;
    uint8_t  header2;
    int16_t  roll_raw;
    int16_t  pitch_raw;
    int16_t  yaw_raw;
    uint16_t version;
    uint8_t  checksum;
} imu_angle_packet_t;

/**
 * @brief 转换后的RPY角度（度）：Angle = RawValue / 32768 * 180
 */
typedef struct __attribute__((__packed__))
{
    float roll;
    float pitch;
    float yaw;
} imu_rpy_data_t;

void imu_data_unpack(uint8_t *data, uint16_t len);
#endif //CTRBOARD_H7_ALL_USART_TASK_H
