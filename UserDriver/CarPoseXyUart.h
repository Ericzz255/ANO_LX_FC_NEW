#ifndef _CAR_POSE_XY_UART_H_
#define _CAR_POSE_XY_UART_H_

#include "SysConfig.h"

/* 小车单片机 -> 无线透传 -> 飞控USART1，固定14字节X/Y毫米坐标帧。 */

#define CAR_POSE_XY_FRAME_SIZE       14U
#define CAR_POSE_XY_VALID_TIMEOUT_MS 150U
#define CAR_POSE_XY_LINK_TIMEOUT_MS  300U
#define CAR_POSE_XY_INVALID_VALUE    ((s32)0x80000000UL)

typedef struct
{
    s32 x_mm;
    s32 y_mm;
    u32 receive_ms;
    u32 update_count;
} car_pose_xy_t;

typedef struct
{
    u32 received_bytes;
    u32 valid_frames;
    u32 crc_errors;
    u32 eof_errors;
    u32 invalid_frames;
    u32 interbyte_timeouts;
    u32 last_frame_ms;
    u8 self_test_pass;
} car_pose_xy_uart_stats_t;

void CarPoseXyUart_Init(void);
void CarPoseXyUart_GetOneByte(u8 data);
void CarPoseXyUart_Task(void);
u8 CarPoseXyUart_GetPose(car_pose_xy_t *pose);
u8 CarPoseXyUart_IsControlValid(void);
u8 CarPoseXyUart_IsLinkAlive(void);
u8 CarPoseXyUart_HasReceivedData(void);
void CarPoseXyUart_GetStats(car_pose_xy_uart_stats_t *stats);
u8 CarPoseXyUart_ProtocolSelfTest(void);

#endif
