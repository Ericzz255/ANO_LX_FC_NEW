#include "CarPoseXyUart.h"
#include "Drv_Sys.h"
#include <string.h>

/*
 * 小车单片机无线透传协议：
 * AA 55 | X_MM(s32 LE) | Y_MM(s32 LE) | CRC16 LE | 0D 0A
 * CRC-16/CCITT-FALSE覆盖X/Y八个数据字节。
 */

#define CAR_POSE_XY_SOF0                 0xAAU
#define CAR_POSE_XY_SOF1                 0x55U
#define CAR_POSE_XY_EOF0                 0x0DU
#define CAR_POSE_XY_EOF1                 0x0AU
#define CAR_POSE_XY_INTERBYTE_TIMEOUT_MS 100U

static u8 rx_frame[CAR_POSE_XY_FRAME_SIZE];
static u8 rx_index;
static u32 last_byte_ms;
static volatile car_pose_xy_t latest_pose;
static volatile car_pose_xy_uart_stats_t rx_stats;
static u8 control_valid;

static u16 CarPoseXyUart_ReadU16Le(const u8 *data)
{
    return (u16)data[0] | ((u16)data[1] << 8);
}

static u32 CarPoseXyUart_ReadU32Le(const u8 *data)
{
    return (u32)data[0] |
           ((u32)data[1] << 8) |
           ((u32)data[2] << 16) |
           ((u32)data[3] << 24);
}

static s32 CarPoseXyUart_ReadS32Le(const u8 *data)
{
    return (s32)CarPoseXyUart_ReadU32Le(data);
}

static u16 CarPoseXyUart_Crc16(const u8 *data, u16 length)
{
    u16 crc = 0xFFFFU;

    while (length-- != 0U)
    {
        u8 i;

        crc ^= (u16)(*data++) << 8;
        for (i = 0U; i < 8U; i++)
        {
            crc = ((crc & 0x8000U) != 0U) ?
                (u16)((crc << 1) ^ 0x1021U) :
                (u16)(crc << 1);
        }
    }
    return crc;
}

static void CarPoseXyUart_ResetAndResync(u8 data)
{
    rx_index = 0U;
    if (data == CAR_POSE_XY_SOF0)
    {
        rx_frame[0] = data;
        rx_index = 1U;
    }
}

static void CarPoseXyUart_ProcessFrame(void)
{
    u16 received_crc;
    u16 calculated_crc;
    s32 x_mm;
    s32 y_mm;
    u32 now_ms;

    if (rx_frame[12] != CAR_POSE_XY_EOF0 ||
        rx_frame[13] != CAR_POSE_XY_EOF1)
    {
        rx_stats.eof_errors++;
        return;
    }

    received_crc = CarPoseXyUart_ReadU16Le(&rx_frame[10]);
    calculated_crc = CarPoseXyUart_Crc16(&rx_frame[2], 8U);
    if (received_crc != calculated_crc)
    {
        rx_stats.crc_errors++;
        return;
    }

    now_ms = GetSysRunTimeMs();
    rx_stats.last_frame_ms = now_ms;
    x_mm = CarPoseXyUart_ReadS32Le(&rx_frame[2]);
    y_mm = CarPoseXyUart_ReadS32Le(&rx_frame[6]);

    if (x_mm == CAR_POSE_XY_INVALID_VALUE ||
        y_mm == CAR_POSE_XY_INVALID_VALUE)
    {
        rx_stats.invalid_frames++;
        control_valid = RESET;
        return;
    }

    rx_stats.valid_frames++;
    latest_pose.x_mm = x_mm;
    latest_pose.y_mm = y_mm;
    latest_pose.receive_ms = now_ms;
    latest_pose.update_count = rx_stats.valid_frames;
    control_valid = SET;
}

u8 CarPoseXyUart_ProtocolSelfTest(void)
{
    static const u8 test_frame[CAR_POSE_XY_FRAME_SIZE] =
    {
        0xAAU, 0x55U, 0xE8U, 0x03U, 0x00U, 0x00U, 0x0CU,
        0xFEU, 0xFFU, 0xFFU, 0x31U, 0x2DU, 0x0DU, 0x0AU
    };

    return (CarPoseXyUart_Crc16(&test_frame[2], 8U) == 0x2D31U &&
            CarPoseXyUart_ReadS32Le(&test_frame[2]) == 1000L &&
            CarPoseXyUart_ReadS32Le(&test_frame[6]) == -500L) ?
        SET : RESET;
}

void CarPoseXyUart_Init(void)
{
    memset(rx_frame, 0, sizeof(rx_frame));
    memset((void *)&latest_pose, 0, sizeof(latest_pose));
    memset((void *)&rx_stats, 0, sizeof(rx_stats));
    rx_index = 0U;
    last_byte_ms = 0U;
    control_valid = RESET;
    rx_stats.self_test_pass = CarPoseXyUart_ProtocolSelfTest();
}

void CarPoseXyUart_GetOneByte(u8 data)
{
    u32 now_ms = GetSysRunTimeMs();

    rx_stats.received_bytes++;

    if (rx_index != 0U &&
        (u32)(now_ms - last_byte_ms) >
            CAR_POSE_XY_INTERBYTE_TIMEOUT_MS)
    {
        rx_stats.interbyte_timeouts++;
        rx_index = 0U;
    }
    last_byte_ms = now_ms;

    if (rx_index == 0U)
    {
        if (data == CAR_POSE_XY_SOF0)
        {
            rx_frame[0] = data;
            rx_index = 1U;
        }
        return;
    }

    if (rx_index == 1U)
    {
        if (data == CAR_POSE_XY_SOF1)
        {
            rx_frame[1] = data;
            rx_index = 2U;
        }
        else
        {
            CarPoseXyUart_ResetAndResync(data);
        }
        return;
    }

    rx_frame[rx_index++] = data;
    if (rx_index >= CAR_POSE_XY_FRAME_SIZE)
    {
        CarPoseXyUart_ProcessFrame();
        rx_index = 0U;
    }
}

void CarPoseXyUart_Task(void)
{
    if (control_valid != RESET &&
        (u32)(GetSysRunTimeMs() - rx_stats.last_frame_ms) >
            CAR_POSE_XY_VALID_TIMEOUT_MS)
    {
        control_valid = RESET;
    }
}

u8 CarPoseXyUart_IsControlValid(void)
{
    CarPoseXyUart_Task();
    return (control_valid != RESET &&
            rx_stats.self_test_pass != RESET) ? SET : RESET;
}

u8 CarPoseXyUart_GetPose(car_pose_xy_t *pose)
{
    if (pose == 0 || CarPoseXyUart_IsControlValid() == RESET)
    {
        return RESET;
    }

    pose->x_mm = latest_pose.x_mm;
    pose->y_mm = latest_pose.y_mm;
    pose->receive_ms = latest_pose.receive_ms;
    pose->update_count = latest_pose.update_count;
    return SET;
}

u8 CarPoseXyUart_IsLinkAlive(void)
{
    return (rx_stats.self_test_pass != RESET &&
            rx_stats.last_frame_ms != 0U &&
            (u32)(GetSysRunTimeMs() - rx_stats.last_frame_ms) <=
                CAR_POSE_XY_LINK_TIMEOUT_MS) ? SET : RESET;
}

u8 CarPoseXyUart_HasReceivedData(void)
{
    return (rx_stats.received_bytes != 0U) ? SET : RESET;
}

void CarPoseXyUart_GetStats(car_pose_xy_uart_stats_t *stats)
{
    if (stats == 0)
    {
        return;
    }

    stats->received_bytes = rx_stats.received_bytes;
    stats->valid_frames = rx_stats.valid_frames;
    stats->crc_errors = rx_stats.crc_errors;
    stats->eof_errors = rx_stats.eof_errors;
    stats->invalid_frames = rx_stats.invalid_frames;
    stats->interbyte_timeouts = rx_stats.interbyte_timeouts;
    stats->last_frame_ms = rx_stats.last_frame_ms;
    stats->self_test_pass = rx_stats.self_test_pass;
}
