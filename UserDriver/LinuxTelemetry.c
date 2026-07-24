#include "LinuxTelemetry.h"
#include "ANO_LX.h"
#include "Drv_AnoOf.h"
#include "Drv_RcIn.h"
#include "Drv_Uart.h"
#include "HorizontalControl.h"
#include "LX_FC_State.h"
#include "Path_Planning.h"
#include "User_Task.h"

#define LINUX_TELEMETRY_HEADER_1          0xAAU
#define LINUX_TELEMETRY_HEADER_2          0x55U
#define LINUX_TELEMETRY_TYPE_FLIGHT_STATE 0x01U
#define LINUX_TELEMETRY_PAYLOAD_LENGTH    18U
#define LINUX_TELEMETRY_FRAME_LENGTH      24U

#define LINUX_STATUS_SLAM_VALID           (1U << 0)
#define LINUX_STATUS_HEIGHT_VALID         (1U << 1)
#define LINUX_STATUS_UNLOCKED             (1U << 2)
#define LINUX_STATUS_BARRIERS_READY        (1U << 3)
#define LINUX_STATUS_PATH_READY            (1U << 4)
#define LINUX_STATUS_RC_FAILSAFE           (1U << 5)

static void LinuxTelemetry_PutU16(u8 *buffer, u8 *index, u16 value)
{
    buffer[(*index)++] = (u8)(value & 0xFFU);
    buffer[(*index)++] = (u8)((value >> 8) & 0xFFU);
}

static void LinuxTelemetry_PutS16(u8 *buffer, u8 *index, s16 value)
{
    LinuxTelemetry_PutU16(buffer, index, (u16)value);
}

/* CRC16-CCITT-FALSE: init=0xFFFF, poly=0x1021, no reflection/xorout. */
static u16 LinuxTelemetry_Crc16(const u8 *data, u8 length)
{
    u16 crc = 0xFFFFU;
    u8 i;

    while (length-- > 0U)
    {
        crc ^= (u16)(*data++) << 8;
        for (i = 0; i < 8U; i++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (u16)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

/**
 * @brief 通过 USART2 向 Linux 地面站发送一帧飞行状态遥测数据。
 *
 * @note 本函数由 Ano_Scheduler.c 的 20Hz 任务调用，即每 50ms 发送一次。
 * @note 串口参数为 115200-8-N-1，多字节数据采用小端序。
 * @note 固定帧格式为：
 *       AA 55 TYPE LEN SEQ MODE STEP STATUS
 *       X Y HEIGHT VX VY WAYPOINT TOTAL CRC16
 *
 * 数据来源：
 * - MODE：飞控实际模式 fc_sta.fc_mode_sta；
 * - X/Y：USART3 接收并通过校验的 SLAM 坐标，单位 cm；
 * - HEIGHT：匿名光流/测距模块高度，单位 cm；
 * - VX/VY：飞控内部估计的实际水平速度，单位 cm/s；
 * - STEP：自动任务状态机步骤；
 * - WAYPOINT/TOTAL：当前显示航点和规划航点总数；
 * - STATUS：SLAM、高度、解锁、航线和遥控失控等状态位。
 *
 * Linux 端必须先检查帧头、TYPE、LEN，再对偏移2至最后一个
 * 数据字节计算 CRC16-CCITT-FALSE。CRC正确后才能刷新界面。
 */
void LinuxTelemetry_Send(void)
{
    static u8 sequence = 0;
    u8 frame[LINUX_TELEMETRY_FRAME_LENGTH];
    u8 index = 0;
    u8 status = 0;
    u8 mission_step = UserTask_GetMissionStep();
    u16 height_cm;
    u16 waypoint_index = UserTask_GetWaypointIndex();
    u16 path_length = UserTask_GetPathLength();
    u16 waypoint_progress = 0;
    u16 crc;

    /* bit0：SLAM坐标已经建立，并且最近300ms内仍有新数据。 */
    if (HorizontalControl_HasValidPosition())
    {
        status |= LINUX_STATUS_SLAM_VALID;
    }

    /*
     * bit1：测高模块在线且工作正常。
     * 遥测显示不套用高度控制器的5~500cm安全量程，因此0cm也有效。
     */
    if (ano_of.link_sta != 0U &&
        ano_of.work_sta != 0U)
    {
        status |= LINUX_STATUS_HEIGHT_VALID;
    }

    /* bit2：飞控当前已经解锁。 */
    if (fc_sta.unlock_sta != 0U)
    {
        status |= LINUX_STATUS_UNLOCKED;
    }

    /* bit3：飞控内置的三个固定禁飞区已经就绪。 */
    if (PathPlanner_HasBarrierConfiguration())
    {
        status |= LINUX_STATUS_BARRIERS_READY;
    }

    /* bit4：路径规划已经成功，final_path中存在有效航点。 */
    if (path_length > 0U)
    {
        status |= LINUX_STATUS_PATH_READY;
    }

    /* bit5：遥控接收机处于失控保护状态。 */
    if (rc_in.fail_safe != 0U)
    {
        status |= LINUX_STATUS_RC_FAILSAFE;
    }

    /* 协议高度字段为uint16，避免异常的32位原始值截断回绕。 */
    if (ano_of.of_alt_cm > 65535U)
    {
        height_cm = 65535U;
    }
    else
    {
        height_cm = (u16)ano_of.of_alt_cm;
    }

    /*
     * 生成适合界面直接显示的航点进度：
     * - 开始航点飞行前显示0/N；
     * - 正在飞向第一个航点时显示1/N；
     * - 进入降落阶段后显示N/N。
     */
    if (path_length > 0U)
    {
        if (mission_step == 6U)
        {
            waypoint_progress =
                (waypoint_index < path_length) ?
                (u16)(waypoint_index + 1U) : path_length;
        }
        else if (mission_step >= 7U)
        {
            waypoint_progress = path_length;
        }
    }

    /* 0..1：双字节帧头。 */
    frame[index++] = LINUX_TELEMETRY_HEADER_1;
    frame[index++] = LINUX_TELEMETRY_HEADER_2;
    /* 2：帧类型，0x01表示飞行状态。 */
    frame[index++] = LINUX_TELEMETRY_TYPE_FLIGHT_STATE;
    /* 3：从SEQ到TOTAL的Payload长度，固定18字节。 */
    frame[index++] = LINUX_TELEMETRY_PAYLOAD_LENGTH;
    /* 4：循环帧序号，可供Linux端统计丢帧。 */
    frame[index++] = sequence++;
    /* 5：飞控实际飞行模式。 */
    frame[index++] = fc_sta.fc_mode_sta;
    /* 6：任务状态机当前步骤。 */
    frame[index++] = mission_step;
    /* 7：各类有效性和安全状态位。 */
    frame[index++] = status;
    /* 8..11：SLAM X/Y坐标，int16，单位cm。 */
    LinuxTelemetry_PutS16(frame, &index, now_x);
    LinuxTelemetry_PutS16(frame, &index, now_y);
    /* 12..13：测距高度，uint16，单位cm。 */
    LinuxTelemetry_PutU16(frame, &index, height_cm);
    /* 14..17：飞控估计的X/Y实际速度，int16，单位cm/s。 */
    LinuxTelemetry_PutS16(frame, &index, fc_vel.st_data.vel_x);
    LinuxTelemetry_PutS16(frame, &index, fc_vel.st_data.vel_y);
    /* 18..21：当前显示航点/总航点，uint16。 */
    LinuxTelemetry_PutU16(frame, &index, waypoint_progress);
    LinuxTelemetry_PutU16(frame, &index, path_length);

    /*
     * 22..23：CRC16，小端序。
     * 计算范围为TYPE、LEN和完整Payload，即frame[2]至frame[21]；
     * 帧头AA 55不参与CRC。
     */
    crc = LinuxTelemetry_Crc16(&frame[2],
                               (u8)(index - 2U));
    LinuxTelemetry_PutU16(frame, &index, crc);

    /*
     * 只有组帧长度完全正确时才启动USART2中断发送，防止以后修改字段
     * 时长度宏未同步而发出不完整帧。
     */
    if (index == LINUX_TELEMETRY_FRAME_LENGTH)
    {
        DrvUart2SendBuf(frame, index);
    }
}
