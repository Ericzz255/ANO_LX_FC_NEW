# MaixCAM - 飞控视觉通信协议 V1.0

本文件与 MaixCAM 端冻结版 `MAIXCAM_FLIGHT_CONTROLLER_PROTOCOL_V1.0.md`
保持一致。V1.0 只提供四 AprilTag 小车靶心的跟踪误差，不提供场地绝对
定位；`TYPE 0x02` 为保留类型，双方不得发送。

## 硬件与串口

- 电平：3.3 V TTL，禁止向 MaixCAM 输入 5 V
- 波特率：115200 bit/s
- 格式：8-N-1，无流控
- MaixCAM：UART1 A19/TX、A18/RX
- 飞控：USART1 PA9/TX、PA10/RX

```text
MaixCAM A19 / UART1_TX  -> 飞控 PA10 / USART1_RX
MaixCAM A18 / UART1_RX  <- 飞控 PA9  / USART1_TX
MaixCAM GND             --- 飞控 GND
```

## 通用帧

```text
AA 4D TYPE LEN PAYLOAD[LEN] CRC16_LO CRC16_HI
```

- 多字节整数为小端序。
- `LEN` 最大为 32。
- 相邻字节间隔超过 50 ms 时接收状态机复位。
- CRC 为 CRC-16/CCITT-FALSE：初值 `0xFFFF`，多项式 `0x1021`，
  不反射，无最终异或。
- CRC 覆盖 `TYPE + LEN + PAYLOAD`，不覆盖帧头。

## TYPE 0x01：跟踪结果

方向：MaixCAM -> 飞控；固定 20 Hz；Payload 固定 8 字节。

| 偏移 | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0 | SEQ | uint8 | 每帧递增，包含无效帧 |
| 1 | FLAGS | uint8 | 视觉状态位 |
| 2 | ERROR_X_E4 | int16 | 机体系前向归一化光学误差，前方为正 |
| 4 | ERROR_Y_E4 | int16 | 机体系横向归一化光学误差，左方为正 |
| 6 | QUALITY | uint8 | V1.0 有效时 255，无效时 0 |
| 7 | TAG_MASK | uint8 | 四个配置槽位的检测位图 |

误差换算：

```c
error_x = (float)ERROR_X_E4 / 10000.0f;
error_y = (float)ERROR_Y_E4 / 10000.0f;
```

它们不是像素、厘米或速度指令，必须经过带限幅的控制器。

### FLAGS

| 位 | 名称 | 说明 |
|---:|---|---|
| bit0 | TARGET_VALID | 当前结果可用于控制 |
| bit1 | ALL_FOUR | 四个配置标签全部识别 |
| bit2 | CALIBRATED | 相机标定已加载 |
| bit3 | HELD | 仅显示用旧结果，不可控制 |
| bit4~7 | RESERVED | 必须为 0 |

飞控只在以下条件全部满足时发布有效跟踪数据：

```text
(FLAGS & 0x0F) == 0x07
FLAGS bit4~7 == 0
QUALITY == 255
TAG_MASK == 0x0F
SEQ 不重复
帧龄 <= 150 ms
模式 0x81 应答确认当前为 MODE=1
CRC、TYPE、LEN 和协议自检均正确
```

任一条件不满足时立即停止使用视觉误差，后续控制器必须将视觉修正速度
置零、清除视觉积分并保持飞控自身定点。

固定测试向量：

```text
AA 4D 01 08 01 07 E8 03 0C FE FF 0F CD D4
```

对应 `ERROR_X_E4=+1000`、`ERROR_Y_E4=-500`、CRC=`0xD4CD`。

## TYPE 0x80：设置视觉模式

方向：飞控 -> MaixCAM；Payload 固定 2 字节。

| 偏移 | 字段 | 说明 |
|---:|---|---|
| 0 | SEQ | 命令序号 |
| 1 | MODE | 0=空闲，1=小车跟踪；其他值非法 |

飞控每 500 ms 重发当前请求模式，以便 MaixCAM 重启后自动恢复。

固定测试向量：

```text
AA 4D 80 02 10 01 CA 24
```

## TYPE 0x81：模式状态/应答

方向：MaixCAM -> 飞控；Payload 固定 4 字节。

| 偏移 | 字段 | 说明 |
|---:|---|---|
| 0 | COMMAND_SEQ | 被应答的 0x80 命令序号 |
| 1 | CURRENT_MODE | 当前模式 |
| 2 | RESULT | 0=接受，1=不支持，2=忙 |
| 3 | PROTOCOL_MAJOR | 固定为 1 |

只有命令序号、模式、结果和协议主版本全部匹配，飞控才确认视觉模式。

固定测试向量：

```text
AA 4D 81 04 10 01 00 01 20 2A
```
