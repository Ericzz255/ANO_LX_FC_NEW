# 通信协议索引

本文只提供系统级索引。实现变更时，应同步修改源码、头文件和对应的详细协议文档。

## USART3：SLAM → 飞控

```text
45 53 05 SEQ X_H X_L Y_H Y_L CRC_H CRC_L
```

| 字段 | 说明 |
|---|---|
| `45 53` | 固定帧头 |
| `05` | LEN |
| `SEQ` | 帧序号 |
| `X_H X_L` | 有符号 16 位大端 X，单位 cm |
| `Y_H Y_L` | 有符号 16 位大端 Y，单位 cm |
| `CRC_H CRC_L` | CRC16-CCITT-FALSE，大端发送 |

CRC 覆盖 `LEN、SEQ、X_H、X_L、Y_H、Y_L`，初值 `0xFFFF`、多项式 `0x1021`。解析实现位于 `UserDriver/Usart3_Pi.c`。

## USART1：小车 → 飞控

当前优先使用 V2.1 15 字节帧：

```text
AA 56 X_MM[4] Y_MM[4] TAKEOFF_FLAG CRC16_LE 0D 0A
```

- X/Y：有符号 32 位小端，单位 mm。
- `TAKEOFF_FLAG`：只能为 0 或 1。
- 坐标无效哨兵：`INT32_MIN`。
- 坐标有效超时：150 ms。
- 链路在线超时：300 ms。
- 完整定义：[CAR_POSE_XY_UART_PROTOCOL_V2.0.md](../UserDriver/CAR_POSE_XY_UART_PROTOCOL_V2.0.md)。

兼容的 V2.0 14 字节旧帧没有任务启动权限；收到旧帧时启动标志按 0 处理。

## USART2：飞控 → Linux 地面站

```text
AA 55 TYPE LEN PAYLOAD[LEN] CRC16_LO CRC16_HI
```

当前状态帧固定 22 字节，包含序号、飞控模式、任务步骤、状态位、无人机 X/Y、高度、电池电压和小车 X/Y。串口为 500000 bit/s、20 Hz、8-N-1、小端多字节整数。

完整定义：[LINUX_UART2_PROTOCOL.md](../UserDriver/LINUX_UART2_PROTOCOL.md)。

## 匿名上位机 USERDATA

| 通道 | 含义 | 单位/范围 |
|---:|---|---|
| 1 | 当前无人机 X | cm |
| 2 | 当前无人机 Y | cm |
| 3 | 自动任务步骤 | 0..8、10 |
| 4 | SLAM 已初始化且新鲜 | 0/1 |
| 5 | 当前盲飞航点索引 | 0..7 |
| 6 | 当前水平目标 X | cm |
| 7 | 当前水平目标 Y | cm |
| 8 | 当前小车起飞标志 | 0/1 |

诊断时应成组记录 USERDATA，而不是只看单个坐标值。例如 X/Y 为 0 且 USERDATA4 为 1，可以表示有效的地图原点帧；USERDATA8 为 1 只说明收到启动标志，不说明 SLAM 或小车坐标有效。

## 协议变更检查项

- 帧头、长度和总字节数；
- 字段偏移、符号位、单位和大小端；
- CRC 类型、覆盖范围和发送字节序；
- 发送周期和接收超时；
- 无效哨兵值；
- 链路在线、数据有效和控制许可三种状态的定义；
- 发送端示例帧、接收端解析器和文档是否同步；
- 至少一条黄金帧、坏 CRC 帧和截断帧测试。
