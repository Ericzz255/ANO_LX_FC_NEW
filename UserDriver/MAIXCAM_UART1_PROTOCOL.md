# MaixCAM 当前程序与飞控的 UART1 协议

本文件对应 MaixCAM `main.py` 当前实现。MaixCAM 单向发送视觉结果，
飞控不再要求模式命令应答；任务代码中的 `MaixCam_SetMode()` 仅作为
飞控本地视觉接管门控。

## 硬件与串口

- 电平：3.3 V TTL
- 参数：115200 bit/s、8-N-1、无流控
- MaixCAM：UART1 A19/TX、A18/RX
- 飞控：USART1 PA9/TX、PA10/RX
- 发送频率：最高 20 Hz

```text
MaixCAM A19 / UART1_TX  -> 飞控 PA10 / USART1_RX
MaixCAM GND             --- 飞控 GND
```

当前 MaixCAM 程序不读取串口，因此 A18/RX 与飞控 PA9/TX 不参与协议。

## 通用帧

```text
AA 5A TYPE LEN PAYLOAD[LEN] CRC16_LO CRC16_HI
```

- `TYPE=0x21`
- `LEN=13`
- 总帧长固定为 19 字节
- 多字节整数为小端序
- CRC 为 CRC-16/CCITT-FALSE：初值 `0xFFFF`、多项式 `0x1021`、
  不反射、无最终异或
- CRC 覆盖 `TYPE + LEN + PAYLOAD`，不覆盖帧头
- 飞控相邻字节超时为 50 ms，控制数据帧龄上限为 150 ms
- 最近300 ms内收到序号正常的合法帧时，通信链路状态为在线

## 视觉 Payload

| 偏移 | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0 | SEQ | uint8 | 每帧递增，0~255循环 |
| 1 | FLAGS | uint8 | 视觉状态位 |
| 2 | CENTER_X | uint16 | 原图目标中心X像素 |
| 4 | CENTER_Y | uint16 | 原图目标中心Y像素 |
| 6 | ERROR_X_E4 | int16 | 机体系前向归一化误差，前方为正 |
| 8 | ERROR_Y_E4 | int16 | 机体系横向归一化误差，左方为正 |
| 10 | TAG_MASK | uint8 | Tag ID 0~7检测位图 |
| 11 | FPS_X10 | uint16 | 图像帧率乘10 |

误差换算：

```c
error_x = (float)ERROR_X_E4 / 10000.0f;
error_y = (float)ERROR_Y_E4 / 10000.0f;
```

## FLAGS

| 位 | 名称 | 飞控处理 |
|---:|---|---|
| bit0 | TARGET_VALID | 必须为1 |
| bit1 | ALL_FOUR | 四码中心可用于控制 |
| bit2 | CALIBRATED | 必须为1 |
| bit3 | HELD | 拒绝控制 |
| bit4 | SEARCH_ACTIVE | 拒绝控制 |
| bit5 | PARTIAL_CENTERED | 拒绝控制 |
| bit6 | DIAGONAL_TRACK | 有效对角码中心可用于控制 |
| bit7 | RESERVED | 必须为0 |

飞控接受以下两种几何状态：

1. `ALL_FOUR=1`、`DIAGONAL_TRACK=0`、`TAG_MASK=0x0F`；
2. `ALL_FOUR=0`、`DIAGONAL_TRACK=1`，且掩码包含对角组合
   ID 1+2（`0x06`）或 ID 0+3（`0x09`）。

两种状态都必须同时满足 `TARGET_VALID=1`、`CALIBRATED=1`、
`HELD/SEARCH_ACTIVE/PARTIAL_CENTERED=0`、CRC正确、序号不重复、
帧龄不超过150 ms，并且任务已打开本地跟踪门控。

## 无码心跳

没有目标时，MaixCAM仍以最高20 Hz发送合法帧：

```text
CENTER_X = 0xFFFF
CENTER_Y = 0xFFFF
ERROR_X_E4 = 0
ERROR_Y_E4 = 0
TAG_MASK = 0
TARGET_VALID = 0
```

`CALIBRATED=1`只表示标定已加载，不代表当前检测有效。飞控会正常解析
无码心跳、刷新通信统计，同时保持视觉控制无效。

固定无码测试向量：

```text
AA 5A 21 0D 02 04 FF FF FF FF 00 00 00 00 00 C8 00 16 C4
```

## 固定测试向量

```text
AA 5A 21 0D 01 07 40 01 F0 00 E8 03 0C FE 0F C8 00 0B 22
```

对应：

- 中心 `(320, 240)`
- `ERROR_X_E4=+1000`
- `ERROR_Y_E4=-500`
- `TAG_MASK=0x0F`
- `FPS_X10=200`
- CRC=`0x220B`
