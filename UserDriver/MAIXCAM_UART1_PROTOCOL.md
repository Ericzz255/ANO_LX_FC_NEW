# MaixCAM USART1 通信协议

## 硬件参数

- 飞控接口：USART1，PA9/TX、PA10/RX
- 串口参数：115200 bit/s、8 数据位、无校验、1 停止位
- MaixCAM TX 接飞控 PA10，MaixCAM RX 接飞控 PA9，双方必须共地

## 帧格式

`AA 4D TYPE LEN PAYLOAD[LEN] CRC16_LO CRC16_HI`

CRC 使用 CRC16-CCITT-FALSE，初值 `0xFFFF`，多项式 `0x1021`，覆盖
`TYPE`、`LEN` 和完整 Payload。多字节整数均为小端序。

### TYPE 0x01：小车视觉跟踪

固定 Payload 长度 8 字节：

| 偏移 | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0 | SEQ | uint8 | 循环帧序号 |
| 1 | FLAGS | uint8 | bit0=识别到小车/图案 |
| 2 | X | int16 | 图像横向坐标或横向偏差，单位 px |
| 4 | Y | int16 | 图像纵向坐标或纵向偏差，单位 px |
| 6 | CONFIDENCE | uint8 | 识别置信度 0~255 |
| 7 | RESERVED | uint8 | 预留，发送 0 |

X、Y 推荐由 MaixCAM 直接发送“目标中心相对画面中心的偏差”：
目标在画面右侧时 X 为正，目标在画面下方时 Y 为正。这样飞控不依赖
MaixCAM 的分辨率。

### TYPE 0x02：场地定位

固定 Payload 长度 10 字节：

| 偏移 | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0 | SEQ | uint8 | 循环帧序号 |
| 1 | FLAGS | uint8 | bit0=定位有效 |
| 2 | X_CM | int16 | 场地坐标 X，单位 cm |
| 4 | Y_CM | int16 | 场地坐标 Y，单位 cm |
| 6 | YAW_CDEG | int16 | 航向角，单位 0.01° |
| 8 | QUALITY | uint8 | 定位质量 0~255 |
| 9 | RESERVED | uint8 | 预留，发送 0 |

坐标轴方向需要在安装和场地标定后，与飞控当前水平控制坐标系统一。

### TYPE 0x80：飞控设置视觉模式

飞控发送，Payload 长度 2 字节：

| 偏移 | 字段 | 说明 |
|---:|---|---|
| 0 | SEQ | 循环帧序号 |
| 1 | MODE | 0=空闲，1=跟踪，2=定位，3=跟踪+定位 |
