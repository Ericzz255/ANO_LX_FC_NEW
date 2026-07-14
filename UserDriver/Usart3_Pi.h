/**
 * @file Usart3_Pi.h
 * @brief USART3 树莓派 SLAM 定位数据接口
 */

#ifndef __USART3_PI_H
#define __USART3_PI_H

#include "McuConfig.h"

/**
 * @brief 逐字节解析树莓派定位帧
 * @note 帧格式：45 53 05 SEQ X_H X_L Y_H Y_L CRC_H CRC_L。
 *       CRC16-CCITT初值0xFFFF，多项式0x1021，覆盖LEN和5字节Payload。
 */
void Pi_DataAnl(u8 com_data);

/** 查询新定位帧，返回SET时自动清零标志。 */
u8 Pi_GetData_Flag(void);

/** 复制X高、X低、Y高、Y低四字节到外部缓冲区。 */
void Pi_GetData(u8 *store_array);

#endif
