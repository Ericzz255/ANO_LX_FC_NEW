#ifndef __USER_DATA_TRANSFER_H
#define __USER_DATA_TRANSFER_H

/*
 * Anonymous ground-station V7 F1 diagnostics:
 *  1 int16 current X, centimetres
 *  2 int16 current Y, centimetres
 *  3 int16 MaixCAM control mode flag (0=no, 1=yes)
 *  4 int16 SLAM position initialized and fresh flag (0=no, 1=yes)
 *  5 int16 MaixCAM communication link alive flag (0=no, 1=yes)
 */
void UserDataTransfer_Task(void);

#endif
