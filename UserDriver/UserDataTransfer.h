#ifndef __USER_DATA_TRANSFER_H
#define __USER_DATA_TRANSFER_H

/*
 * Anonymous ground-station V7 F1 diagnostics:
 *  1 int16 current X, centimetres
 *  2 int16 current Y, centimetres
 *  3 int16 fresh car pose eligible for control (0=no, 1=yes)
 *  4 int16 SLAM position initialized and fresh flag (0=no, 1=yes)
 *  5 int16 UART1 car-pose link alive flag (0=no, 1=yes)
 *  6 int16 current valid vehicle X, millimetres (0 while invalid)
 *  7 int16 current valid vehicle Y, millimetres (0 while invalid)
 */
void UserDataTransfer_Task(void);

#endif
