#ifndef __USER_DATA_TRANSFER_H
#define __USER_DATA_TRANSFER_H

/*
 * Anonymous ground-station V7 F1 diagnostics:
 *  1 int16 current X, centimetres
 *  2 int16 current Y, centimetres
 *  3 int16 current blind-flight mission step
 *  4 int16 SLAM position initialized and fresh flag (0=no, 1=yes)
 *  5 int16 current blind-flight waypoint index (0..7)
 *  6 int16 current horizontal target X, centimetres
 *  7 int16 current horizontal target Y, centimetres
 *  8 int16 vehicle takeoff flag (0=no request, 1=start request)
 */
void UserDataTransfer_Task(void);

#endif
