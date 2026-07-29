#ifndef __USER_DATA_TRANSFER_H
#define __USER_DATA_TRANSFER_H

/*
 * Anonymous ground-station V7 F1 diagnostics:
 *  1 int16 current X, centimetres
 *  2 int16 current Y, centimetres
 *  3 int16 estimated X velocity, centimetres/second
 *  4 int16 estimated Y velocity, centimetres/second
 *  5 int16 position-valid flag
 *  6 int16 optical-flow image quality
 *  7 int16 MODE1 optical-flow velocity-valid flag
 */
void UserDataTransfer_Task(void);

#endif
