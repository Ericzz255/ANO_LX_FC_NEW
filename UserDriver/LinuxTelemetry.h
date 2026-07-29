#ifndef __LINUX_TELEMETRY_H
#define __LINUX_TELEMETRY_H

#include "SysConfig.h"

/*
 * Send aircraft position, motion and mission state to the Linux ground
 * station over USART2 at 20 Hz.
 */
void LinuxTelemetry_Send(void);

#endif
