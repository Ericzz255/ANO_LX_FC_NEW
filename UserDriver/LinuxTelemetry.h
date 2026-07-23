#ifndef __LINUX_TELEMETRY_H
#define __LINUX_TELEMETRY_H

#include "SysConfig.h"

/*
 * Send one flight-state frame from FC USART2 TX to the Linux ground station.
 * Called by the 20 Hz scheduler; USART2 is configured as 115200-8-N-1.
 */
void LinuxTelemetry_Send(void);

#endif
