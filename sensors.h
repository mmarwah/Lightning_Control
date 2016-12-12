#ifndef SENSORS_H
#define SENSORS_H
#include "freertos.h"
#include "queue.h"

void vStartSensors( unsigned portBASE_TYPE uxPriority, xQueueHandle xQueue );
void vStartPolling( unsigned portBASE_TYPE uxPriority, xQueueHandle xQueue );

#endif
