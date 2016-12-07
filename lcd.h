#ifndef LCD_H
#define LCD_H
#include "freertos.h"
#include "queue.h"

void vStartLcd( unsigned portBASE_TYPE uxPriority, xQueueHandle xQueue);

#endif
