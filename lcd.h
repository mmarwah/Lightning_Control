#ifndef LCD_H
#define LCD_H

#include "freertos.h"
#include "queue.h"
#include "utility.h"

void vStartLcd( unsigned portBASE_TYPE uxPriority, xQueueHandle xQueue);
void StateCheck(Region_t region);
void drawScreen();
#endif