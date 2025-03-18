#pragma once


#include "main.h"
#include "BambuBus.h"
#ifdef __cplusplus
extern "C"
{
#endif

// #define AMCU_uart uart1
// #define AMCU_uart_IRQ UART1_IRQ
// #define AMCU_pin_tx 4
// #define AMCU_pin_rx 5

#define AMCU_AS5600_SDA 2
#define AMCU_AS5600_SCL 3

void set_color(u_int8_t r, u_int8_t g, u_int8_t b, u_int8_t a);
void rgb_set_breath(int16_t time,uint16_t count);
void selectOnePos(char num);
void releaseAllPos();
void set_LED_state(char led, char state);
void debug_info(bool force);

extern void AMCU_init();
extern void AMCU_run();
extern void AMCU_motion();

#ifdef __cplusplus
}
#endif

