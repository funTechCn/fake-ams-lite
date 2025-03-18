#include "main.h"
// #include "pinDefinitions.h"

#include "mbed.h"

#include "multicore.h"

// extern void debug_send_run();

void watchdog_init() {
    watchdog_enable(10000, true); // 1000ms = 1秒
}

void watchdog_feed() {
    // 重置看门狗计时器，防止看门狗超时
    watchdog_update();
}

void setup()
{
    watchdog_init(); 
    USB_debug_init();
    AMCU_init();
}

// extern double distance_count;
// extern void AMCU_bus_run();
void loop()
{
    while (1)
    {
        watchdog_feed();
        USB_debug_run();
        AMCU_run();
    }
}