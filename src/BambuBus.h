#pragma once

#include "main.h"
#ifdef __cplusplus
extern "C"
{
#endif

#define BambuBus_uart uart0
#define BambuBus_uart_IRQ UART0_IRQ
#define BambuBus_pin_tx 0
#define BambuBus_pin_rx 1
#define BambuBus_pin_de 6
    enum _filament_status
    {
        offline,
        online,
        NFC_waiting
    };
    enum _filament_motion_state_set
    {
        need_pull_back,
        need_send_out,
        waiting,
        act_send_mm,
        act_pull_mm,
        select_pos,
        release_all,
        cancel
    };

enum package_type
{
    BambuBus_package_filament_motion_short,//0
    BambuBus_package_filament_motion_long,//1
    BambuBus_package_online_detect,//2
    BambuBus_package_REQx6,//3
    BambuBus_package_NFC_detect,//4
    BambuBus_package_set_filament,//5
    BambuBus_long_package_MC_online,//6
    BambuBus_longe_package_filament,//7
    BambuBus_long_package_version,//8
    BambuBus_package_heartbeat,//9
    BambuBus_package_ETC,//10
    BambuBus_package_ERROR,//11
    __BambuBus_package_packge_type_size
};

    extern void BambuBus_init();
    extern int BambuBus_run();


    extern bool Bambubus_read();
    extern void Bambubus_set_need_to_save();
    extern int get_now_filament_num();
    extern void reset_filament_meters(int num);
    extern void add_filament_meters(int num, float meters);
    extern float get_filament_meters(int num);
    extern void set_filament_online(int num, bool if_online);
    extern bool is_filament_online(int num) ;
    extern uint8_t get_filament_color_A(int num);
    extern uint8_t get_filament_color_R(int num);
    extern uint8_t get_filament_color_G(int num);
    extern uint8_t get_filament_color_B(int num);
    extern int get_cmd_type();
    extern unsigned char get_now_op_num();
    extern void set_filament_motion(int num,_filament_motion_state_set motion);
    extern void send_uart(const unsigned char *data, uint16_t length);

    _filament_motion_state_set get_filament_motion(int num);
// #define BambuBus_use_forwarding_Serial
#ifdef BambuBus_use_forwarding_Serial
#define forwarding_Serial Serial4
    extern void BambuBus_run_forward();
#endif
#ifdef __cplusplus
}
#endif