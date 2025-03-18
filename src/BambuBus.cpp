#include "BambuBus.h"
#include "CRC16.h"
#include "CRC8.h"

CRC16 crc_16;
CRC8 crc_8;
uint8_t BambuBus_data_buf[1000];
int BambuBus_have_data = 0;

unsigned char now_op_num = 255;
int cmd=0;

// uint16_t BambuBus_address = 0x1200; //0x700 AMS08 ; 0x1200 AMS lite

#define BambuBus_address_AMS08 ((uint16_t)0x700)
#define BambuBus_address_AMSLite ((uint16_t)0x1200)

#define SERSIAL_NUMBER  long_packge_version_serial_number_AMS_lite
#define VERSION_NAME long_packge_version_version_and_name_AMS_lite
struct _filament
{
    // AMS statu
    char ID[8] = "GFG00";
    uint8_t color_R = 0xFF;
    uint8_t color_G = 0xFF;
    uint8_t color_B = 0xFF;
    uint8_t color_A = 0xFF;
    int16_t temperature_min = 220;
    int16_t temperature_max = 240;
    char name[20] = "PETG";
    _filament_status statu = online;
    float meters = 0;

    // printer_set
    _filament_motion_state_set motion_set = waiting;
};
#include "flash.h" // 包含用于Flash操作的函数
#define use_flash_addr (256 * 1024)

struct alignas(FLASH_PAGE_SIZE) flash_save_struct
{
    _filament filament[4][4];
    int BambuBus_now_filament_num = 0;
    uint32_t check = 0x40614061;
} data_save;

bool Bambubus_read()
{
    flash_save_struct *ptr = (flash_save_struct *)(XIP_BASE + use_flash_addr);
    if (ptr->check == 0x40614061)
    {
        memcpy(&data_save, (void *)(XIP_BASE + use_flash_addr), sizeof(data_save));
        return true;
    }
    return false;
}
bool Bambubus_need_to_save = false;
void Bambubus_set_need_to_save()
{
    Bambubus_need_to_save = true;
}
void Bambubus_save()
{
    uint32_t primask = __get_PRIMASK(); // 保存当前中断状态
    __disable_irq();                    // 禁用中断
    flash_range_erase(use_flash_addr, ((sizeof(data_save) / FLASH_SECTOR_SIZE + 1) * FLASH_SECTOR_SIZE));
    flash_range_program(use_flash_addr, (uint8_t *)&data_save, sizeof(data_save));
    __set_PRIMASK(primask); // 恢复之前的中断状态
}

int get_now_filament_num()
{
    return data_save.BambuBus_now_filament_num;
}

void reset_filament_meters(int num)
{
    data_save.filament[num / 4][num % 4].meters = 0;
}
void add_filament_meters(int num, float meters)
{
    data_save.filament[num / 4][num % 4].meters += meters;
}
float get_filament_meters(int num)
{
    return data_save.filament[num / 4][num % 4].meters;
}

bool is_filament_online(int num) {
    return data_save.filament[num / 4][num % 4].statu == online;
}

void set_filament_online(int num, bool if_online)
{
    if (if_online)
        data_save.filament[num / 4][num % 4].statu = online;
    else
        data_save.filament[num / 4][num % 4].statu = offline;
}
_filament_motion_state_set get_filament_motion(int num)
{
    return data_save.filament[num / 4][num % 4].motion_set;
}
void set_filament_motion(int num,_filament_motion_state_set motion)
{
    data_save.filament[num / 4][num % 4].motion_set = motion;
}
uint8_t get_filament_color_A(int num)
{
    return data_save.filament[num / 4][num % 4].color_A;
}

uint8_t get_filament_color_R(int num)
{
    return data_save.filament[num / 4][num % 4].color_R;
}

uint8_t get_filament_color_G(int num)
{
    return data_save.filament[num / 4][num % 4].color_G;
}

uint8_t get_filament_color_B(int num)
{
    return data_save.filament[num / 4][num % 4].color_B;
}




bool TX_IDLE = true;
bool need_debug = false;
void send_uart(const unsigned char *data, uint16_t length)
{
    TX_IDLE = false;
    digitalWrite(BambuBus_pin_de, 1);
    uart_write_blocking(BambuBus_uart, data, length);
    // if (need_debug)
    // {
    //     DEBUG_hex(data, length);
    //     // need_debug = false;
    // }
    uart_tx_wait_blocking(BambuBus_uart);
    digitalWrite(BambuBus_pin_de, 0);
    TX_IDLE = true;
}

uint8_t buf_X[1000]; //AMS-buffer
CRC8 _RX_IRQ_crcx(0x39, 0x66, 0x00, false, false);
void RX_IRQ(unsigned char _RX_IRQ_data)
{
    static int _index = 0;
    static int length = 500;
    static uint8_t data_length_index;
    static uint8_t data_CRC8_index;
    unsigned char data = _RX_IRQ_data;

    if (_index == 0)
    {
        if (data == 0x3D)
        {
            BambuBus_data_buf[0] = 0x3D;
            _RX_IRQ_crcx.restart();
            _RX_IRQ_crcx.add(0x3D);
            data_length_index = 4;
            length = data_CRC8_index = 6;
            _index = 1;
        }
        return;
    }
    else
    {
        BambuBus_data_buf[_index] = data;
        if (_index == 1)
        {
            if (data & 0x80)
            {
                data_length_index = 2;
                data_CRC8_index = 3;
            }
            else
            {
                data_length_index = 4;
                data_CRC8_index = 6;
            }
        }
        if (_index == data_length_index)
        {
            length = data;
        }
        if (_index < data_CRC8_index)
        {
            _RX_IRQ_crcx.add(data);
        }
        else if (_index == data_CRC8_index)
        {
            if (data != _RX_IRQ_crcx.calc())
            {
                _index = 0;
                return;
            }
        }
        ++_index;
        if (_index >= length)
        {
            _index = 0;
            memcpy(buf_X, BambuBus_data_buf, length);
            BambuBus_have_data = length;
        }
        if (_index >= 999)
        {
            _index = 0;
        }
    }
}

// uint8_t _rx_bufx[64];
// uint8_t _tx_bufx[64];
#include <stdio.h>
#include "hardware/uart.h"
#include "hardware/irq.h"

// 数据接收回调
void on_uart_rx()
{
    while (uart_is_readable(BambuBus_uart))
    {
        unsigned char x = uart_getc(BambuBus_uart);
        RX_IRQ(x);
    }
}

//上位机IO初始化
void BambuBUS_UART_Init(uint32_t baudrate)
{
    // 初始化UART
    uart_init(BambuBus_uart, baudrate);
    // 将引脚设置为UART功能
    gpio_set_function(BambuBus_pin_tx, GPIO_FUNC_UART);
    gpio_set_function(BambuBus_pin_rx, GPIO_FUNC_UART);
    // 关闭硬件流
    uart_set_hw_flow(BambuBus_uart, false, false);
    // 设置奇偶位
    uart_set_format(BambuBus_uart, 8, 1, UART_PARITY_EVEN);
    // FIFO
    uart_set_fifo_enabled(BambuBus_uart, true);
    // 注册中断处理函数
    irq_set_exclusive_handler(BambuBus_uart_IRQ, on_uart_rx);
    irq_set_enabled(BambuBus_uart_IRQ, true);
    // 开启接收中断
    uart_set_irq_enables(BambuBus_uart, true, false);
}


//初始化上位通讯
void BambuBus_init()
{
    bool _init_ready = Bambubus_read();
    crc_8.reset(0x39, 0x66, 0, false, false);
    crc_16.reset(0x1021, 0x913D, 0, false, false);
    pinMode(BambuBus_pin_de, OUTPUT);

    if (!_init_ready)
    {
        data_save.filament[0][0].color_R = 0xFF;
        data_save.filament[0][0].color_G = 0x00;
        data_save.filament[0][0].color_B = 0x00;
        data_save.filament[0][1].color_R = 0x00;
        data_save.filament[0][1].color_G = 0xFF;
        data_save.filament[0][1].color_B = 0x00;
        data_save.filament[0][2].color_R = 0x00;
        data_save.filament[0][2].color_G = 0x00;
        data_save.filament[0][2].color_B = 0xFF;
        data_save.filament[0][3].color_R = 0x88;
        data_save.filament[0][3].color_G = 0x88;
        data_save.filament[0][3].color_B = 0x88;
        
        data_save.filament[1][0].color_R = 0xC0;
        data_save.filament[1][0].color_G = 0x20;
        data_save.filament[1][0].color_B = 0x20;
        data_save.filament[1][1].color_R = 0x20;
        data_save.filament[1][1].color_G = 0xC0;
        data_save.filament[1][1].color_B = 0x20;
        data_save.filament[1][2].color_R = 0x20;
        data_save.filament[1][2].color_G = 0x20;
        data_save.filament[1][2].color_B = 0xC0;
        data_save.filament[1][3].color_R = 0x60;
        data_save.filament[1][3].color_G = 0x60;
        data_save.filament[1][3].color_B = 0x60;

        data_save.filament[2][0].color_R = 0x80;
        data_save.filament[2][0].color_G = 0x40;
        data_save.filament[2][0].color_B = 0x40;
        data_save.filament[2][1].color_R = 0x40;
        data_save.filament[2][1].color_G = 0x80;
        data_save.filament[2][1].color_B = 0x40;
        data_save.filament[2][2].color_R = 0x40;
        data_save.filament[2][2].color_G = 0x40;
        data_save.filament[2][2].color_B = 0x80;
        data_save.filament[2][3].color_R = 0x40;
        data_save.filament[2][3].color_G = 0x40;
        data_save.filament[2][3].color_B = 0x40;

        data_save.filament[3][0].color_R = 0x40;
        data_save.filament[3][0].color_G = 0x20;
        data_save.filament[3][0].color_B = 0x20;
        data_save.filament[3][1].color_R = 0x20;
        data_save.filament[3][1].color_G = 0x40;
        data_save.filament[3][1].color_B = 0x20;
        data_save.filament[3][2].color_R = 0x20;
        data_save.filament[3][2].color_G = 0x20;
        data_save.filament[3][2].color_B = 0x40;
        data_save.filament[3][3].color_R = 0x20;
        data_save.filament[3][3].color_G = 0x20;
        data_save.filament[3][3].color_B = 0x20;
    }
    for (auto &i : data_save.filament)
    {
        for (auto &j : i)
        {
            #ifdef _Bambubus_DEBUG_mode_
            j.statu = online;
            #else
            j.statu = offline;
            #endif // DEBUG
            
            j.motion_set = waiting;
        }
    }

    BambuBUS_UART_Init(1228800);
}

bool package_check_crc16(uint8_t *data, int data_length)
{
    crc_16.restart();
    data_length -= 2;
    for (auto i = 0; i < data_length; i++)
    {
        crc_16.add(data[i]);
    }
    uint16_t num = crc_16.calc();
    if ((data[(data_length)] == (num & 0xFF)) && (data[(data_length + 1)] == ((num >> 8) & 0xFF)))
        return true;
    return false;
}

void package_send_with_crc(uint8_t *data, int data_length)
{

    crc_8.restart();
    if (data[1] & 0x80)
    {
        for (auto i = 0; i < 3; i++)
        {
            crc_8.add(data[i]);
        }
        data[3] = crc_8.calc();
    }
    else
    {
        for (auto i = 0; i < 6; i++)
        {
            crc_8.add(data[i]);
        }
        data[6] = crc_8.calc();
    }
    crc_16.restart();
    data_length -= 2;
    for (auto i = 0; i < data_length; i++)
    {
        crc_16.add(data[i]);
    }
    uint16_t num = crc_16.calc();
    data[(data_length)] = num & 0xFF;
    data[(data_length + 1)] = num >> 8;
    data_length += 2;
    send_uart(data, data_length);
}
void package_send_with_crc_debug(uint8_t *data, int data_length)
{

    crc_8.restart();
    if (data[1] & 0x80)
    {
        for (auto i = 0; i < 3; i++)
        {
            crc_8.add(data[i]);
        }
        data[3] = crc_8.calc();
    }
    else
    {
        for (auto i = 0; i < 6; i++)
        {
            crc_8.add(data[i]);
        }
        data[6] = crc_8.calc();
    }
    crc_16.restart();
    data_length -= 2;
    for (auto i = 0; i < data_length; i++)
    {
        crc_16.add(data[i]);
    }
    uint16_t num = crc_16.calc();
    data[(data_length)] = num & 0xFF;
    data[(data_length + 1)] = num >> 8;
    data_length += 2;
    send_uart(data, data_length);
    // DEBUG_num(data, data_length);
}

uint8_t packge_send_buf[1000];

#pragma pack(push, 1) // 将结构体按1字节对齐
struct long_packge_data
{
    uint16_t package_number;
    uint16_t package_length;
    uint8_t crc8;
    uint16_t target_address;
    uint16_t source_address;
    uint16_t type;
    uint8_t *datas;
    uint16_t data_length;
};
#pragma pack(pop) // 恢复默认对齐

void Bambubus_long_package_send(long_packge_data *data)
{
    packge_send_buf[0] = 0x3D;
    packge_send_buf[1] = 0x00;
    data->package_length = data->data_length + 15;
    memcpy(packge_send_buf + 2, data, 11);
    memcpy(packge_send_buf + 13, data->datas, data->data_length);
    package_send_with_crc(packge_send_buf, data->data_length + 15);
}

void Bambubus_long_package_analysis(uint8_t *buf, int data_length, long_packge_data *data)
{
    memcpy(data, buf + 2, 11);
    data->datas = buf + 13;
    data->data_length = data_length - 15; // 最后2字节为CRC16
}


long_packge_data printer_data_long;
package_type get_packge_type(unsigned char *buf, int length)
{
    if (package_check_crc16(buf, length) == false)
    {
        return BambuBus_package_ERROR;
    }
    if (buf[1] == 0xC5)
    {

        switch (buf[4])
        {
        case 0x03:
            return BambuBus_package_filament_motion_short;
        case 0x04:
            return BambuBus_package_filament_motion_long;
        case 0x05:
            return BambuBus_package_online_detect;
        case 0x06:
            return BambuBus_package_REQx6;
        case 0x07:
            return BambuBus_package_NFC_detect;
        case 0x08:
            return BambuBus_package_set_filament;
        case 0x20:
            return BambuBus_package_heartbeat;
        default:
            return BambuBus_package_ETC;
        }
    }
    else if (buf[1] == 0x05)
    {
        Bambubus_long_package_analysis(buf, length, &printer_data_long);
        if (printer_data_long.target_address == BambuBus_address_AMS08)
        {
            printer_data_long.target_address = printer_data_long.target_address;
        }
        else if (printer_data_long.target_address == BambuBus_address_AMSLite)
        {
            printer_data_long.target_address = printer_data_long.target_address;
        }

        switch (printer_data_long.type)
        {
        case 0x21A:
            return BambuBus_long_package_MC_online;
        case 0x211:
            return BambuBus_longe_package_filament;
        case 0x103:
        case 0x402:
            return BambuBus_long_package_version;
        default:
            return BambuBus_package_ETC;
        }
    }
    return BambuBus_package_ERROR;
}
uint8_t package_num = 0;

uint8_t get_filament_left_char(uint8_t AMS_num)
{
    uint8_t data = 0;
    for (int i = 0; i < 4; i++)
    {
        if (data_save.filament[AMS_num][i].statu == online)
        {
            data |= (1 << i) << i; // 1<<(2*i)
        }
    }
    return data;
}

void set_cxx_buf(unsigned char *set_buf, unsigned char *buf, int length)
{
    // unsigned char statu_flags = buf[6];
    unsigned char AMS_num = buf[5];
    unsigned char read_num = buf[7];
    now_op_num=read_num;
    unsigned char fliment_motion_flag = buf[8];
    float meters = -1;

    meters = data_save.filament[AMS_num][read_num].meters;
    uint8_t flagx = 0x02;
    set_buf[2] = flagx;
    set_buf[3] = read_num;

    memcpy(set_buf + 4, &meters, sizeof(meters));

    set_buf[24] = get_filament_left_char(AMS_num);
}
// 3D E0 3C 12 04 00 00 00 00 09 09 09 00 00 00 00 00 00 00
// 02 00 E9 3F 14 BF 00 00 76 03 6A 03 6D 00 E5 FB 99 14 2E 19 6A 03 41 F4 C3 BE E8 01 01 01 01 00 00 00 00 64 64 64 64 0A 27
// 3D E0 2C C9 03 00 00
// 04 01 79 30 61 BE 00 00 03 00 44 00 12 00 FF FF FF FF 00 00 44 00 54 C1 F4 EE E7 01 01 01 01 00 00 00 00 FA 35
#define C_test 0x00, 0x00, 0x00, 0x00, \
               0x00, 0x00, 0x80, 0xBF, \
               0x00, 0x00, 0x00, 0xC0, \
               0x00, 0xC0, 0x5D, 0xFF, \
               0xFC, 0xFF, 0xFC, 0xFF, \
               0x00, 0x00, 0x44, 0x00, \
               0x55,                   \
               0xC1, 0xC3, 0xEC, 0xBC, \
               0x01, 0x01, 0x01, 0x01,
/*
#define C_test 0x00, 0x00, 0x02, 0x01, \
                0xF8, 0x65, 0x30, 0xBF, \
                0x00, 0x00, 0x28, 0x03, \
                0x2A, 0x03, 0x6F, 0x00, \
                0xB6, 0x04, 0xFC, 0xEC, \
                0xDF, 0xE7, 0x44, 0x00, \
                0x04, \
                0xC3, 0xF2, 0xBF, 0xBC, \
                0x01, 0x01, 0x01, 0x01,*/
unsigned char Cxx_res[] = {0x3D, 0xE0, 0x2C, 0x1A, 0x03,
                           C_test 0x00, 0x00, 0x00, 0x00,
                           0x90, 0xE4};
void send_for_Cxx(unsigned char *buf, int length)
{
    Cxx_res[1] = 0xC0 | (package_num << 3);

    set_cxx_buf(Cxx_res + 5, buf, length);

    package_send_with_crc(Cxx_res, sizeof(Cxx_res));
    if (need_debug)DEBUG_hex(Cxx_res, sizeof(Cxx_res));
    if (package_num < 7)
        package_num++;
    else
        package_num = 0;
}
/*
0x00, 0x00, 0x00, 0xFF, // 0x0C...
0x00, 0x00, 0x80, 0xBF, // distance
0x00, 0x00, 0x00, 0xC0,
0x00, 0xC0, 0x5D, 0xFF,
0xFE, 0xFF, 0xFE, 0xFF, // 0xFE, 0xFF, 0xFE, 0xFF,
0x00, 0x44, 0x00, 0x00,
0x10,
0xC1, 0xC3, 0xEC, 0xBC,
0x01, 0x01, 0x01, 0x01,
*/
unsigned char Dxx_res[] = {0x3D, 0xE0, 0x3C, 0x1A, 0x04,
                           0x00, 0x75, 0x01, 0x11,
                           0x04, 0x04, 0x04, 0xFF, // flags
                           0x00, 0x00, 0x00, 0x00,
                           C_test 0x00, 0x00, 0x00, 0x00,
                           0xFF, 0xFF, 0xFF, 0xFF,
                           0x90, 0xE4};
unsigned char Dxx_res2[] = {0x3D, 0xE0, 0x3C, 0x1A, 0x04,
                            0x00, 0x75, 0x01, 0x11,
                            0x0C, 0x04, 0x04, 0x03,
                            0x08, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x03, 0x03,
                            0x5F, 0x6E, 0xD7, 0xBE,
                            0x00, 0x00, 0x03, 0x00,
                            0x44, 0x00, 0x01, 0x00,
                            0xFE, 0xFF, 0xFE, 0xFF,
                            0x00, 0x00, 0x00, 0x00,
                            0x50,
                            0xC1, 0xC3, 0xED, 0xE9,
                            0x01, 0x01, 0x01, 0x01,
                            0x00, 0x00, 0x00, 0x00,
                            0xFF, 0xFF, 0xFF, 0xFF,
                            0xEC, 0xF0};
bool need_res_for_06 = false;
uint8_t res_for_06_num = 0xFF;
int last_detect = 0;
uint8_t filament_flag_detected = 0;
void send_for_Dxx(unsigned char *buf, int length)
{
    unsigned char filament_flag_on = 0x00;
    unsigned char filament_flag_NFC = 0x00;
    unsigned char AMS_num = buf[5];
    unsigned char statu_flags = buf[6];
    unsigned char fliment_motion_flag = buf[7];
    unsigned char read_num = buf[9];
    now_op_num=read_num;
    float meters = -1;
    uint8_t flagx = 0x02;

    for (auto i = 0; i < 4; i++)
    {
        // filament[i].meters;
        if (data_save.filament[AMS_num][i].statu == online)
        {
            filament_flag_on |= 1 << i;
        }
        else if (data_save.filament[AMS_num][i].statu == NFC_waiting)
        {
            filament_flag_on |= 1 << i;
            filament_flag_NFC |= 1 << i;
        }
    }
    if ((read_num != 0xFF)&&(read_num < 4))
    {
        if (printer_data_long.target_address == BambuBus_address_AMS08) // AMS08
        {
            DEBUG("AMS08,err\r\n");
            if (fliment_motion_flag == 0x00)
            {
                switch (statu_flags)
                {
                case 0x07:
                    data_save.BambuBus_now_filament_num = AMS_num*4+read_num;
                    data_save.filament[AMS_num][read_num].motion_set = need_pull_back;
                    break;
                case 0x03:
                    data_save.BambuBus_now_filament_num = AMS_num*4+read_num;
                    data_save.filament[AMS_num][read_num].motion_set = need_send_out;
                    break;
                default:
                    data_save.filament[AMS_num][read_num].motion_set = waiting;
                    break;
                }
            }
        }
        else if (printer_data_long.target_address == BambuBus_address_AMSLite) // AMS lite
        {
            switch (fliment_motion_flag)
            {
            case 0x3F:
                data_save.BambuBus_now_filament_num = AMS_num*4+read_num;
                data_save.filament[AMS_num][read_num].motion_set = need_pull_back;
                break;
            case 0xBF:
                data_save.BambuBus_now_filament_num = AMS_num*4+read_num;
                data_save.filament[AMS_num][read_num].motion_set = need_send_out;
                break;
            default:
                data_save.filament[AMS_num][read_num].motion_set = waiting;
                break;
            }
        }
        meters = data_save.filament[AMS_num][read_num].meters;
    }


    {
        Dxx_res[1] = 0xC0 | (package_num << 3);
        Dxx_res[5] = AMS_num;
        Dxx_res[9] = filament_flag_on;
        Dxx_res[10] = filament_flag_on - filament_flag_NFC;
        Dxx_res[11] = filament_flag_on - filament_flag_NFC;
        Dxx_res[12] = 0xFF;
        Dxx_res[17] = AMS_num;
        Dxx_res[19] = flagx;
        Dxx_res[20] = Dxx_res[12] = buf[9];
        Dxx_res[13] = filament_flag_NFC;
        Dxx_res[41] = get_filament_left_char(AMS_num);
        memcpy(Dxx_res + 21, &meters, sizeof(meters));
    }
    if (last_detect != 0)
    {
        if (last_detect > 10)
        {
            Dxx_res[19] = 0x01;
        }
        else
        {
            Dxx_res[12] = filament_flag_detected;
            Dxx_res[19] = 0x01;
            Dxx_res[20] = filament_flag_detected;
        }
        last_detect--;
    }
    package_send_with_crc(Dxx_res, sizeof(Dxx_res));
    if (need_debug)DEBUG_hex(Cxx_res, sizeof(Cxx_res));
    if (package_num < 7)
        package_num++;
    else
        package_num = 0;
}

// uint8_t online_detect_num2[] = {0x0E, 0x7D, 0x32, 0x31, 0x31, 0x38, 0x15, 0x00, // 序列号？(额外包含之前一位)
//                                 0x36, 0x39, 0x37, 0x33, 0xFF, 0xFF, 0xFF, 0xFF};
// uint8_t online_detect_num[] = {0x90, 0x31, 0x33, 0x34, 0x36, 0x35, 0x02, 0x00, 0x37, 0x39, 0x33, 0x38, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char F00_res[] = {
    0x3D, 0xC0, 0x1D, 0xB4, 0x05, 0x01, 0x00,
    0x16,
    0x0E, 0x7D, 0x32, 0x31, 0x31, 0x38, 0x15, 0x00, 0x36, 0x39, 0x37, 0x33, 0xFF, 0xFF, 0xFF, 0xFF,
    0x00, 0x00, 0x00, 0x33, 0xF0};
void send_for_Fxx(unsigned char *buf, int length)
{
    uint8_t AMS_num;
    /*if ((buf[5] == 0x00))
    {
        if((buf[8]==0x30)&&(buf[9]==0x31))
            BambuBus_device_address=0x700;
        else if((buf[8]==0x30)&&(buf[9]==0x33))
            BambuBus_device_address=0x1200;
        memcpy(F00_res + 4, buf + 4, 3);
        memcpy(F00_res + 8, online_detect_num2, sizeof(online_detect_num2));
        package_send_with_crc(F00_res, sizeof(F00_res));
    }*/

    if ((buf[5] == 0x01) && (buf[6] < 4))
    {
        memcpy(F00_res + 4, buf + 4, 3);
        // memcpy(F00_res + 8, online_detect_num, sizeof(online_detect_num));
        /*
        if (buf[6])
            memcpy(F00_res + 8, online_detect_num, sizeof(online_detect_num));
        else
            memcpy(F00_res + 8, online_detect_num2, sizeof(online_detect_num2));*/
        // memcpy(online_detect_num, buf + 8, sizeof(online_detect_num));
        //  F00_res[5] = buf[5];
        //  F00_res[6] = buf[6];
        package_send_with_crc(F00_res, sizeof(F00_res));
    }
}
// 3D C5 0D F1 07 00 00 00 00 00 00 CE EC
// 3D C0 0D 6F 07 00 00 00 00 00 00 9A 70

// unsigned char NFC_detect_res[] = {0x3D, 0xC0, 0x0D, 0x6F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xE8};
// void send_for_NFC_detect(unsigned char *buf, int length)
// {
//     last_detect = 20;
//     filament_flag_detected = 1 << buf[6];
//     NFC_detect_res[6] = buf[6];
//     NFC_detect_res[7] = buf[7];
//     package_send_with_crc(NFC_detect_res, sizeof(NFC_detect_res));
// }

unsigned char long_packge_MC_online[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void send_for_long_packge_MC_online(unsigned char *buf, int length)
{
    long_packge_data data;
    uint8_t AMS_num = printer_data_long.datas[0];
    Bambubus_long_package_analysis(buf, length, &printer_data_long);
    if (printer_data_long.target_address == BambuBus_address_AMS08){
    }else if (printer_data_long.target_address == BambuBus_address_AMSLite){
    }
    /*else if(printer_data_long.target_address==0x0F00)
    {

    }*/
    else
    {
        return;
    }

    data.datas = long_packge_MC_online;
    data.datas[0] = AMS_num;
    data.data_length = sizeof(long_packge_MC_online);

    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = printer_data_long.target_address;
    data.target_address = printer_data_long.source_address;
    Bambubus_long_package_send(&data);
}
unsigned char long_packge_filament[] =
    {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x47, 0x46, 0x42, 0x30, 0x30, 0x00, 0x00, 0x00,
        0x41, 0x42, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xDD, 0xB1, 0xD4, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x18, 0x01, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void send_for_long_packge_filament(unsigned char *buf, int length)
{
    long_packge_data data;
    Bambubus_long_package_analysis(buf, length, &printer_data_long);

    uint8_t AMS_num = printer_data_long.datas[0];
    uint8_t filament_num = printer_data_long.datas[1];
    long_packge_filament[0] = AMS_num;
    long_packge_filament[1] = filament_num;
    memcpy(long_packge_filament + 19, data_save.filament[AMS_num][filament_num].ID, sizeof(data_save.filament[AMS_num][filament_num].ID));
    memcpy(long_packge_filament + 27, data_save.filament[AMS_num][filament_num].name, sizeof(data_save.filament[AMS_num][filament_num].name));
    long_packge_filament[59] = data_save.filament[AMS_num][filament_num].color_R;
    long_packge_filament[60] = data_save.filament[AMS_num][filament_num].color_G;
    long_packge_filament[61] = data_save.filament[AMS_num][filament_num].color_B;
    long_packge_filament[62] = data_save.filament[AMS_num][filament_num].color_A;
    memcpy(long_packge_filament + 79, &data_save.filament[AMS_num][filament_num].temperature_max, 2);
    memcpy(long_packge_filament + 81, &data_save.filament[AMS_num][filament_num].temperature_min, 2);

    data.datas = long_packge_filament;
    data.data_length = sizeof(long_packge_filament);

    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = printer_data_long.target_address;
    data.target_address = printer_data_long.source_address;
    Bambubus_long_package_send(&data);
}
unsigned char long_packge_version_serial_number_AMS_lite[] = {0x0F,                                                                                     // length
                                                              0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, // AMS lite serial number
                                                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                              0x0E, 0x7D,
                                                              0x32, 0x31, 0x31, 0x38, 0x15, 0x00, 0x36, 0x39, 0x37, 0x33, 0xFF, 0xFF, 0xFF, 0xFF,
                                                              0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBB, 0x44, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
unsigned char long_packge_version_serial_number_AMS08[] = {0x0F,                                                                                     // length
                                                           0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, // AMS08 serial number
                                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                           0x0E, 0x7D,
                                                           0x32, 0x31, 0x31, 0x38, 0x15, 0x00, 0x36, 0x39, 0x37, 0x33, 0xFF, 0xFF, 0xFF, 0xFF,
                                                           0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBB, 0x44, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
unsigned char long_packge_version_version_and_name_AMS_lite[] = {0x00, 0x00, 0x00, 0x00, // verison number
                                                                 0x41, 0x4D, 0x53, 0x5F, 0x46, 0x31, 0x30, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char long_packge_version_version_and_name_AMS08[] = {0x00, 0x00, 0x00, 0x00, // verison number
                                                              0x41, 0x4D, 0x53, 0x30, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void send_for_long_packge_version(unsigned char *buf, int length)
{
    long_packge_data data;
    Bambubus_long_package_analysis(buf, length, &printer_data_long);
    uint8_t AMS_num = printer_data_long.datas[0];
    unsigned char *long_packge_version_serial_number;
    unsigned char *long_packge_version_version_and_name;

    if(printer_data_long.target_address == BambuBus_address_AMS08){
        long_packge_version_serial_number = long_packge_version_serial_number_AMS08;
        long_packge_version_version_and_name = long_packge_version_version_and_name_AMS08;
    }else if(printer_data_long.target_address == BambuBus_address_AMSLite){
        long_packge_version_serial_number = long_packge_version_serial_number_AMS_lite;
        long_packge_version_version_and_name = long_packge_version_version_and_name_AMS_lite;
    }else{
        return;
    }

    switch (printer_data_long.type)
    {
    case 0x402:

        // AMS_num = printer_data_long.datas[33];
        // data.datas = long_packge_version_serial_number;
        // data.data_length = sizeof(long_packge_version_serial_number);
        // data.datas[65] = AMS_num;

        /*AMS_num = printer_data_long.datas[33];
        data.datas = long_packge_version_serial_number;
        data.data_length = sizeof(long_packge_version_serial_number_AMS08);

        data.datas[65] = AMS_num;
        break;*/
        return;
    case 0x103:

        AMS_num = printer_data_long.datas[0];
        data.datas = long_packge_version_version_and_name;
        data.data_length = sizeof(long_packge_version_version_and_name_AMS08);
        data.datas[20] = AMS_num;
        break;
    default:
        return;
    }

    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = printer_data_long.target_address;
    data.target_address = printer_data_long.source_address;
    Bambubus_long_package_send(&data);
}
unsigned char s = 0x01;

unsigned char Set_filament_res[] = {0x3D, 0xC0, 0x08, 0xB2, 0x08, 0x60, 0xB4, 0x04};
void send_for_Set_filament(unsigned char *buf, int length)
{
    uint8_t read_num = buf[5];
    uint8_t AMS_num = read_num&0xF0;
    read_num=read_num&0x0F;
    now_op_num=read_num;
    memcpy(data_save.filament[AMS_num][read_num].ID, buf + 7, sizeof(data_save.filament[AMS_num][read_num].ID));

    data_save.filament[AMS_num][read_num].color_R = buf[15];
    data_save.filament[AMS_num][read_num].color_G = buf[16];
    data_save.filament[AMS_num][read_num].color_B = buf[17];
    data_save.filament[AMS_num][read_num].color_A = buf[18];

    memcpy(&data_save.filament[AMS_num][read_num].temperature_min, buf + 19, 2);
    memcpy(&data_save.filament[AMS_num][read_num].temperature_max, buf + 21, 2);
    memcpy(data_save.filament[AMS_num][read_num].name, buf + 23, sizeof(data_save.filament[AMS_num][read_num].name));
    package_send_with_crc(Set_filament_res, sizeof(Set_filament_res));
    if (need_debug)DEBUG_hex(Set_filament_res, sizeof(Set_filament_res));
    Bambubus_set_need_to_save();
}

int get_cmd_type(){
    return cmd;
}

unsigned char get_now_op_num(){
    return now_op_num;
}

int BambuBus_run()
{
    int stu=0;//online_wait
    static uint64_t time_set=200;
    uint64_t timex=get_time64();
    if(timex>time_set)
    {
        stu=-1;//offline
    }
    if (BambuBus_have_data)
    {
        int data_length = BambuBus_have_data;
        BambuBus_have_data = 0;
        // need_debug = false;
        stu=1;//have_data
        time_set=timex+200;
        cmd = get_packge_type(buf_X, data_length);
        switch (cmd)
        {
        case BambuBus_package_heartbeat://心跳
            break;
        case BambuBus_package_filament_motion_short://
            send_for_Cxx(buf_X, data_length);
            break;
        case BambuBus_package_filament_motion_long:
            send_for_Dxx(buf_X, data_length);
            break;
        case BambuBus_package_online_detect:
            
            send_for_Fxx(buf_X, data_length);
            break;
        case BambuBus_package_REQx6:
            //send_for_REQx6(buf_X, data_length);
            break;
        case BambuBus_long_package_MC_online:
            send_for_long_packge_MC_online(buf_X, data_length);
            break;
        case BambuBus_longe_package_filament://读取耗材信息
            send_for_long_packge_filament(buf_X, data_length);
            break;
        case BambuBus_long_package_version:
            send_for_long_packge_version(buf_X, data_length);
            break;
        case BambuBus_package_NFC_detect:
            // send_for_NFC_detect(buf_X, data_length);
            break;
        case BambuBus_package_set_filament://设置耗材信息
            send_for_Set_filament(buf_X, data_length);
            break;
        default:
            break;
        }
    }
    if (Bambubus_need_to_save)
    {
        Bambubus_save();
        Bambubus_need_to_save = false;
    }
    // HAL_UART_Transmit(&use_Serial.handle,&s,1,1000);

    //NFC_detect_run();
    return stu;
}
