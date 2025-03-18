#include "USB_debug.h"
#include <inttypes.h>
#include "multicore.h"
#include "mbed.h"

uint8_t _USB_debug_BUF_datas[USB_debug_FIFO_size];
FIFO_buffer USB_debug_FIFO;
// uint32_t stack[1000];
mbed::Timer USB_debug_timer;

// 接收缓冲区
#define CMD_BUFFER_SIZE 32
char cmd_buffer[CMD_BUFFER_SIZE]; // 用于存储接收的命令
int cmd_pos = 0;                 // 当前缓冲区位置
bool has_cmd=false;


void USB_debug_init()
{
#ifdef USB_debug_on
    USB_debug_timer.start();
    USB_debug_FIFO = FIFO_buf_by_normal_buf(_USB_debug_BUF_datas, sizeof(_USB_debug_BUF_datas));
    USB_debug_serial.begin(USB_debug_baudrate, USB_debug_format);

#endif
}

uint64_t USB_debug_count64()
{
    return USB_debug_timer.elapsed_time().count();
}

void USB_debug_time()
{
#ifdef USB_debug_on
    unsigned char data[100];
    uint64_t _time64 = USB_debug_timer.elapsed_time().count();
    int i = sprintf((char *)data, "\n[%llu s", _time64 / 1000);
    FIFO_buffer_input_many(&USB_debug_FIFO, data, i);
    i = sprintf((char *)data, "%llu ms]", _time64 % 1000);
    FIFO_buffer_input_many(&USB_debug_FIFO, data, i);
#endif
}

void USB_debug_write(const void *data)
{
#ifdef USB_debug_on
    int i = strlen((const char*)data);
    USB_debug_serial.write((const char *)data,i);
#endif
}
void USB_debug_write_num(const void *data, int num)
{
#ifdef USB_debug_on
    USB_debug_serial.write((const char *)data,num);
#endif
}

#define LINE_LENGTH 64 // 每行显示的字节数
void USB_debug_write_hex(const void *data, int num) {
    const uint8_t* byteData = static_cast<const uint8_t*>(data); // 转换为字节指针
    char output[3 * LINE_LENGTH + 2]; // 每个字节占 3 个字符 (XX + 空格)，外加换行符
    int outputPos = 0;

    for (int i = 0; i < num; i++) {
        // 格式化当前字节为十六进制并添加到输出缓冲区
        outputPos += snprintf(output + outputPos, sizeof(output) - outputPos, "%02X ", byteData[i]);

        // 如果到达了 LINE_LENGTH 或最后一个字节，输出到串口
        if ((i + 1) % LINE_LENGTH == 0 || i + 1 == num) {
            output[outputPos++] = '\n'; // 添加换行符
            output[outputPos] = '\0';  // 确保字符串结束
            USB_debug_serial.write((const char*)output, outputPos);
            // 重置缓冲区位置
            outputPos = 0;
        }
    }
}


extern const char*  USB_debug_get_cmd(); 
extern bool USB_debug_has_cmd(); 
bool USB_debug_has_cmd() {
    return has_cmd;
}
const char* USB_debug_get_cmd() {
    if (has_cmd) {
        has_cmd = false; // 重置标志
        return cmd_buffer;
    }
    return NULL; // 没有新命令时返回 NULL
}

void USB_debug_run()
{
    #ifdef USB_debug_on
    while (USB_debug_serial.available()) {
        char c = USB_debug_serial.read(); // 读取一个字符
        if (c == '\n') {                 // 检测到换行符，表示命令结束
            cmd_buffer[cmd_pos] = '\0';  // 结束字符串
            // process_command(cmd_buffer); // 处理接收到的命令
            has_cmd=true;
            cmd_pos = 0;                 // 重置缓冲区位置
        } else if (cmd_pos < CMD_BUFFER_SIZE - 1) {
            cmd_buffer[cmd_pos++] = c;   // 将字符存入缓冲区
        } else {
            // 缓冲区满，丢弃数据并警告
            USB_debug_write("CMD buffer over\n");
            cmd_pos = 0;
        }
    }
    #endif
}