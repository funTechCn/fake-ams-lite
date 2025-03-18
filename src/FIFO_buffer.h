#ifndef __FIFO_BUFFER_H
#define __FIFO_BUFFER_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct _FIFO_buffer
    {
        unsigned char *buffer;     // 数据区地址
        unsigned char *buffer_END; // 数据区结尾地址(在数据区中)
        unsigned char *O_ptr;      // 读取指针地址
        unsigned char *I_ptr;      // 写入指针地址
        unsigned char ERROR;       // 记录错误的变量：0:无错误，1:写入时发现数据区全满，2:读取时发现数据队列为空
    } FIFO_buffer;

    /*对输入的一维数组创造一个FIFO流控方法，实际数据保存在数组中，流控指针保存在FIFO结构中。
    buf_ADR :   存储实际数据的一维数组
    _size :     数组长度
    */
    FIFO_buffer FIFO_buf_by_normal_buf(unsigned char *buf_ADR, unsigned int _size);

    /*向FIFO流中放入一个字节
    buf : FIFO流控结构体的地址
    dat : 数据
    */
    void FIFO_buffer_input_byte(FIFO_buffer *buf, const unsigned char dat);

    /*宏函数:快速在FIFO流中放入一个字节，可用于中断的快速处理
    FIFO:   FIFO流控结构体
    dat:    放入的数据
    */
#define FIFO_buffer_fast_input(FIFO, dat) \
    {                                     \
        unsigned char *In = FIFO.I_ptr;   \
        *(In) = dat;                      \
        if (In == FIFO.buffer_END)        \
            In = FIFO.buffer;             \
        else                              \
            In++;                         \
            FIFO.I_ptr = In;              \
    }

    /*向FIFO流中放入多个字节
    buf :       FIFO流控结构体的地址
    buf_in :    输入的多字节数组
    len :       输入长度
    */
    void FIFO_buffer_input_many(FIFO_buffer *buf, unsigned char *buf_in, unsigned int len);

    /*从FIFO流中取出一个字节
    buf :   FIFO流控结构体的地址
    return: 取出的数据
    */
    unsigned char FIFO_buffer_output_byte(FIFO_buffer *buf);

    /*宏函数:快速在FIFO流中取出一个字节，可用于中断的快速处理
    FIFO:   FIFO流控结构体
    return_dat: 取出的数据
    */
#define FIFO_buffer_fast_output(FIFO, return_dat) \
    {                                             \
        unsigned char *Out = FIFO.O_ptr;          \
        {                                         \
            if (Out == FIFO.buffer_END)           \
                FIFO.O_ptr = FIFO.buffer;         \
            else                                  \
                FIFO.O_ptr = Out + 1;             \
        }                                         \
        return_dat = *Out;                        \
    }

    /*从FIFO流中取出多个字节
    buf :       FIFO流控结构体的地址
    buf_out :   放入输出数据的数组地址
    max_datas : 最大输出数量限制
    */
    unsigned int FIFO_buffer_output_many(FIFO_buffer *buf, unsigned char *buf_out, unsigned int max_datas);

    /*统计FIFO中已存放的数据长度
    return: 数据长度
    */
    unsigned int FIFO_buffer_length(FIFO_buffer *buf);

    /*宏定义:查看FIFO中是否有数据,必须以逻辑值使用
    FIFO:   FIFO流控结构体
    return: 逻辑0:无数据，逻辑1:有数据
    */
#define if_FIFO_buffer_have_data(FIFO) (FIFO.I_ptr != FIFO.O_ptr)

#ifdef __cplusplus
}
#endif

#endif // !__FIFO_BUFFER_H
