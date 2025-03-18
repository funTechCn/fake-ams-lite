#include "FIFO_buffer.h"
#include "string.h"//使用memcpy()函数

FIFO_buffer FIFO_buf_by_normal_buf(unsigned char *buf_ADR, unsigned int _size)
{
    FIFO_buffer BUF;
    BUF.buffer = buf_ADR;
    BUF.buffer_END = buf_ADR + _size - 1;
    BUF.I_ptr = buf_ADR;
    BUF.O_ptr = buf_ADR;
    BUF.ERROR = 0;
    return BUF;
}
void FIFO_buffer_input_byte(FIFO_buffer *buf, const unsigned char dat)
{
    unsigned char *In = buf->I_ptr;

    *(In) = dat;
    if (In == buf->buffer_END)
        In = buf->buffer;
    else
        In++;
    if (In == buf->O_ptr)
        buf->ERROR = 1;
    else
        buf->I_ptr = In;
}
void FIFO_buffer_input_many(FIFO_buffer *buf, unsigned char *buf_in, unsigned int len)
{
    unsigned int O1 = buf->O_ptr - buf->buffer;
    unsigned int I1 = buf->I_ptr - buf->buffer;
    unsigned int I2 = buf->buffer_END - buf->I_ptr + 1;
    if (I1 >= O1)
    {
        if (len > I2)
        {
            memcpy(buf->I_ptr, buf_in, I2);
            memcpy(buf->buffer,buf_in+I2, len-I2);
            buf->I_ptr = buf->buffer + len-I2;
            buf->ERROR = 1;
        }
        else
        {
            memcpy(buf->I_ptr, buf_in, len);
            buf->I_ptr = buf->I_ptr + len;
        }
    }
    else
    {
        if (len > O1-I1)
        {
            memcpy(buf->I_ptr, buf_in, O1-I1);
            buf->I_ptr = buf->I_ptr + O1-I1;
            buf->ERROR = 1;
        }
        else
        {
            memcpy(buf->I_ptr, buf_in, len);
            buf->I_ptr = buf->I_ptr + len;
        }
    }
}
unsigned char FIFO_buffer_output_byte(FIFO_buffer *buf)
{
    unsigned char *Out = buf->O_ptr;
    if (buf->O_ptr == buf->I_ptr)
        buf->ERROR = 2;
    else
    {
        if (Out == buf->buffer_END)
            buf->O_ptr = buf->buffer;
        else
            buf->O_ptr = Out + 1;
    }
    return *Out;
}
unsigned int FIFO_buffer_output_many(FIFO_buffer *buf, unsigned char *buf_out,unsigned int max_datas)
{
    unsigned int count = 0;
    if (buf->O_ptr == buf->I_ptr)
        buf->ERROR = 2;
    else
    {
        unsigned int O1 = buf->O_ptr - buf->buffer;
        unsigned int O2 = buf->buffer_END - buf->O_ptr+1;
        unsigned int I1 = buf->I_ptr - buf->buffer;
        if (I1 >= O1)
        {
            count = I1 - O1;
            if(count<max_datas)
            {
                memcpy(buf_out, buf->O_ptr, count);
                buf->O_ptr += count;
                return count;
            }
            else
            {
                memcpy(buf_out, buf->O_ptr, max_datas);
                buf->O_ptr += max_datas;
                return max_datas;
            }
        }
        else
        {
            if(O2<max_datas)
            {
                memcpy(buf_out, buf->O_ptr, O2);
                max_datas -= O2;
                if(I1<max_datas)
                {
                    memcpy(buf_out + O2, buf->buffer, I1);
                    buf->O_ptr = buf->buffer + I1;
                    return O2+I1;
                }
                else
                {
                    memcpy(buf_out + O2, buf->buffer, max_datas);
                    buf->O_ptr = buf->buffer + max_datas;
                    return O2+max_datas;
                }
            }
            else
            {
                memcpy(buf_out, buf->O_ptr, max_datas);
                buf->O_ptr = buf->O_ptr + max_datas;
                return max_datas;
            }
        }
    }
    return 0;
}
unsigned int FIFO_buffer_length(FIFO_buffer *buf)
{
    unsigned int O1 = buf->O_ptr - buf->buffer;
    unsigned int O2 = buf->buffer_END - buf->O_ptr +1;
    unsigned int I1 = buf->I_ptr - buf->buffer;
    if (I1 >= O1)
    {
        return I1 - O1;
    }
    else
    {
        return I1 + O2;
    }
}