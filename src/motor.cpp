#include "ros/ros.h"
#include <pigpiod_if2.h>

char *ser_tty= "/dev/ttyAMA0";
int pi = pigpio_start(0, 0);
unsigned handle = serial_open(pi, "/dev/tty1", 9600, 0);

/* プロトタイプ宣言 */
unsigned int Serial_GetBufferSpace(void);
void Serial_Update(unsigned char);
void Serial_putc(unsigned char, unsigned char);
int Serial_kbhit(void);
unsigned char Serial_getc(void);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);

    while(ros::ok())
    {
        Serial_putc(0x01, 0x10);
        printf("0x01, 0x10\n");
    }
    serial_close(pi, handle);

}




#define SERIAL_BUFFERSIZE       5
#define SERIAL_FRAMEDELIMITER   0x7e
#define SERIAL_ESCAPECHAR       0x7d
#define SERIAL_ID 0x00

unsigned char serial_buf[SERIAL_BUFFERSIZE];    /* 受信バッファ */
signed int serial_buf_idx = -1;                 /* 受信バッファインデックス */

/* バッファに1byteを入力 */
void Serial_SetBuffer(unsigned char data)
{
    if(SERIAL_BUFFERSIZE - 1 > serial_buf_idx)
    {
        serial_buf_idx++;
        serial_buf[serial_buf_idx] = data;
    }
}

/* バッファから1byteを出力 */
unsigned char Serial_GetBuffer(void)
{
    int i;
    unsigned char ret, tmp;
    if(serial_buf_idx > -1)
    {
        ret = serial_buf[0];
        for(i=0; i<serial_buf_idx; i++)
        {
            tmp = serial_buf[i];
            serial_buf[i] = serial_buf[i + 1];
            serial_buf[i+1] = tmp;
        }
        serial_buf_idx--;
        return ret;
    }
    return 0x00;
}

/* バッファの空き容量を調べる */
unsigned int Serial_GetBufferSpace(void)
{
    if (serial_buf_idx == -1)
        return SERIAL_BUFFERSIZE;
    else
        return SERIAL_BUFFERSIZE - serial_buf_idx - 1;
}

/* 受信データを処理する(受信割り込みを有効にします) */
#define SERIAL_STATE_WAIT   0                       /* 待機状態を定義 */
#define SERIAL_STATE_DATA   1                       /* データ待ち状態を定義 */
#define SERIAL_STATE_ESC    2                       /* エスケープシーケンス待ち状態を定義 */

unsigned char serial_state = SERIAL_STATE_WAIT;     /* 状態 */
unsigned char serial_fbuf[2];                       /* 受信フレームバッファ */
unsigned int serial_fbuf_idx = 0;                   /* 受信フレームバッファインデックス */

void Serial_Update(unsigned char data)
{
    /* 1フレーム分を受信する */
    switch(data)
    {
        case SERIAL_FRAMEDELIMITER:                 /* フレームデリミタの場合 */
          serial_state = SERIAL_STATE_DATA;
          serial_fbuf_idx = 0;
          break;
        case SERIAL_ESCAPECHAR:                     /* エスケープ文字の場合 */
          serial_state = SERIAL_STATE_ESC;
          break;
        default:                                    /* その他の場合 */
          switch(serial_state)
          {
              case SERIAL_STATE_DATA:                 /* データ待ち状態時 */
                serial_fbuf[serial_fbuf_idx] = data;
                serial_fbuf_idx++;
                break;
              case SERIAL_STATE_ESC:                  /* エスケープシーケンス待ち状態時 */
                serial_fbuf[serial_fbuf_idx] = 0xff - data;
                serial_fbuf_idx++;
                break;
          }
          break;
    }
    
    /* 1フレーム分の受信が完了した場合 */
    if (serial_fbuf_idx == 2)
    {
        if (SERIAL_ID == serial_fbuf[0])        /* 自デバイス宛なら受信バッファに入力 */
            Serial_SetBuffer(serial_fbuf[1]);
        else                                    /* 他デバイス宛なら次デバイスに送信 */
            Serial_putc(serial_fbuf[0], serial_fbuf[1]);
        serial_state = SERIAL_STATE_WAIT;
        serial_fbuf_idx = 0;
    }
    return;
}

/* 指定したID(id)を持つデバイスに1byteのデータ(data)を送信(受信割り込みを有効にします) */
void Serial_putc(unsigned char id, unsigned char data)
{
    serial_write_byte(pi, handle, SERIAL_FRAMEDELIMITER);                /* フレームデリミタ */
    serial_write_byte(pi, handle, id);                                   /* 送信先ID */
    switch(data)
    {
        case SERIAL_FRAMEDELIMITER:                 /* 送信データがフレームデリミタと重複してた場合 */
          serial_write_byte(pi, handle, SERIAL_ESCAPECHAR);
          serial_write_byte(pi, handle, 0xff - SERIAL_FRAMEDELIMITER);
          break;
        case SERIAL_ESCAPECHAR:                     /* 送信データがエスケープキャラクタと重複してた場合 */
          serial_write_byte(pi, handle, SERIAL_ESCAPECHAR);
          serial_write_byte(pi, handle, 0xff - SERIAL_ESCAPECHAR);
        break;
        default:                                    /* 通常のデータ送信 */
          serial_write_byte(pi, handle, data);
        break;
    }
    return;
}

/* 受信バッファにデータが入っているか調べる */
int Serial_kbhit(void)
{
    if(-1 == serial_buf_idx)
        return 0;
    else;
    return 1;
}

/* 受信バッファから1byte取得 */
unsigned char Serial_getc(void)
{
    while(!Serial_kbhit());
    return Serial_GetBuffer();
}
