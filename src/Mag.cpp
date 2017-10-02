#include "ros/ros.h"
#include <pigpiod_if2.h>

int u2s(unsigned unsigneddata)
{
    if(unsigneddata & (0x01 << 15))
    {
        unsigneddata = -1 * ((unsigneddata ^ 0xffff) + 1);
    }
    return unsigneddata;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mag");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);

    int pi = pigpio_start(0, 0);
    //i2c_open(pi, unsigned i2c_bus, unsigned i2c_addr, unsigned i2c_flags)
    unsigned handle = i2c_open(pi, 1, 0x68, 0);
    unsigned mag_handle = i2c_open(pi, 1, 0x0C, 0);

    // レジスタをリセットする
    i2c_write_byte_data(pi, handle, 0x6B, 0x80);
    i2c_write_byte_data(pi, mag_handle, 0x0B, 0x01);
    time_sleep(0.1);

    //I2Cで磁気センサ機能（AK8963）へアクセスできるようにする（BYPAS_EN=1）
    i2c_write_byte_data(pi, handle, 0x6B, 0x00);
    i2c_write_byte_data(pi, handle, 0x37, 0x02);
    time_sleep(0.1);

    //磁気センサのレジスタを設定する
    i2c_write_byte_data(pi, mag_handle, 0x0A, 0x06 | 0x10);
 
    while(ros::ok())
    {
        //磁気センサの値を取得する
        char status[1];
        i2c_read_i2c_block_data(pi, mag_handle, 0x02, status, 1);
        if((status[0] & 0x02) == 0x02)
        {
            //データオーバーランがあるので再度センシング
            i2c_read_byte_data(pi, mag_handle, 0x09);
        }

        //ST1レジスタを確認してデータ読み出しが可能か確認する
        i2c_read_i2c_block_data(pi, mag_handle, 0x02, status, 1);
        while((status[0] & 0x01) != 0x01)
        {
            time_sleep(0.01);
            i2c_read_i2c_block_data(pi, mag_handle, 0x02, status, 1);
        }

        //データの読み出し
        char data[7];
        i2c_read_i2c_block_data(pi, mag_handle, 0x03, data, 7);
        float rawX = u2s(data[1] << 8 | data[0]); //下位bitが先
        float rawY = u2s(data[3] << 8 | data[2]); //下位bitが先
        float rawZ = u2s(data[5] << 8 | data[4]); //下位bitが先
        char st2  = data[6];

        //オーバーフローチェック
        if((st2 & 0x08) == 0x08)
        {
            printf("004 Mag sensor over flow\n");
        }

        //μTへの変換
        float magCoefficient16 = 4912  / 32760.0; //センシングされたDecimal値をμTに変換する係数(16bit時)
        rawX = rawX * magCoefficient16;
        rawY = rawY * magCoefficient16;
        rawZ = rawZ * magCoefficient16;

        printf("%8.7f\t", rawX);
        printf("%8.7f\t", rawY);
        printf("%8.7f\n", rawZ);

        loop_rate.sleep();
    }
}
