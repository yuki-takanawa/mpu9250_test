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
    ros::init(argc, argv, "MPU9250_test_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);

    int pi = pigpio_start(0, 0);
    unsigned handle = i2c_open(pi, 1, 0x68, 0);   //i2c_open(pi, unsigned i2c_bus, unsigned i2c_addr, unsigned i2c_flags)

    // レジスタをリセットする
    i2c_write_byte_data(pi, handle, 0x6B, 0x00);  //i2c_write_byte_data(pi, handle, unsigned i2c_reg(書き込むレジスタのアドレス), unsigned bVal(書き込むデータ))
    time_sleep(0.1);

    //PWR_MGMT_1をクリア
    i2c_write_byte_data(pi, handle, 0x37, 0x00);
    time_sleep(0.1);

    //生データを取得する
    while(ros::ok())
    {
        char data[6];
        i2c_read_i2c_block_data(pi, handle, 0x3B, data, 6); //i2c_read_i2c_block_data(int pi, unsigned handle, unsigned i2c_reg, char *buf, unsigned count(欲しいバイト数))
        float rawX = (2.0 / float(0x8000)) * u2s(data[0] << 8 | data[1]);  //上位ビットが先
        float rawY = (2.0 / float(0x8000)) * u2s(data[2] << 8 | data[3]);  //上位ビットが先
        float rawZ = (2.0 / float(0x8000)) * u2s(data[4] << 8 | data[5]);  //上位ビットが先

        printf("%8.7f", rawX);
        printf(" ");
        printf("%8.7f", rawY);
        printf(" ");
        printf("%8.7f\n", rawZ);

        time_sleep(1);
    }
    return 0;
}
