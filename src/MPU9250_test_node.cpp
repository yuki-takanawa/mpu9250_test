#include "ros/ros.h"
#include <pigpiod_if2.h>

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
    i2c_write_byte_data(pi, handle, 0x37, 0x02);
    time_sleep(0.1);
}
