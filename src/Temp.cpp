#include "ros/ros.h"
#include <pigpiod_if2.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Temp");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);
    
    int pi = pigpio_start(0, 0);
    unsigned handle = i2c_open(pi, 1, 0x68, 0);

    // レジスタをリセットする
    i2c_write_byte_data(pi, handle, 0x6B, 0x80);
    time_sleep(0.1);

    //PWR_MGMT_1をクリア
    i2c_write_byte_data(pi, handle, 0x6B, 0x00);
    time_sleep(0.1);

    char H[1];
    char L[1];

    //生データを取得する
    while(ros::ok())
    {
        i2c_read_i2c_block_data(pi, handle, 0x41, H, 1);
        i2c_read_i2c_block_data(pi, handle, 0x42, L, 1);
        float raw = H[0] << 8 | L[0];

        //温度の算出式はデータシートから下記の通り
        //((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 21degC
        float temp = (raw / 333.87) + 21.0;

        printf ("%.2f\n", temp);

        loop_rate.sleep();
    }
    return 0;
}
