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
    ros::init(argc, argv, "Gyro");
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

    //ジャイロセンサの計測レンジを1000dpsに修正する
    i2c_write_byte_data(pi, handle, 0x1B, 0x10);

    //low-passフィルタを設定
    i2c_write_byte_data(pi, handle, 0x1A, 0x02);

    //dpsを算出する係数
    float gyroCoefficient = 1000 / float(0x8000);

    //較正値を算出する
    printf("Gyro calibration start\n");
    float sum[3] = {0, 0, 0};

    //実データのサンプルを取る
    char data[6];

    for(int i = 0; i < 1000; i++)
    {
        i2c_read_i2c_block_data(pi, handle, 0x43, data, 6);
        sum[0] += gyroCoefficient * u2s(data[0] << 8 | data[1]);
        sum[1] += gyroCoefficient * u2s(data[2] << 8 | data[3]);
        sum[2] += gyroCoefficient * u2s(data[4] << 8 | data[5]);
        printf("%d\n", i);
    }

    //平均値をオフセットにする
    float offsetGyroX = -1.0 * sum[0] / 1000;
    float offsetGyroY = -1.0 * sum[1] / 1000;
    float offsetGyroZ = -1.0 * sum[2] / 1000;

    printf("Gyro calibration complete\n");


    //データを取得する
    while(ros::ok())
    {
        i2c_read_i2c_block_data(pi, handle, 0x43, data, 6);
        float rawX = gyroCoefficient * u2s(data[0] << 8 | data[1]);
        float rawY = gyroCoefficient * u2s(data[2] << 8 | data[3]);
        float rawZ = gyroCoefficient * u2s(data[4] << 8 | data[5]);

        float rawX_1 = gyroCoefficient * u2s(data[0] << 8 | data[1]) + offsetGyroX;
        float rawY_1 = gyroCoefficient * u2s(data[2] << 8 | data[3]) + offsetGyroY;
        float rawZ_1 = gyroCoefficient * u2s(data[4] << 8 | data[5]) + offsetGyroZ;

        printf("%8.7f\t", rawX);
        printf("%8.7f\t", rawY);
        printf("%8.7f\t", rawZ);
        printf("%8.7f\t", rawX_1);
        printf("%8.7f\t", rawY_1);
        printf("%8.7f\n", rawZ_1);

        loop_rate.sleep();
    }
    return 0;
}
