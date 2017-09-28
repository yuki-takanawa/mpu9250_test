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
    //i2c_open(pi, unsigned i2c_bus, unsigned i2c_addr, unsigned i2c_flags)
    unsigned handle = i2c_open(pi, 1, 0x68, 0);

    // レジスタをリセットする
    //i2c_write_byte_data(pi, handle, unsigned i2c_reg(書き込むレジスタのアドレス), unsigned bVal(書き込むデータ))
    i2c_write_byte_data(pi, handle, 0x6B, 0x80);
    time_sleep(0.1);

    //PWR_MGMT_1をクリア
    i2c_write_byte_data(pi, handle, 0x6B, 0x00);
    time_sleep(0.1);

    //加速度センサのレンジを±8gにする
    i2c_write_byte_data(pi, handle, 0x1C, 0x02);

    //生データを取得する
/*    while(ros::ok())
    {
        char data[6];
        //i2c_read_i2c_block_data(int pi, unsigned handle, unsigned i2c_reg, char *buf, unsigned count(欲しいバイト数))
        i2c_read_i2c_block_data(pi, handle, 0x3B, data, 6);
        float rawX = (2.0 / float(0x8000)) * u2s(data[0] << 8 | data[1]); //上位ビットが先
        float rawY = (2.0 / float(0x8000)) * u2s(data[2] << 8 | data[3]); //上位ビットが先
        float rawZ = (2.0 / float(0x8000)) * u2s(data[4] << 8 | data[5]); //上位ビットが先

        static float X[2] = {0, 0};
        static float Y[2] = {0, 0};
        static float Z[2] = {0, 0};

        X[1] = 0.8 * X[0] + 0.2 * rawX;
        Y[1] = 0.8 * Y[0] + 0.2 * rawY;
        Z[1] = 0.8 * Z[0] + 0.2 * rawZ;

        printf("%8.7f\t", rawX);
        printf("%8.7f\t", rawY);
        printf("%8.7f\t", rawZ);
        printf("%8.7f\t", X[1]);
        printf("%8.7f\t", Y[1]);
        printf("%8.7f\n", Z[1]);

        X[0] = X[1];
        Y[0] = Y[1];
        Z[0] = Z[1];
*/
        //較正値を産出する
        printf("Accel calibration start\n");
        float sum[3] = {0, 0, 0};
        float sum_1[3] = {0, 0, 0};

        //実データのサンプルをとる
        char data[6];
        int flag = 0;

        for(int i = 0; i < 1000; i++)
        {
            i2c_read_i2c_block_data(pi, handle, 0x3B, data, 6);
            sum[0] += (2.0 / float(0x8000)) * u2s(data[0] << 8 | data[1]);
            sum[1] += (2.0 / float(0x8000)) * u2s(data[2] << 8 | data[3]);
            sum[2] += (2.0 / float(0x8000)) * u2s(data[4] << 8 | data[5]);
            printf("%d\n", i);
        }

        float offsetAccelX = -1.0 * sum[0] / 1000;
        float offsetAccelY = -1.0 * sum[1] / 1000;
        float offsetAccelZ = -1.0 * sum[2] / 1000;
        
        printf("OK\n");

        time_sleep(1);
        

        for(int i = 0; i < 10000; i++)
        {
            i2c_read_i2c_block_data(pi, handle, 0x3B, data, 6);
            sum_1[0] += (2.0 / float(0x8000)) * u2s(data[0] << 8 | data[1]);
            sum_1[1] += (2.0 / float(0x8000)) * u2s(data[2] << 8 | data[3]);
            sum_1[2] += (2.0 / float(0x8000)) * u2s(data[4] << 8 | data[5]);
            printf("%d\n", i);
        }

        //平均値をオフセットにする
        float offsetAccelX_1 = -1.0 * sum_1[0] / 10000;
        float offsetAccelY_1 = -1.0 * sum_1[1] / 10000;
        float offsetAccelZ_1 = -1.0 * sum_1[2] / 10000;

        printf("Accel calibration complete\n");

        //生データを取得する
        while(ros::ok())
        {
            i2c_read_i2c_block_data(pi, handle, 0x3B, data, 6);
            float rawX = (2.0 / float(0x8000)) * u2s(data[0] << 8 | data[1]);
            float rawY = (2.0 / float(0x8000)) * u2s(data[2] << 8 | data[3]);
            float rawZ = (2.0 / float(0x8000)) * u2s(data[4] << 8 | data[5]);

            float rawX_1 = (2.0 / float(0x8000)) * u2s(data[0] << 8 | data[1]) + offsetAccelX;
            float rawY_1 = (2.0 / float(0x8000)) * u2s(data[2] << 8 | data[3]) + offsetAccelY;
            float rawZ_1 = (2.0 / float(0x8000)) * u2s(data[4] << 8 | data[5]) + offsetAccelZ;

            float rawX_2 = (2.0 / float(0x8000)) * u2s(data[0] << 8 | data[1]) + offsetAccelX_1;
            float rawY_2 = (2.0 / float(0x8000)) * u2s(data[2] << 8 | data[3]) + offsetAccelY_1;
            float rawZ_2 = (2.0 / float(0x8000)) * u2s(data[4] << 8 | data[5]) + offsetAccelZ_1;

            printf("%8.7f\t", rawX);
            printf("%8.7f\t", rawY);
            printf("%8.7f\t", rawZ);
            printf("%8.7f\t", rawX_1);
            printf("%8.7f\t", rawY_1);
            printf("%8.7f\t", rawZ_1);
            printf("%8.7f\t", rawX_2);
            printf("%8.7f\t", rawY_2);
            printf("%8.7f\n", rawZ_2);




        time_sleep(1);
    }
    return 0;
}
