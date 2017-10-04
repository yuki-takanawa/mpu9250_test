#include "ros/ros.h"
#include <pigpiod_if2.h>
#include <sensor_msgs/Imu.h>
//#include <geometry_msgs/Vector3.h>

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
    ros::init(argc, argv, "Accel");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("data", 10);
    sensor_msgs::Imu msg;

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

    //low-passフィルタを設定
    i2c_write_byte_data(pi, handle, 0x1D, 0x03);

    //加速度センサのレンジを±8gにする
    i2c_write_byte_data(pi, handle, 0x1C, 0x02);

    //較正値を産出する
    printf("Accel calibration start\n");
    float sum[3] = {0, 0, 0};

    //実データのサンプルをとる
    char data[6];
    int flag = 0;

    for(int i = 0; i < 1000; i++)
    {
        i2c_read_i2c_block_data(pi, handle, 0x3B, data, 6);
        sum[0] += (2.0 / float(0x8000)) * u2s(data[0] << 8 | data[1]);
        sum[1] += (2.0 / float(0x8000)) * u2s(data[2] << 8 | data[3]);
        sum[2] += (2.0 / float(0x8000)) * u2s(data[4] << 8 | data[5]);
        //printf("%d\n", i);
    }

    //平均値をオフセットにする
    float offsetAccelX = -1.0 * sum[0] / 1000;
    float offsetAccelY = -1.0 * sum[1] / 1000;
    float offsetAccelZ = -1.0 * ((sum[2] / 1000) - 1.0);

    
    printf("Accel calibration complete\n");


    //重力である1Gの加速度（m/s^2）
    float ms2 = 9.80665;


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

        float ax = rawX_1 * ms2;
        float ay = rawY_1 * ms2;
        float az = rawZ_1 * ms2;

        msg.linear_acceleration.x = ax;
        msg.linear_acceleration.y = ay;
        msg.linear_acceleration.z = az;
        imu_pub.publish(msg);

        //生データ & 較正値＋LPF
        printf("%8.7f\t", rawX);
        printf("%8.7f\t", rawY);
        printf("%8.7f\t", rawZ);
        printf("%8.7f\t", rawX_1 * ms2);
        printf("%8.7f\t", rawY_1 * ms2);
        printf("%8.7f\n", rawZ_1 * ms2);

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
