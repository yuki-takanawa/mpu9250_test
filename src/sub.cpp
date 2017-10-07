#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

void messageCallBack(const sensor_msgs::Imu& msg)
{
    ROS_INFO_STREAM(msg.linear_acceleration.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"sub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("data", 10, &messageCallBack);
    ros::spin();
}
