#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>

double orientation[4];


void CallbackOrientation(const geometry_msgs::Quaternion::ConstPtr& q)
{
  orientation[0] = q->x;
  orientation[1] = q->y;
  orientation[2] = q->z;
  orientation[3] = q->w;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "converter");

  ros::NodeHandle nh;

  ros::Subscriber imu_ori = nh.subscribe("imu_orientation", 100,CallbackOrientation);

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 100);

  ros::Rate loop_rate(10);

  sensor_msgs::Imu imu;

  while(ros::ok())
  {
    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = "imu";

    imu.orientation.x = orientation[0];
    imu.orientation.y = orientation[1];
    imu.orientation.z = orientation[2];
    imu.orientation.w = orientation[3];

    

    imu_pub.publish(imu);

    ros::spinOnce();

    loop_rate.sleep();
  }
}

