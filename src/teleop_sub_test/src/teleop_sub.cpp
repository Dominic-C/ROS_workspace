#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("linear x = %f", msg->linear.x);
    ROS_INFO("linear y = %f", msg->linear.y);
    ROS_INFO("linear z = %f", msg->linear.z);
    ROS_INFO("angular x = %f", msg->angular.x);
    ROS_INFO("angular y = %f", msg->angular.y);
    ROS_INFO("angular z = %f", msg->angular.z);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_sub");
    ros::NodeHandle nh;

    ros::Subscriber ros_teleop_sub = nh.subscribe("/cmd_vel_mux/input/teleop", 100, chatterCallback);

    ros::spin();
    return 0;
}