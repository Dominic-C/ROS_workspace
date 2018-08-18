#include "ros/ros.h"
#include "ros_tutorials_topic/MsgTutorial.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topic_publisher"); // initilize node name
    ros::NodeHandle nh; // node handle declaration for communication with ROS system

    // initialise publisher
    // make node handler advertise a topic
    // message file from ros_tutorials_topic package
    // topic name is ros_tutorial_msg
    ros::Publisher ros_tutorial_pub = nh.advertise<ros_tutorials_topic::MsgTutorial>("ros_tutorial_msg", 100);

    ros::Rate loop_rate(10); // publish at 10 Hz

    // create message
    ros_tutorials_topic::MsgTutorial msg;

    int count = 0;

    while(ros::ok())
    {
        msg.stamp = ros::Time::now();
        msg.data = count;

        ROS_INFO("send msg = %d", msg.stamp.sec); // printing message contents
        ROS_INFO("send msg = %d", msg.stamp.nsec);
        ROS_INFO("send msg = %d", msg.data);

        // publish message
        ros_tutorial_pub.publish(msg);
        loop_rate.sleep();

        ++count;
    }

    return 0;
}
