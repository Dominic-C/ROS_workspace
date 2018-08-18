#include <ros/ros.h>					// ROS Default Header File
#include <actionlib/client/simple_action_client.h> 	 // action Library Header File
#include <actionlib/client/terminal_state.h> 	 // Action Goal Status Header File
#include <ros_tutorials_action/FibonacciAction.h>	 // FibonacciAction Action File Header

int main(int argc, char** argv) // Node Main function
{
    ros::init(argc, argv, "action_client"); // Node Name Initialization

    // Action Client Declaration (Action Name: ros_tutorial_action)
    actionlib::SimpleActionClient<ros_tutorials_action::FibonacciAction> ac("ros_tutorial_action", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); // wait for action server to start
    
    ROS_INFO("Action server started, sending goal.");
    ros_tutorials_action::FibonacciGoal goal;
    goal.order = 20; // Set action goal (process the fibonacci sequence 20 times)
    ac.sendGoal(goal);

    // set action time limit (set to 30 seconds)
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out."); // If time out occurs
        // exit
        return 0;

    }