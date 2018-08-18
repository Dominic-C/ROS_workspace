#include <ros/ros.h> // ROS Default header file
#include <actionlib/server/simple_action_server.h> // action Library Header File
#include <ros_tutorials_action/FibonacciAction.h> // fibonacci action header file

class FibonacciAction // making a class called fibonacci action
{
    protected:
        // Node handle declaration
        ros::NodeHandle nh_;
        // Action Server declaration
        actionlib::SimpleActionServer<ros_tutorials_action::FibonacciAction> as_; // create action object using SimpleActionServer function, choosing FibonacciAction action

        std::string action_name_;
    
        // declare the action feedback and the result to publish.
        ros_tutorials_action::FibonacciFeedback feedback_;
        ros_tutorials_action::FibonacciResult result_;

    public:
        // constructor
        // Initialize action server (Node handle, action name, action callback function)
        FibonacciAction(std::string name) : as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false), action_name_(name)
        {
            as_.start();
        }

        // deconstructor
        ~FibonacciAction(void)
        {

        }

    void executeCB(const ros_tutorials_action::FibonacciGoalConstPtr &goal)
    {
        ros::Rate r(1); // Loop rate 1 Hz
        bool success = true;

        // Setting Fibonacci sequence initialization
        feedback_.sequence.clear();
        feedback_.sequence.push_back(0);
        feedback_.sequence.push_back(1);

        ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);
            
        for( int i = 1;  i < goal->order; i++)
        {
            // confirm action cancellation from action client
            if(as_.isPreemptRequested() || !ros::ok())
            {
                // Notify action cancellation
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted(); // action cancelled
                success = false; // consider action as failure and save to variable
                break;
            }

            feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
            as_.publishFeedback(feedback_); // publish feedback
            r.sleep();
        }

        if(success)
        {
            result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
        }


    }
};

int main(int argc, char** argv) // Node Main Function
{
    ros::init(argc, argv, "action_server"); // Initializes node name
    FibonacciAction fibonacci("ros_tutorial_action");
    ros::spin(); // wait to receive action goal
    return 0; // wait to receieve action goal
}
