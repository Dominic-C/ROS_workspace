#include "ros/ros.h"
// service header file
#include "ros_tutorials_service/SrvTutorial.h"


#define PLUS 1
#define MINUS 2
#define MULTIPLICATION 3
#define DIVISION 4

int g_operator;
// The process below is performed if there is a service request
// The service request is declared as 'req', and the service response is declared as 'res'
bool calculation(ros_tutorials_service::SrvTutorial::Request &req, ros_tutorials_service::SrvTutorial::Response &res)
    {
        // The operator will be selected according to the parameter value and calculate 'a' and 'b',
        // which were received upon the service request.
        // The result is stored as the Response value.
        switch(g_operator)
        {
            case PLUS:
                res.result = req.a + req.b;
                break;
            case MINUS:
                res.result = req.a - req.b;
                break;
            case MULTIPLICATION:
                res.result = req.a * req.b;
                break;
            case DIVISION:
                if(req.b == 0)
                {
                    res.result = 0;
                    break;
                }
                else
                {
                    res.result = req.a / req.b;
                    break;
                }
            default:
                res.result = req.a + req.b;
                break;
            }
        // Displays the values of 'a' and 'b' used in the service request, and the 'result' value
        // corresponding to the service response.
        ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
        ROS_INFO("sending back response: [%ld]", (long int)res.result);
        return true;
    }

// main node function
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "service_server"); // initialize node name
    ros::NodeHandle nh; // create Node handler
    nh.setParam("calculation_method", PLUS); // sets the parameter of "calculation method" to PLUS
    // perimeters can be set to integers, booleans, float, strings, dictionaries etc.

    // Declare service server as ros_tutorials_service_server
    // calls 'calculation upon service request'
    ros::ServiceServer ros_tutorials_service_server = nh.advertiseService("ros_tutorial_srv", calculation);
    ROS_INFO("ready srv server!");
    ros::Rate r(10);
    while (1)
    {
        nh.getParam("calculation_method", g_operator); // gets value from "calculation_method" parameter and sets g_operator to be that value
        ROS_INFO("changed calculation method to %d", g_operator);
        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
}