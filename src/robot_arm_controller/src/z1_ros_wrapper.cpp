#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "unitree_arm_sdk/control/unitreeArm.h"

UNITREE_ARM::unitreeArm* armPtr = nullptr; // Pointer to the arm object
double gripper = 0.0;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // Example usage: setting some arm commands based on the incoming Twist message
    // Here we simply print the received velocity commands
    /* ROS_INFO("Received cmd_vel: linear x: [%f], y: [%f], z: [%f]; angular x: [%f], y: [%f], z: [%f]",
        msg->linear.x, msg->linear.y, msg->linear.z,
        msg->angular.x, msg->angular.y, msg->angular.z); */

    if (armPtr)
    {
        // Mapping the velocity command to Cartesian control directions
        Vec7 directions;
        directions << msg->angular.x, msg->angular.y, msg->angular.z, 
                      msg->linear.x, msg->linear.y, msg->linear.z, gripper; // Assuming 0.0 for the 7th component
                      
        ROS_INFO("Received cmd_vel: linear x: [%f], y: [%f], z: [%f]; angular x: [%f], y: [%f], z: [%f], Gripper:[%f]",
        msg->linear.x, msg->linear.y, msg->linear.z,
        msg->angular.x, msg->angular.y, msg->angular.z, gripper);

        double angular_vel = 0.6; // Example orientation speed
        double linear_vel = 0.6; // Example position speed
        //armPtr->startTrack(UNITREE_ARM::ArmFSMState::CARTESIAN);
        armPtr->cartesianCtrlCmd(directions, angular_vel, linear_vel);

    }
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{

        if (msg->buttons[0] == 1 && msg->buttons[1] == 1) // Both buttons pressed to close gripper
        {
            gripper = 1.0;
        }
        else if ((msg->buttons[0] == 1 && msg->buttons[1] == 0) ||  (msg->buttons[0] == 0 && msg->buttons[1] == 1)) // 1 button press to open gripper
        {
            gripper = -1.0;
        }
        else
        {
            gripper = 0.0;
        }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "unitree_arm_controller");
    ros::NodeHandle nh;

    ROS_INFO("Starting unitree_arm_controller node");

    // Initialize the arm
    UNITREE_ARM::unitreeArm arm(true);
    armPtr = &arm; // Assign the pointer to the arm object
    arm.sendRecvThread->start();
    // arm.backToStart();
    arm.startTrack(UNITREE_ARM::ArmFSMState::CARTESIAN);

    // Subscribe to the /cmd_vel topic
    ros::Subscriber sub = nh.subscribe("/spacenav/twist", 10, cmdVelCallback);

    ROS_INFO("Subscribed to /spacenav/twist");

    ros::Subscriber subGripper = nh.subscribe("/spacenav/joy", 10, joyCallback);

    ROS_INFO("Subscribed to /spacenav/joy");

    // ROS spin to keep the callback function alive
    ros::spin();

    //Clean up
    //arm.backToStart();
    //arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();

    ROS_INFO("Shutting down unitree_arm_controller node");

    return 0;
}

