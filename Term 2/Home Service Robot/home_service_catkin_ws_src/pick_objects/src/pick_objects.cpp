//
// Created by nvidia on 5/30/18.
//
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int target_goal_one() {

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    move_base_msgs::MoveBaseGoal goal;

    // set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = -0.01;
    goal.target_pose.pose.position.y = 5.25;
    goal.target_pose.pose.orientation.w = -0.01;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal number one");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base made it to the first goal");
        //Sleep 5 seconds
        ROS_INFO("Sleep 5 seconds and then continue");
        ros::Duration(5).sleep();
        return 1;
    }
    else {
        ROS_INFO("The base failed to move forward for some reason");
        return 0;
    }
}

void target_goal_two() {

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    move_base_msgs::MoveBaseGoal goal;
    // set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = -5.50;
    goal.target_pose.pose.position.y = -0.70;
    goal.target_pose.pose.orientation.w = -0.01;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal goal number two");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base made it to the second goal");
    }
    else {
        ROS_INFO("The base failed to move forward for some reason");
    }
    return;
}

int main(int argc, char** argv){
    // Initialize the simple_navigation_goals node
    ros::init(argc, argv, "pick_objects");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }


    int tg_1 = target_goal_one();

    if (tg_1 == 1) {
        target_goal_two();
    } else {
        ROS_INFO("ABSOLUTE FAILURE");
    }

    return 0;
}