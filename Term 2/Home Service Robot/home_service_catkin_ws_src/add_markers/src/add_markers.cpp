//
// Created by nvidia on 5/30/18.
//
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sstream>
#include <cmath>

float pos1_x = -0.01;
float pos1_y = 5.25;

float pos2_x = -5.50;
float pos2_y = -0.70;

float pos_threshold = 0.15;

visualization_msgs::Marker marker;

//singular method to setup and show the marker at the desired location.
void setup_marker(visualization_msgs::Marker &marker, float pos_x, float pos_y)
{

    uint32_t shape = visualization_msgs::Marker::CYLINDER;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pos_x;
    marker.pose.position.y = pos_y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    return;
}

//This is the manual mode of adding and removing the marker per part 5 of the project
void add_marker_mode()
{
    ROS_INFO("add_marker_mode()");

    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    //show the marker in the top right section of the map for 5 seconds
    int marker_pause_seconds = 0;
    while (ros::ok() && marker_pause_seconds < 5)
    {

        setup_marker(marker, pos1_x, pos1_y);

        marker_pub.publish(marker);

        r.sleep();
        marker_pause_seconds += 1;
        ROS_INFO_STREAM(" show seconds: " << marker_pause_seconds);
    }

    //remove the marker for 5 seconds
    marker_pause_seconds = 0;
    while (ros::ok() && marker_pause_seconds < 5)
    {
        marker.action = visualization_msgs::Marker::DELETE;

        marker_pub.publish(marker);
        ROS_INFO("delete");
        r.sleep();
        marker_pause_seconds += 1;
    }

    //show the marker in the bottom left corner indefinitely
    while (ros::ok())
    {
        setup_marker(marker, pos2_x, pos2_y);

        marker_pub.publish(marker);

        r.sleep();

        ROS_INFO_STREAM(" show");
    }
}

//callback for handling odometery topic information.
void pose_amcl_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg_amcl)
{
    float posX = msg_amcl->pose.pose.position.x;
    float posY = msg_amcl->pose.pose.position.y;

    ROS_INFO_STREAM("amcl poseX: " << posX << " amcl poseY: " << posY);

    //determine if close to position 1 and then remove the marker
    float pos1_x_diff = posX - pos1_x;
    float pos1_y_diff = posY - pos1_y;
    if (std::abs (pos1_x_diff) < pos_threshold && std::abs(pos1_y_diff) < pos_threshold)
    {
        //remove the marker
        ROS_INFO("REMOVE THE MARKER");
        marker.action = visualization_msgs::Marker::DELETE;
    }


    //determine if close to position 2 and then show the marker
    float pos2_x_diff = posX - pos2_x;
    float pos2_y_diff = posY - pos2_y;
    if (std::abs (pos2_x_diff < pos_threshold && std::abs(pos2_y_diff < pos_threshold)))
    {
       //add the marker
       ROS_INFO("ADD THE MARKER");
       setup_marker(marker, pos2_x, pos2_y);
    }
}

void home_service_mode()
{
    ROS_INFO("home_service_mode");

    //add the marker to position 1 before subscribing to the topic
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::Subscriber sub_amcl = n.subscribe("amcl_pose", 100, pose_amcl_cb);

    ros::Duration time_between_ros_wakeups(0.001);

    setup_marker(marker, pos1_x, pos1_y);

    while (ros::ok()) {

        marker_pub.publish(marker);

        ros::spinOnce();
        time_between_ros_wakeups.sleep();
    }
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers");

    //pass in parameter from the command line to switch between the requirements of the project
    ros::NodeHandle nh_private("~");
    bool hsm_param;
    nh_private.getParam("home_service_mode", hsm_param);
    bool hsm = false;
    hsm = hsm_param;

    ROS_INFO_STREAM("home_service_mode: " << hsm_param);

    if (hsm == true) {
        home_service_mode();
    }
    else {
        add_marker_mode();
    }

    return 0;
}



