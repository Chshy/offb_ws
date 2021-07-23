/**********************************************

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher set_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber currentHeading = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 10, heading_cb);
    
**********************************************/

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <geometry_msgs/Pose2D.h>
#include <mavros_msgs/CommandTOL.h>
#include <time.h>
#include <cmath>
#include <math.h>
#include <ros/duration.h>

#include "frame_trans.hpp"

using namespace std;

//Set global variables
mavros_msgs::State fcu_state;
geometry_msgs::PoseStamped fcu_pose;
geometry_msgs::PoseStamped gym_pose;
std_msgs::Float64 fcu_heading;
std_msgs::Float64 gym_heading;

//获取状态(mavros/state 的回调函数)
//get armed state
//DISARMED-锁定 ARMED-解锁
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    fcu_state = *msg;
    // bool connected = fcu_state.connected;
    // bool armed = fcu_state.armed;
}

//获取当前飞机位置(/mavros/global_position/pose 的回调函数)
//get current position of drone
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    fcu_pose = *msg;
    gym_pose = frame_fcu2gym(fcu_pose);
    //ROS_INFO("x: %f y: %f z: %f", fcu_pose.pose.position.x, fcu_pose.pose.position.y, fcu_pose.pose.position.z);
}

//获取罗盘方向(/mavros/global_position/compass_hdg 的回调函数)
//get compass heading
void heading_cb(const std_msgs::Float64::ConstPtr &msg)
{
    fcu_heading = *msg;
    gym_heading = fcu_heading
    // ROS_INFO("current heading: %f", fcu_heading.data);
}
