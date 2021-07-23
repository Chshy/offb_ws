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

geometry_msgs::PoseStamped destinatin_gym; //注意 此变量的orientation未使用
geometry_msgs::PoseStamped destinatin_fcu;
float yaw_fcu;
float yaw_gym;

//设置飞机的朝向
//set orientation of the drone (drone should always be level)
// void setYaw_fcu(float heading)
// {
//     float yaw = heading * (M_PI / 180);
//     float pitch = 0;
//     float roll = 0;
//     float cy = cos(yaw * 0.5);
//     float sy = sin(yaw * 0.5);
//     float cr = cos(roll * 0.5);
//     float sr = sin(roll * 0.5);
//     float cp = cos(pitch * 0.5);
//     float sp = sin(pitch * 0.5);
//     float qw = cy * cr * cp + sy * sr * sp;
//     float qx = cy * sr * cp - sy * cr * sp;
//     float qy = cy * cr * sp + sy * sr * cp;
//     float qz = sy * cr * cp - cy * sr * sp;
//     destinatin_fcu.pose.orientation.w = qw;
//     destinatin_fcu.pose.orientation.x = qx;
//     destinatin_fcu.pose.orientation.y = qy;
//     destinatin_fcu.pose.orientation.z = qz;
//     return;
// }

void setYaw_fcu_Quaternion(float fcu_heading)
{
    float yaw = fcu_heading * (M_PI / 180); //rad
    float pitch = 0;
    float roll = 0;
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float qw = cy * cr * cp + sy * sr * sp;
    float qx = cy * sr * cp - sy * cr * sp;
    float qy = cy * cr * sp + sy * sr * cp;
    float qz = sy * cr * cp - cy * sr * sp;
    destinatin_fcu.pose.orientation.w = qw;
    destinatin_fcu.pose.orientation.x = qx;
    destinatin_fcu.pose.orientation.y = qy;
    destinatin_fcu.pose.orientation.z = qz;
    return;
}

void setYaw_fcu(float heading)
{
    yaw_fcu = heading;
    yaw_gym = yaw_fcu2gym(heading);
    setYaw_fcu_Quaternion(yaw_fcu);
    return;
}

void setYaw_gym(float heading)
{
    yaw_gym = heading;
    yaw_fcu = yaw_gym2fcu(heading);
    setYaw_fcu_Quaternion(yaw_fcu);
    // setYaw_fcu(-heading + 90 - YAW_OFFSET);
    return;
}

void setDestination_fcu(float x, float y, float z)
{
    destinatin_fcu.pose.position.x = x;
    destinatin_fcu.pose.position.y = y;
    destinatin_fcu.pose.position.z = z;
    frame_fcu2gym(&x, &y, &z);
    destinatin_gym.pose.position.x = x;
    destinatin_gym.pose.position.y = y;
    destinatin_gym.pose.position.z = z;
    return;
}

// set position to fly to in the gym frame
void setDestination_gym(float x, float y, float z)
{
    destinatin_gym.pose.position.x = x;
    destinatin_gym.pose.position.y = y;
    destinatin_gym.pose.position.z = z;
    frame_gym2fcu(&x, &y, &z);
    destinatin_fcu.pose.position.x = x;
    destinatin_fcu.pose.position.y = y;
    destinatin_fcu.pose.position.z = z;
    return;
}
