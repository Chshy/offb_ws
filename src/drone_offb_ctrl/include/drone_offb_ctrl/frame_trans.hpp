#pragma once
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
using namespace std;

#define CALIB_TIME 30

//起飞地点在飞控ENU下的坐标
geometry_msgs::PoseStamped POSE_OFFSET;

//起飞航向在飞控ENU下的Yaw
float YAW_OFFSET;
float YAW_OFFSET_COS, YAW_OFFSET_SIN;

//重置偏置值
void offset_reset()
{
    POSE_OFFSET.pose.position.x = 0;
    POSE_OFFSET.pose.position.y = 0;
    POSE_OFFSET.pose.position.z = 0;
    YAW_OFFSET = 0;
    YAW_OFFSET_COS = 1;
    YAW_OFFSET_SIN = 0;
    return;
}

//设定偏置值
void offset_calib(std_msgs::Float64 fcu_heading,
                  geometry_msgs::PoseStamped fcu_pose)
{

    for (int i = 1; i <= CALIB_TIME; ++i)
    {

        ros::spinOnce();            //处理消息订阅
        ros::Duration(0.1).sleep(); //Sleep 0.1s

        YAW_OFFSET += fcu_heading.data;
        POSE_OFFSET.pose.position.x += fcu_pose.pose.position.x;
        POSE_OFFSET.pose.position.y += fcu_pose.pose.position.y;
        POSE_OFFSET.pose.position.z += fcu_pose.pose.position.z;

        // ROS_INFO("current heading%d: %f", i, GYM_OFFSET / i);
    }
    YAW_OFFSET /= CALIB_TIME;
    POSE_OFFSET.pose.position.x /= CALIB_TIME;
    POSE_OFFSET.pose.position.y /= CALIB_TIME;
    POSE_OFFSET.pose.position.z /= CALIB_TIME;

    YAW_OFFSET_COS = cos(YAW_OFFSET);
    YAW_OFFSET_SIN = sin(YAW_OFFSET);

    // ROS_INFO("the N' axis is facing: %f", GYM_OFFSET);
    // cout << GYM_OFFSET << endl;

    return;
}

/////////////// 以下为坐标转换实现 ///////////////

float yaw_fcu2gym(float fcuyaw)
{
    return fcuyaw - YAW_OFFSET;
}

float yaw_gym2fcu(float gymyaw)
{
    return gymyaw + YAW_OFFSET;
}

//注意Yaw变换！！

//因为偏置值是在FCU ENU 坐标系下的
//所以坐标的加减要在方向统一的情况下做
//即在Yaw0是E方向的时候

void frame_fcu2gym(float *x, float *y, float *z)
{
    *z -= POSE_OFFSET.pose.position.z;

    float tx, ty;
    tx = *x - POSE_OFFSET.pose.position.x;
    ty = *y - POSE_OFFSET.pose.position.y;

    *x = tx * YAW_OFFSET_COS + ty * YAW_OFFSET_SIN;
    *y = ty * YAW_OFFSET_COS - tx * YAW_OFFSET_SIN;
    return;
}

void frame_gym2fcu(float *x, float *y, float *z)
{
    *z += POSE_OFFSET.pose.position.z;

    //sin(-x)=-sin(x)
    //cos(-x)=cos(x)
    float tx, ty;
    tx = *x * YAW_OFFSET_COS - *y * YAW_OFFSET_SIN;
    ty = *y * YAW_OFFSET_COS + *x * YAW_OFFSET_SIN;
    *x = tx + POSE_OFFSET.pose.position.x;
    *y = ty + POSE_OFFSET.pose.position.y;
    return;
}

void frame_fcu2gym(double *x, double *y, double *z)
{
    *z -= POSE_OFFSET.pose.position.z;

    double tx, ty;
    tx = *x - POSE_OFFSET.pose.position.x;
    ty = *y - POSE_OFFSET.pose.position.y;

    *x = tx * YAW_OFFSET_COS + ty * YAW_OFFSET_SIN;
    *y = ty * YAW_OFFSET_COS - tx * YAW_OFFSET_SIN;
    return;
}

void frame_gym2fcu(double *x, double *y, double *z)
{
    *z += POSE_OFFSET.pose.position.z;

    //sin(-x)=-sin(x)
    //cos(-x)=cos(x)
    double tx, ty;
    tx = *x * YAW_OFFSET_COS - *y * YAW_OFFSET_SIN;
    ty = *y * YAW_OFFSET_COS + *x * YAW_OFFSET_SIN;
    *x = tx + POSE_OFFSET.pose.position.x;
    *y = ty + POSE_OFFSET.pose.position.y;
    return;
}

geometry_msgs::PoseStamped frame_fcu2gym(geometry_msgs::PoseStamped pose)
{
    frame_fcu2gym(&pose.pose.position.x, &pose.pose.position.y, &pose.pose.position.z);
    return pose;
}

geometry_msgs::PoseStamped frame_gym2fcu(geometry_msgs::PoseStamped pose)
{
    frame_gym2fcu(&pose.pose.position.x, &pose.pose.position.y, &pose.pose.position.z);
    return pose;
}