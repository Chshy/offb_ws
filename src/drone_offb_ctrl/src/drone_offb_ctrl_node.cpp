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

#include "drone_offb_ctrl/frame_trans.hpp"
#include "drone_offb_ctrl/basic_info_callback.hpp"
#include "drone_offb_ctrl/destination_set.hpp"


using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node"); //初始化节点名称和其他信息
    ros::NodeHandle nh;                 //创建节点的句柄 它可以用来创建Publisher、Subscriber以及做其他事情

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    ros::Publisher set_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);       //mavros/setpoint_raw/local Topic 的 Publisher
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10); //mavros/setpoint_raw/local Topic 的 Publisher

    ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);//global?
    ros::Subscriber currentHeading = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 10, heading_cb);

    // allow the subscribers to initialize
    ROS_INFO("INITILIZING...");

    for (int i = 0; i < 100; i++)
    {
        ros::spinOnce();             //处理消息订阅
        ros::Duration(0.01).sleep(); //Sleep 0.01s
    }

    //当不为GUIDED模式时一直等待
    while (fcu_state.guided != true)
    {
        ros::spinOnce();             //处理消息订阅
        ros::Duration(0.01).sleep(); //Sleep 0.01s
    }

    ROS_INFO("GUIDED = True");


    /*
    //MAV:33
    //set the orientation of the gym 
    //检测飞行场地相对于ENU的Yaw角度
    GYM_OFFSET = 0;
    for (int i = 1; i <= 30; ++i)
    {
        ros::spinOnce();            //处理消息订阅
        ros::Duration(0.1).sleep(); //Sleep 0.1s
        GYM_OFFSET += current_heading.data;
        ROS_INFO("current heading%d: %f", i, GYM_OFFSET / i);
    }
    GYM_OFFSET /= 30;
    ROS_INFO("the N' axis is facing: %f", GYM_OFFSET);
    cout << GYM_OFFSET << endl;
    */
   
    //等待飞控连接
    // wait for FCU connection
    while (ros::ok() && !fcu_state.connected)
    {
        ros::spinOnce(); //处理消息订阅
        rate.sleep();    //睡眠，保证循环频率为20Hz
    }


//while(1)
//{
    //飞控解锁(Request) CMD:400
    // arming
/*
    ros::ServiceClient arming_client_i = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mavros_msgs::CommandBool srv_arm_i;
    srv_arm_i.request.value = true;

    if (arming_client_i.call(srv_arm_i) && srv_arm_i.response.success)
        ROS_INFO("ARM sent %d", srv_arm_i.response.success);
    else
    {
        ROS_ERROR("Failed arming");
        return -1;
    }
*/
//}

    //起飞请求(Request)  CMD:22
    //request takeoff
    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 1.5;

    if (takeoff_cl.call(srv_takeoff))
    {
        ROS_INFO("takeoff sent %d", srv_takeoff.response.success);

        offset_calib( fcu_heading , fcu_pose );

        // POSE_OFFSET = current_pose;
        // ROS_INFO("POSE_OFFSET E=%lf N=%lf U=%lf", POSE_OFFSET.pose.position.x,POSE_OFFSET.pose.position.y,POSE_OFFSET.pose.position.z);
    }
    else
    {
        ROS_ERROR("Failed Takeoff");
        return -1;
    }
    sleep(10);

    //前进(Publish)
    //move foreward
    setYaw_gym(0);
    // setDestination(0, 2, 1.5);
    setDestination_gym(0, 0, 0.5);

    float tollorance = .15;//35
    if (local_pos_pub)
    {
        for (int i = 10000; ros::ok() && i > 0; --i)
        {
            local_pos_pub.publish(destinatin_fcu);


            float deltaX = abs(destinatin_gym.pose.position.x - gym_pose.pose.position.x);
            float deltaY = abs(destinatin_gym.pose.position.y - gym_pose.pose.position.y);
            float deltaZ = abs(destinatin_gym.pose.position.z - gym_pose.pose.position.z);
	        ROS_INFO("Current Pos: x=%5.3lf y=%5.3lf z=%5.3lf", gym_pose.pose.position.x,gym_pose.pose.position.y,gym_pose.pose.position.z);

            float dMag = sqrt(pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2));

            if (dMag < tollorance)
            {
                break;
            }

            ros::spinOnce();            //处理消息订阅
            ros::Duration(0.5).sleep(); //Sleep 0.5s

            if (i == 1)
            {
                ROS_INFO("Failed to reach destination. Stepping to next task.");
            }
        }
        ROS_INFO("Done moving foreward.");
    }

    //降落(Request)
    //land
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;

    if (land_client.call(srv_land) && srv_land.response.success)
        ROS_INFO("land sent %d", srv_land.response.success);
    else
    {
        ROS_ERROR("Landing failed");
        ros::shutdown(); //相当于Terminal里的Ctrl+C
        return -1;
    }

    while (ros::ok())
    {
        ros::spinOnce(); //处理消息订阅
        rate.sleep();
    }

    return 0;
}
