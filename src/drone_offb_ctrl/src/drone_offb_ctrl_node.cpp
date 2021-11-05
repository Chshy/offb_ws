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
#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <time.h>
#include <cmath>
#include <math.h>
#include <ros/duration.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

using namespace std;
using namespace mavros_msgs;

//Set global variables
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
/*pos_z and speed_xy info from t265 variables */
geometry_msgs::TwistWithCovarianceStamped VisionSpeed2;
/*pos_z and speed_xy info from t265 variables end */
geometry_msgs::PoseStamped pose;
std_msgs::Float64 current_heading;
float GYM_OFFSET;
tf2_ros::Buffer tfBuffer;
ros::Publisher set_gp_origin_pub;
ros::Publisher set_raw_pub;
ros::Publisher local_pos_pub;

//get state
//获取飞控状态(回调函数)
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
    bool connected = current_state.connected;
    bool armed = current_state.armed;
}

//get current position of drone
//获取飞控回传的位置
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
    //  ROS_INFO("x: %f y: %f z: %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    tf2::Quaternion q(
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_heading.data = yaw;
}

//set orientation of the drone (drone should always be level)

/*
Heading sets the yaw of base_link in snapshot_takeoff in rad
base_link and snapshot_takeoff are both FLU frames.
Thus,the x axis of snapshot_takeoff is 0 rad.
must call send_tf_snapshot_takeoff before takeoff to take a snapshot.
*/
void set_pose_local(float x, float y, float z, float heading) //x y z is in reference to the body (base_link) frame captured when taking off
{
    heading = heading + GYM_OFFSET; //Sent pose gets transformed from ENU to NED later in order to be sent to FCU.
    float yaw = heading;            //YAW: from X(E) to Y(N) in ENU frame with X(E) axis being 0 deg
    float pitch = 0;                //level
    float roll = 0;                 //level

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

    pose.pose.orientation.w = qw;
    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;

    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PointStamped initial_pt, transformed_pt;
    try
    {
        transformStamped = tfBuffer.lookupTransform("map", "snapshot_takeoff", ros::Time(0));

        initial_pt.point.x = x;
        initial_pt.point.y = y;
        initial_pt.point.z = z;
        tf2::doTransform(initial_pt, transformed_pt, transformStamped);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    float X = transformed_pt.point.x;
    float Y = transformed_pt.point.y;
    float Z = transformed_pt.point.z;
    pose.pose.position.x = X;
    pose.pose.position.y = Y;
    pose.pose.position.z = Z;
    local_pos_pub.publish(pose);
    ROS_INFO("Destination set to x: %f y: %f z %f", X, Y, Z);
}

void send_tf_snapshot_takeoff(void)
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseStamped pose_snapshot = current_pose;
    ;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "snapshot_takeoff";
    transformStamped.transform.translation.x = pose_snapshot.pose.position.x;
    transformStamped.transform.translation.y = pose_snapshot.pose.position.y;
    transformStamped.transform.translation.z = pose_snapshot.pose.position.z;
    transformStamped.transform.rotation = pose_snapshot.pose.orientation;
    GYM_OFFSET = current_heading.data;
    br.sendTransform(transformStamped);
}

int arm_drone(ros::NodeHandle &nh)
{
    // arming
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
    return 0;
}

int takeoff(ros::NodeHandle &nh, double height) //meters
{
    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = height;
    if (takeoff_cl.call(srv_takeoff))
    {
        ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
    }
    else
    {
        ROS_ERROR("Failed Takeoff");
        return -1;
    }
    return 0;
}

int land(ros::NodeHandle &nh)
{
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    if (land_client.call(srv_land) && srv_land.response.success)
        ROS_INFO("land sent %d", srv_land.response.success);
    else
    {
        ROS_ERROR("Landing failed");
        ros::shutdown();
        return -1;
    }
    return 0;
}

int set_speed_body(double x, double y, double z, double yaw_rate = 0) //flu meter/s rad/s, must set continously, or the vehicle stops after a few seconds(failsafe feature). yaw_rate = 0 when not used.
{
    mavros_msgs::PositionTarget raw_target;
    raw_target.coordinate_frame = PositionTarget::FRAME_BODY_OFFSET_NED;
    raw_target.type_mask = PositionTarget::IGNORE_PX | PositionTarget::IGNORE_PY | PositionTarget::IGNORE_PZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW;
    if (fabs(yaw_rate) < 1e-6)
        raw_target.type_mask |= PositionTarget::IGNORE_YAW_RATE;
    raw_target.velocity.x = x;
    raw_target.velocity.y = y;
    raw_target.velocity.z = z;
    raw_target.yaw_rate = yaw_rate;
    set_raw_pub.publish(raw_target);
    return 0;
}

int set_speed_enu(double x, double y, double z, double yaw_rate) //flu meter/s rad/s, must set continously, or the vehicle stops after a few seconds(failsafe feature). yaw_rate = 0 when not used.
{
    mavros_msgs::PositionTarget raw_target;
    raw_target.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
    raw_target.type_mask = PositionTarget::IGNORE_PX | PositionTarget::IGNORE_PY | PositionTarget::IGNORE_PZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW;
    if (fabs(yaw_rate) < 1e-6)
        raw_target.type_mask |= PositionTarget::IGNORE_YAW_RATE;
    raw_target.velocity.x = x;
    raw_target.velocity.y = y;
    raw_target.velocity.z = z;
    raw_target.yaw_rate = yaw_rate * 0.01745329;
    set_raw_pub.publish(raw_target);
    return 0;
}

int set_angular_rate(double yaw_rate) //FLU rad/s , must set continously, or the vehicle stops after a few seconds.(failsafe feature) used for adjusting yaw without setting others.
{
    mavros_msgs::PositionTarget raw_target;
    raw_target.coordinate_frame = PositionTarget::FRAME_BODY_OFFSET_NED;
    raw_target.type_mask = PositionTarget::IGNORE_VX | PositionTarget::IGNORE_VY | PositionTarget::IGNORE_VZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW; //yaw_rate must be used with pose or vel.
    raw_target.position.x = 0;
    raw_target.position.y = 0;
    raw_target.position.z = 0;
    raw_target.yaw_rate = yaw_rate;
    set_raw_pub.publish(raw_target);
    return 0;
}

int set_pose_body(double x, double y, double z, double yaw) //flu meters rad. yaw = 0 when not used. x=y=z=0 when not used.
{
    mavros_msgs::PositionTarget raw_target;
    raw_target.coordinate_frame = PositionTarget::FRAME_BODY_OFFSET_NED;
    raw_target.type_mask = PositionTarget::IGNORE_VX | PositionTarget::IGNORE_VY | PositionTarget::IGNORE_VZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW_RATE;
    if (fabs(yaw) < 1e-6)
        raw_target.type_mask |= PositionTarget::IGNORE_YAW;
    raw_target.position.x = x;
    raw_target.position.y = y;
    raw_target.position.z = z;
    raw_target.yaw = yaw;
    set_raw_pub.publish(raw_target);
    return 0;
}

int set_break()
{
    mavros_msgs::PositionTarget raw_target;
    raw_target.coordinate_frame = PositionTarget::FRAME_BODY_OFFSET_NED;
    raw_target.type_mask = PositionTarget::IGNORE_VX | PositionTarget::IGNORE_VY | PositionTarget::IGNORE_VZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW_RATE;
    raw_target.position.x = 0;
    raw_target.position.y = 0;
    raw_target.position.z = 0;
    raw_target.yaw = 0;
    set_raw_pub.publish(raw_target);
    return 0;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
    VisionSpeed2.twist.twist.linear.x = odom->twist.twist.linear.x;
    VisionSpeed2.twist.twist.linear.y = odom->twist.twist.linear.y;
    VisionSpeed2.twist.twist.linear.z = odom->twist.twist.linear.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_node");
    ros::NodeHandle nh;
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    set_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber odomSub = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 10, odomCallback);
    tf2_ros::TransformListener tfListener(tfBuffer);

    // allow the subscribers to initialize
    ROS_INFO("INITILIZING...");
    for (int i = 0; i < 100; i++)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    // while (current_state.mode != "OFFBOARD") //wait for remote command
    // {
    //     ros::spinOnce();
    //     ros::Duration(0.01).sleep();
    //     ROS_INFO("GUIDED_no");
    // }
    ROS_INFO("GUIDED_ok");

    //等两秒
    // for (int i = 0; i < 200; i++)
    // {
    //     ros::spinOnce();
    //     ros::Duration(0.01).sleep();
    // }

    // send_tf_snapshot_takeoff();

    // // recheck for FCU connection
    // while (ros::ok() && !current_state.connected)
    // {
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    ROS_INFO("connected_ok");
    //arm
    // arm_drone(nh);
    //等一秒

    // for (int i = 0; i < 100; i++)
    // {
    //     ros::spinOnce();
    //     ros::Duration(0.01).sleep();
    // }

    // //request takeoff
    // takeoff(nh, 1.5); //meters
    // //等10秒
    // for (int i = 0; i < 1000; i++)
    // {
    //     ros::spinOnce();
    //     ros::Duration(0.01).sleep();
    // }


        double x,y,z,w;
        while(1)
        {
            ros::spinOnce();
            tf2::Quaternion q(
                VisionSpeed2.pose.pose.orientation.x,
                VisionSpeed2.pose.pose.orientation.y,
                VisionSpeed2.pose.pose.orientation.z,
                VisionSpeed2.pose.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            ROS_INFO("Current_RPY:%lf,%lf,%lf\r\n",roll ,pitch, yaw);
        }



// #define TOTAL_STEP 4
//     const double dF[TOTAL_STEP] = {1.5, 0, -1.5, 0};
//     const double dL[TOTAL_STEP] = {0, 1.5, 0, -1.5};

//     for (int mission_step = 0; mission_step < 4; mission_step++)
//     {
//         double RestrainSpeed = 0.2;    // 限速 m/s
//         double curr_f, curr_l, curr_u; //当前位置
//         double dest_f, dest_l, dest_u; //目标位置
        

//         //获取当前T265位置(从/camera/odom/sample订阅,T265镜头朝向为x,插头的反方向为y,上方为z)
//         //需要变换到FLU坐标系
//         ros::spinOnce();
//         curr_f = -VisionSpeed2.twist.twist.linear.x;
//         curr_l = -VisionSpeed2.twist.twist.linear.y;
//         curr_u = VisionSpeed2.twist.twist.linear.z;
//         curr_

//         //从数组读取这一步相对于上一步移动的距离(m)并保存T

//         for (int i = 200; i > 0; i--)
//         {
//             ros::spinOnce();
//             set_speed_body(dF[mission_step], dF[mission_step], 0); //flu
//             ros::Duration(0.05).sleep();
//             ROS_INFO("CB:%lf %lf %lf\n", VisionSpeed2.twist.twist.linear.x, VisionSpeed2.twist.twist.linear.y, VisionSpeed2.twist.twist.linear.z);
//         }
//         //等2秒
//         for (int i = 0; i < 200; i++)
//         {
//             ros::spinOnce();
//             set_speed_body(0, 0, 0);
//             ros::Duration(0.01).sleep();
//         }
//     }


    // //  move foreward
    //   setHeading(45);//in reference to snapshot_takeoff,FLU,direction:x to y
    //   setDestination(2, 0, 1.5);//in reference to snapshot_takeoff,FLU
    //   float tollorance = .35;
    //   if (local_pos_pub)
    //   {

    //     for (int i = 10000; ros::ok() && i > 0; --i)
    //     {

    //       local_pos_pub.publish(pose);

    //       float deltaX = abs(pose.pose.position.x - current_pose.pose.position.x);
    //       float deltaY = abs(pose.pose.position.y - current_pose.pose.position.y);
    //       float deltaZ = abs(pose.pose.position.z - current_pose.pose.position.z);
    //       //cout << " dx " << deltaX << " dy " << deltaY << " dz " << deltaZ << endl;
    //       float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
    //       cout << dMag << endl;
    //       if( dMag < tollorance)
    //       {
    //         break;
    //       }
    //       ros::spinOnce();
    //       ros::Duration(0.5).sleep();
    //       if(i == 1)
    //       {
    //         ROS_INFO("Failed to reach destination. Stepping to next task.");
    //       }
    //     }
    //     ROS_INFO("Done moving foreward.");
    //   }

    /*******************test run starts*******************/

    // float yaw = 45*(M_PI/180);//YAW: from X(E) to Y(N) in ENU frame with X(E) axis being 0 deg
    // float pitch = 0;//level
    // float roll = 0;//level

    // float cy = cos(yaw * 0.5);
    // float sy = sin(yaw * 0.5);
    // float cr = cos(roll * 0.5);
    // float sr = sin(roll * 0.5);
    // float cp = cos(pitch * 0.5);
    // float sp = sin(pitch * 0.5);

    // float qw = cy * cr * cp + sy * sr * sp;
    // float qx = cy * sr * cp - sy * cr * sp;
    // float qy = cy * cr * sp + sy * sr * cp;
    // float qz = sy * cr * cp - cy * sr * sp;

    // pose.pose.orientation.w = qw;
    // pose.pose.orientation.x = qx;
    // pose.pose.orientation.y = qy;
    // pose.pose.orientation.z = qz;

    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = 0;
    // local_pos_pub.publish(pose);

    /*******************test run ends*********************/
    //S sleep(10);
    //land
    land(nh);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
