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

#include "drone_offb_ctrl/MissionPoint.hpp"
#include <vector>
#include "JetsonGPIO.h"
#include "drone_offb_ctrl/GPIOCtrl.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <drone_offb_ctrl/image_converter.hpp>
#include <drone_offb_ctrl/d435_subscriber.hpp>

using namespace std;
using namespace mavros_msgs;

//Set global variables
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
/*pos_z and speed_xy info from t265 variables */
geometry_msgs::TwistWithCovarianceStamped VisionSpeed2;
geometry_msgs::PoseStamped VisionPose;
/*pos_z and speed_xy info from t265 variables end */
geometry_msgs::PoseStamped pose;
std_msgs::Float64 current_heading;
float GYM_OFFSET;
tf2_ros::Buffer tfBuffer;
ros::Publisher set_gp_origin_pub;
ros::Publisher set_raw_pub;
ros::Publisher local_pos_pub;

bool T265_Present = false;

void ExitRosNode()
{
    GPIO::cleanup();
    ros::shutdown();
    return;
}

double RestrianValue(double x, double limit) //该函数默认limit>0!!!!!!!!!!
{
    return fabs(x) <= fabs(limit) ? x : (x >= 0 ? limit : -limit);
}

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

//单位是米
int takeoff(ros::NodeHandle &nh, double height) //meters
{
    ROS_INFO("OFFB: Requesting takeoff...");
    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = height;
    if (takeoff_cl.call(srv_takeoff))
    {
        // ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
        ROS_INFO("OFFB: Takeoff service called.");
        if (srv_takeoff.response.success)
        {
            ROS_INFO("OFFB: Takeoff Sucessful.");
            return 0;
        }
        else
        {
            ROS_ERROR("OFFB: Takeoff Failed.");
            return -1;
        }
    }
    else
    {
        // ROS_ERROR("Failed Takeoff");
        ROS_ERROR("OFFB: Call takeoff service Failed.");
        return -2;
    }
    return 0;
}

int land(ros::NodeHandle &nh)
{
    ROS_INFO("OFFB: Requesting land...");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    if (land_client.call(srv_land))
    {
        //ROS_INFO("land sent %d", srv_land.response.success);
        ROS_INFO("OFFB: Land service called.");
        if (srv_land.response.success)
        {
            ROS_INFO("OFFB: Land Sucessful.");
            return 0;
        }
        else
        {
            ROS_ERROR("OFFB: Land Failed.");
            return -1;
        }
    }
    else
    {
        ROS_ERROR("OFFB: Call takeoff service Failed.");
        return -2;
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
    raw_target.velocity.y = -y;
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
    T265_Present = true;
    //注意:此函数内针对T265朝后装进行变换，获取到的数据坐标系说明如下:
    //位置:以T265启动时刻飞机位置为原点,FLU坐标系,启动时Yaw=0,从上看逆时针为Yaw正方向
    //速度:测定时刻飞机FLU坐标系
    //Pitch Roll 及角速度未测定!!!!!!!

    //记录位置信息(相对于T265启动位置)
    VisionPose.pose.position.x = -odom->pose.pose.position.x;
    VisionPose.pose.position.y = -odom->pose.pose.position.y;
    VisionPose.pose.position.z = odom->pose.pose.position.z;
    //记录角度信息(相对于T265启动位置)
    VisionPose.pose.orientation.x = odom->pose.pose.orientation.x;
    VisionPose.pose.orientation.y = odom->pose.pose.orientation.y;
    VisionPose.pose.orientation.z = odom->pose.pose.orientation.z;
    VisionPose.pose.orientation.w = odom->pose.pose.orientation.w;
    //记录速度信息(飞机右后下)
    VisionSpeed2.twist.twist.linear.x = -odom->twist.twist.linear.x;
    VisionSpeed2.twist.twist.linear.y = -odom->twist.twist.linear.y;
    VisionSpeed2.twist.twist.linear.z = odom->twist.twist.linear.z;
    // ROS_INFO("%5.3lf %5.3lf %5.3lf %5.3lf %5.3lf %5.3lf %5.3lf %5.3lf %5.3lf %5.3lf",\
    // VisionPose.pose.position.x = odom->pose.pose.position.x,
    // VisionPose.pose.position.y = odom->pose.pose.position.y,
    // VisionPose.pose.position.z = odom->pose.pose.position.z,
    // VisionPose.pose.orientation.x = odom->pose.pose.orientation.x,
    // VisionPose.pose.orientation.y = odom->pose.pose.orientation.y,
    // VisionPose.pose.orientation.z = odom->pose.pose.orientation.z,
    // VisionPose.pose.orientation.w = odom->pose.pose.orientation.w,
    // VisionSpeed2.twist.twist.linear.x = odom->twist.twist.linear.x,
    // VisionSpeed2.twist.twist.linear.y = odom->twist.twist.linear.y,
    // VisionSpeed2.twist.twist.linear.z = odom->twist.twist.linear.z
    // );
}

enum MissionPayloadDefine
{
    MissionPayload_None = 0,        //无任务
    MissionPayload_Laser = 1,       //打点
    MissionPayload_DetectLaser = 2, //判断后打点
    MissionPayload_FindBar = 3,
    MissionPayload_ScanCode = 4
};

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

    GPIOController CtrlPanel(12, 7, 8, 25,
                             0, 0, 0, 0, 0, 0,
                             13, 19, 26,
                             16, 17);

    // allow the subscribers to initialize
    // 等待一些东西初始化
    CtrlPanel.WriteLED(1, 1);
    CtrlPanel.WriteLED(2, 1);
    CtrlPanel.WriteLED(3, 1);
    ROS_INFO("INITILIZING...");
    for (int i = 0; i < 250; i++)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    //检查T265信息是否发布
    bool bits = 0;
    uint8_t display_digit = 1;
    uint8_t display_cnt = 0;
    CtrlPanel.WriteLED(1, 0);
    CtrlPanel.WriteLED(2, 0);
    CtrlPanel.WriteLED(3, 0);
    while (!T265_Present) //wait for remote command
    {
        ROS_INFO("OFFB: Waiting for T265 Publish ...");
        ros::spinOnce();
        ros::Duration(0.01).sleep();

        if (CtrlPanel.ReadKey(1))
        {
            CtrlPanel.WriteBeep(1);
            ros::Duration(2.0).sleep();
            CtrlPanel.WriteBeep(0);
            ExitRosNode();
            return 0;
        }
        //灯光闪烁
        display_cnt++;
        if (display_cnt >= 50)
        {
            display_cnt = 0;
            CtrlPanel.WriteLED(3, bits = !bits);
        }
    }

    display_digit = 1;
    display_cnt = 0;
    image_converter usbcam(nh, "/usb_cam/image_raw", 10, sensor_msgs::image_encodings::BGR8);

    // ROS_INFO("Enter Test");
    // // cv::namedWindow("123", cv::WINDOW_NORMAL);
    // // cv::resizeWindow("123", 320, 320);
    // while (1)
    // {
    //     // ROS_INFO("OFFB: Detect Block.");
    //     ros::spinOnce();
    //     ros::Duration(0.01).sleep();
    //     if (usbcam.new_img_come())
    //     {
    //         //获取最新图像
    //         cv::Mat USBCam = usbcam.get_cv_img();
    //         cv::imshow("1234", USBCam);
    //         //定义一些参数
    //         static const uint16_t CetnerX = 308, CenterY = 235, CenterR = 10;
    //         static const cv::Scalar LowerBound(65, 53, 108), UpperBound(90, 255, 255);
    //         static const cv::Rect CenterRect(CetnerX - CenterR, CenterY - CenterR, 2 * CenterR, 2 * CenterR);
    //         //切割中央图像

    //         //根据阈值二值化，滤波哦，求均值
    //         cv::cvtColor(USBCam, USBCam, cv::COLOR_BGR2HSV);
    //         cv::inRange(USBCam, LowerBound, UpperBound, USBCam);
    //         cv::Mat kernel_size = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    //         // cv::morphologyEx(CenterImg, CenterImg, cv::MORPH_OPEN, kernel_size);
    //         // cv::morphologyEx(CenterImg, CenterImg, cv::MORPH_CLOSE, kernel_size);
    //         cv::Mat CenterImg = USBCam(CenterRect);
    //         // cv::imshow("123", CenterImg);
    //         cv::cvtColor(USBCam, USBCam, cv::COLOR_GRAY2BGR);
    //         cv::imshow("12345", USBCam);
    //         double m = cv::mean(CenterImg)[0];
    //         //判断均值大小
    //         ROS_INFO("OFFB: m = %lf.", m);
    //         // if (m > 180.0)
    //         // {
    //         //     CtrlPanel.WriteLaser(1);
    //         // }
    //     }
    //     else
    //     {
    //         ROS_WARN("OFFB: Capture Image Failed!");
    //     }
    //     cv::waitKey(1);
    //     if (CtrlPanel.ReadKey(1))
    //     {
    //         CtrlPanel.WriteBeep(1);
    //         ros::Duration(2.0).sleep();
    //         CtrlPanel.WriteBeep(0);
    //         ExitRosNode();
    //         return 0;
    //     }
    // }

    // // ROS_INFO("Enter Test");
    // // while (1)
    // // {

    // //     ros::spinOnce();

    // //     if (usbcam.new_img_come())
    // //     {
    // //         cv::Mat src = usbcam.get_cv_img();

    // //         //设定测量范围
    // //         int xx, yy, rr;
    // //         xx = 308; //320
    // //         yy = 235; //240
    // //         rr = 10;
    // //         cv::Rect center_rect(xx - rr, yy - rr, 2 * rr, 2 * rr);

    // //         cv::Mat center_img = src(center_rect);
    // //         cv::Mat center_img_proc; //, center_img_inrange;
    // //         cv::Mat tmpmat;

    // //         cv::cvtColor(center_img, center_img_proc, cv::COLOR_BGR2HSV);
    // //         cv::inRange(center_img_proc, cv::Scalar(72, 0, 108), cv::Scalar(77, 255, 255), center_img_proc);
    // //         cv::Mat kernel_size = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    // //         cv::morphologyEx(center_img_proc, center_img_proc, cv::MORPH_OPEN, kernel_size);
    // //         cv::morphologyEx(center_img_proc, center_img_proc, cv::MORPH_CLOSE, kernel_size);
    // //         cv::cvtColor(center_img_proc, tmpmat, cv::COLOR_GRAY2BGR);
    // //         cv::imshow("tmpmat", tmpmat);

    // //         // cv::Mat tmp_m, tmp_sd;
    // //         double m = 0, sd = 0;
    // //         m = cv::mean(center_img_proc)[0];
    // //         // cout << "Mean: " << m << endl;
    // //         // meanStdDev(gray, tmp_m, tmp_sd);
    // //         // m = tmp_m.at<double>(0, 0);
    // //         // sd = tmp_sd.at<double>(0, 0);
    // //         // cout << "Mean: " << m << " , StdDev: " << sd << endl;
    // //         // char tmpstr[64];
    // //         // sprintf(tmpstr, "Mean=%lf", m);
    // //         // cv::putText(src, "", cv::Point(5, 5), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 0), 3);
    // //         ROS_INFO("OFFB: CV: Mean=%lf", m);
    // //         cv::line(src, cv::Point(xx, 0), cv::Point(xx, 480), cv::Scalar(0, 0, 0));
    // //         cv::line(src, cv::Point(0, yy), cv::Point(640, yy), cv::Scalar(0, 0, 0));
    // //         // cv::circle(src, cv::Point(xx, yy), rr, cv::Scalar(0, 0, 0));
    // //         cv::rectangle(src, center_rect, cv::Scalar(0, 0, 0));
    // //         cv::imshow("usb_cam", src);
    // //         // cv::imshow("center_img_inrange", center_img_inrange);

    // //         cv::Mat src1, src2, src3;
    // //         cv::cvtColor(src, src1, cv::COLOR_BGR2HSV);
    // //         cv::inRange(src1, cv::Scalar(72, 0, 108), cv::Scalar(77, 255, 255), src2);
    // //         kernel_size = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    // //         cv::morphologyEx(src2, src2, cv::MORPH_OPEN, kernel_size);
    // //         cv::morphologyEx(center_img_proc, center_img_proc, cv::MORPH_CLOSE, kernel_size);
    // //         cv::cvtColor(src2, src3, cv::COLOR_GRAY2BGR);
    // //         cv::imshow("usb_cam22", src3);
    // //     }
    // //     cv::waitKey(1);

    // //     ros::Duration(0.01).sleep();

    // //     if (CtrlPanel.ReadKey(1))
    // //     {
    // //         CtrlPanel.WriteBeep(1);
    // //         ros::Duration(2.0).sleep();
    // //         CtrlPanel.WriteBeep(0);
    // //         ExitRosNode();
    // //         return 0;
    // //     }
    // //     //灯光闪烁
    // //     display_cnt++;
    // //     if (display_cnt >= 50)
    // //     {
    // //         display_cnt = 0;
    // //         CtrlPanel.WriteLaser(bits = !bits);
    // //     }
    // // }

    // 等待飞控将 custom mode 设置为 OFFBOARD
    display_digit = 1;
    display_cnt = 0;

#define MAX_MISSION_NUM 3
    int SelectedMissionInd = 1; //从1开始
    //1.检测颜色
    //2.检测颜色,找杆子
    //3.基本要求
    while (current_state.mode != "OFFBOARD") //wait for remote command
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
        ROS_INFO("OFFB: Waiting for FCU to set OFFBOARD mode...");
        if (CtrlPanel.ReadKey(1))
        {
            CtrlPanel.WriteBeep(1);
            ros::Duration(2.0).sleep();
            CtrlPanel.WriteBeep(0);
            ExitRosNode();
            return 0;
        }
        if (CtrlPanel.ReadKey(2))
        {
            SelectedMissionInd++;
            if (SelectedMissionInd > MAX_MISSION_NUM)
            {
                SelectedMissionInd = 1;
            }

            for (int i = 0; i < SelectedMissionInd; i++)
            {
                CtrlPanel.WriteBeep(1);
                ros::Duration(0.2).sleep();
                CtrlPanel.WriteBeep(0);
                ros::Duration(0.2).sleep();
            }
        }
        //灯光闪烁
        display_cnt++;
        if (display_cnt >= 50)
        {
            display_cnt = 0;
            display_digit *= 2;
            if (display_digit > 7)
                display_digit = 1;
            CtrlPanel.WriteLED(1, display_digit & 1);
            CtrlPanel.WriteLED(2, display_digit & 2);
            CtrlPanel.WriteLED(3, display_digit & 4);
        }
    }
    ROS_INFO("OFFB: OFFBOARD mode confirmed.");
    CtrlPanel.WriteLED(1, 0);
    CtrlPanel.WriteLED(2, 0);
    CtrlPanel.WriteLED(3, 0);
    CtrlPanel.WriteBeep(1);
    //等0.5秒
    for (int i = 0; i < 50; i++)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    // 记录起飞时的坐标信息(没看，不知道怎么用)
    send_tf_snapshot_takeoff();

    // recheck for FCU connection
    // 再次检查FCU连接情况
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("OFFB: FCU connection confirmed. Start executing command in 1 sec.");
    // 解锁
    // arm_drone(nh);

    //等一秒
    for (int i = 0; i < 100; i++)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    CtrlPanel.WriteBeep(0);

    // //request takeoff
    // 发送起飞命令
    if (takeoff(nh, 1.5))
    { //如果起飞失败
        //发出报警
        //退出程序
        ExitRosNode();
        return -1;
    }

    //等8.5秒 等待上升到指定高度
    ROS_INFO("OFFB: Waiting for climbing...");
    for (int i = 0; i < 850; i++)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    //开始水平运动
    ROS_INFO("OFFB: Start horizontal movement.");
    // #define TOTAL_STEP 4
    // #define MAX_CTRL_STEP 400
    // #define P_FACTOR 0.55
    // const double dxStorage[TOTAL_STEP] = {1.5, 1.5, 0, 0};
    // const double dyStorage[TOTAL_STEP] = {0, 1.5, 1.5, 0};
    vector<MissionPoint> MPStorage;
    if (SelectedMissionInd == 1)
    {
        //到A
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 1.85, -0.5));
        //向前1
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0.5, 0));
        //向右6
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, -0.5));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, -0.5));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, -0.5));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, -0.5));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, -0.5));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, -0.5));
        //向后5
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, -0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, -0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, -1, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, -0.5, 0));
        //向左1
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, 0.5));
        //向前4
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 1, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0.5, 0));
        //向左1
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, 0.5));
        //向后4
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, -0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, -0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, -0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, -0.5, 0));
        //向左1
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, 0.5));
        //向前4
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0.5, 0));
        //向左2
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, 0.5));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, 0.5));
        //回0
        // MPStorage.push_back(MissionPoint(MissionPayload_None, MPStorageMethod_ABSOLUTE, 0, 0)); //30
        MPStorage.push_back(MissionPoint(MissionPayload_None, MPStorageMethod_ABSOLUTE, 0, -0.025, 0.1, 0.1, 1.1));     //30
        MPStorage.push_back(MissionPoint(MissionPayload_None, MPStorageMethod_ABSOLUTE, 0, -0.025, 0.025, 0.025, 2.0)); //30
    }
    else if (SelectedMissionInd == 2)
    {
        //到A
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 1.85, -0.5));
        //向前1
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0.5, 0));
        //向右6
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, -0.5));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, -0.5));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, -0.5));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, -0.5));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, -0.5));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, -0.5));
        //向后5
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, -0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, -0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, -1, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, -0.5, 0));
        //向左1
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, 0.5));
        //向前4
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 1, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0.5, 0));
        //向左1
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, 0.5));
        //向后4
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, -0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, -0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, -0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, -0.5, 0));
        //向左1
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, 0.5));
        //向前4
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0.5, 0));
        //向左2
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, 0.5));
        MPStorage.push_back(MissionPoint(MissionPayload_DetectLaser, MPStorageMethod_RELATIVE, 0, 0.5));
        //回0
        // MPStorage.push_back(MissionPoint(MissionPayload_None, MPStorageMethod_ABSOLUTE, 0, 0)); //30
        MPStorage.push_back(MissionPoint(MissionPayload_None, MPStorageMethod_ABSOLUTE, 0, -0.025, 0.1, 0.1, 1.1));     //30
        MPStorage.push_back(MissionPoint(MissionPayload_None, MPStorageMethod_ABSOLUTE, 0, -0.025, 0.025, 0.025, 2.0)); //30
    }
    else if (SelectedMissionInd == 3)
    {
        //到A
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 1.85, -0.5));
        //向前1
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 0.5, 0));
        //向右6
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 0, -0.5));
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 0, -0.5));
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 0, -0.5));
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 0, -0.5));
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 0, -0.5));
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 0, -0.5));
        //向后5
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, -0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, -0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, -1, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, -0.5, 0));
        //向左1
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 0, 0.5));
        //向前4
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 1, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 0.5, 0));
        //向左1
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 0, 0.5));
        //向后4
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, -0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, -0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, -0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, -0.5, 0));
        //向左1
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 0, 0.5));
        //向前4
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 0.5, 0));
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 0.5, 0));
        //向左2
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 0, 0.5));
        MPStorage.push_back(MissionPoint(MissionPayload_Laser, MPStorageMethod_RELATIVE, 0, 0.5));
        //回0
        // MPStorage.push_back(MissionPoint(MissionPayload_None, MPStorageMethod_ABSOLUTE, 0, 0)); //30
        MPStorage.push_back(MissionPoint(MissionPayload_None, MPStorageMethod_ABSOLUTE, 0, -0.025, 0.1, 0.1, 1.1));     //30
        MPStorage.push_back(MissionPoint(MissionPayload_None, MPStorageMethod_ABSOLUTE, 0, -0.025, 0.025, 0.025, 2.0)); //30
    }

    // for (std::vector<Vocabulary>::size_type it = 0; it < list.size(); ++it)
    // {
    //     if ((list[it].word.compare(input)) == 0) //两个字符串相等
    //     {
    //         return it;
    //     }
    // }

    double dest_x = 0, dest_y = 0, dest_z = 0, dest_yaw = 0; //目标位置(Gym Frame)
    for (std::vector<MissionPoint>::size_type MissionStep = 0; MissionStep < MPStorage.size(); ++MissionStep)
    // for (int MissionStep = 0; MissionStep < TOTAL_STEP; MissionStep++)
    {
        ROS_INFO("OFFB: Current control step: %d.", (int)MissionStep);
        //---------------------------------------------
        // double RestrainSpeed = 0.4; // 限速 m/s

        //当前值
        double curr_x = 0, curr_y = 0, curr_z = 0; //当前位置(Gym Frame)
        double curr_roll = 0, curr_pitch = 0, curr_yaw = 0;

        //误差值
        double dx = 0, dy = 0, dz = 0, dw = 0;
        double df_body = 0, dl_body = 0;

        //////////////////////////////////////////////

        // set_speed_body(dF[MissionStep], dF[MissionStep], 0); //flu

        //获取当前T265位置(回调函数内已取反)
        // ros::spinOnce();
        // curr_x = VisionPose.pose.position.x;
        // curr_y = VisionPose.pose.position.y;
        // curr_z = VisionPose.pose.position.z;

        //从数组读取这一步相对于上一步移动的距离(m)并保存
        switch (MPStorage[MissionStep].StorgMeth)
        {
        case MPStorageMethod_ABSOLUTE:
        {
            dest_x = MPStorage[MissionStep].xCoordinate;
            dest_y = MPStorage[MissionStep].yCoordinate;
        }
        break;
        case MPStorageMethod_RELATIVE:
        {
            dest_x += MPStorage[MissionStep].xCoordinate;
            dest_y += MPStorage[MissionStep].yCoordinate;
        }
        break;
        default:
            break;
        }

        // dest_x = dxStorage[MissionStep];
        // dest_y = dyStorage[MissionStep];

        ROS_INFO("OFFB: Moving to %lf %lf...(%lf sec MAX)", dest_x, dest_y, MPStorage[MissionStep].IntervalSleepTime * MPStorage[MissionStep].MaxCtrlStep);
        int CtrlStep;
        for (CtrlStep = 0; CtrlStep < MPStorage[MissionStep].MaxCtrlStep; CtrlStep++)
        {
            //获取当前T265位置(回调函数内已取反)
            ros::spinOnce();
            curr_x = VisionPose.pose.position.x;
            curr_y = VisionPose.pose.position.y;
            curr_z = VisionPose.pose.position.z;

            //获取当前Yaw角度
            tf2::Quaternion q(
                VisionPose.pose.orientation.x,
                VisionPose.pose.orientation.y,
                VisionPose.pose.orientation.z,
                VisionPose.pose.orientation.w);
            q.normalize();
            tf2::Matrix3x3 m(q);
            m.getRPY(curr_roll, curr_pitch, curr_yaw);

            //计算位移矢量(以当前飞机为起点,目的地为终点)
            dx = dest_x - curr_x;
            dy = dest_y - curr_y;
            // dz = dest_z - curr_z;
            // dw = dest_w - curr_w;

            //判断误差是否在容忍范围内,如果在则退出控制
            if (MPStorage[MissionStep].inRange(dx, dy))
            {
                break;
            }

            //将位移矢量旋转至BodyHeading(FLU)下,用于控制飞机速度
            df_body = dx * cos(-curr_yaw) - dy * sin(-curr_yaw);
            dl_body = dx * sin(-curr_yaw) + dy * cos(-curr_yaw);

            //乘以P控制参数 并限幅
            df_body = RestrianValue(df_body * MPStorage[MissionStep].P_FACTOR, MPStorage[MissionStep].RestrainSpeed);
            dl_body = RestrianValue(dl_body * MPStorage[MissionStep].P_FACTOR, MPStorage[MissionStep].RestrainSpeed);

            ROS_INFO("OFFB: x=%lf y=%lf z=%lf yaw=%lf df=%lf dl=%lf", curr_x, curr_y, curr_z, curr_yaw, df_body, dl_body);

            set_speed_body(df_body, dl_body, 0); //FLU坐标系
            ros::Duration(MPStorage[MissionStep].IntervalSleepTime).sleep();
            // ROS_INFO("CB:%lf %lf %lf\n", VisionSpeed2.twist.twist.linear.x, VisionSpeed2.twist.twist.linear.y, VisionSpeed2.twist.twist.linear.z);
        }

        if (CtrlStep >= MPStorage[MissionStep].MaxCtrlStep)
        {
            ROS_WARN("OFFB: Control Timeout. Position may be imprecise!");
        }
        else
        {
            ROS_INFO("OFFB: Destination (%lf,%lf) reached.", dest_x, dest_y);
        }

        ROS_INFO("OFFB: Breaking...(%lf sec)", MPStorage[MissionStep].BreakIntervalSleepTime * MPStorage[MissionStep].BreakStep);
        CtrlPanel.WriteBeep(1);

        if (MPStorage[MissionStep].MissionPayload == MissionPayload_Laser)
        {
            CtrlPanel.WriteLaser(1);
        }
        else if (MPStorage[MissionStep].MissionPayload == MissionPayload_DetectLaser)
        {
            ROS_INFO("OFFB: Detect Block.");
            ros::spinOnce();
            if (usbcam.new_img_come())
            {
                //获取最新图像
                cv::Mat USBCam = usbcam.get_cv_img();
                //定义一些参数
                static const uint16_t CetnerX = 308, CenterY = 235, CenterR = 10;
                static const cv::Scalar LowerBound(65, 53, 108), UpperBound(90, 255, 255);
                static const cv::Rect CenterRect(CetnerX - CenterR, CenterY - CenterR, 2 * CenterR, 2 * CenterR);
                //切割中央图像
                cv::Mat CenterImg = USBCam(CenterRect);
                //根据阈值二值化，滤波哦，求均值
                cv::cvtColor(CenterImg, CenterImg, cv::COLOR_BGR2HSV);
                cv::inRange(CenterImg, LowerBound, UpperBound, CenterImg);
                // cv::Mat kernel_size = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
                // cv::morphologyEx(CenterImg, CenterImg, cv::MORPH_OPEN, kernel_size);
                // cv::morphologyEx(CenterImg, CenterImg, cv::MORPH_CLOSE, kernel_size);
                double m = cv::mean(CenterImg)[0];
                //判断均值大小
                ROS_INFO("OFFB: m = %lf.", m);
                if (m > 180.0)
                {
                    CtrlPanel.WriteLaser(1);
                }
            }
            else
            {
                ROS_WARN("OFFB: Capture Image Failed!");
            }
        }

        // if (MissionStep != 10 && MissionStep != 15 && MissionStep != 30)

        for (int BreakCtrlStep = 0; BreakCtrlStep < MPStorage[MissionStep].BreakStep; BreakCtrlStep++)
        {
            ros::spinOnce();
            set_speed_body(0, 0, 0);
            ros::Duration(MPStorage[MissionStep].BreakIntervalSleepTime).sleep();
        }
        CtrlPanel.WriteLaser(0);
        CtrlPanel.WriteBeep(0);
    }

    //尝试降落
    uint8_t land_time;
    for (land_time = 1; land(nh) < 0 && land_time <= 3; land_time++)
    {
        ROS_WARN("OFFB: Try again.(%d)", land_time);
    };

    if (land_time > 3)
    {
        ROS_ERROR("OFFB: Failed landing! Please Control Manually!!");
        ExitRosNode();
        return -1;
    }
    else
    {
        ROS_INFO("OFFB: Land Sucessful.");
    }

    ROS_INFO("OFFB: Waiting for landing...");
    for (int i = 0; i < 1000; i++)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    int8_t disarm_time;
    for (disarm_time = 1; arm_drone(nh) < 0 && disarm_time <= 3; disarm_time++)
    {
        ROS_WARN("OFFB: Try again.(%d)", disarm_time);
    };

    if (disarm_time > 3)
    {
        ROS_ERROR("OFFB: Failed disarming! Please Control Manually!!");
        ExitRosNode();
        return -1;
    }
    else
    {
        ROS_INFO("OFFB: Disarm Sucessful.");
    }

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
