#pragma once

#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <string.h>

#include <drone_offb_ctrl/image_converter.hpp>

enum D435_Cam_Type
{
    D435_RGB_Cam = 0,
    D435_Depth_Cam = 1
};

class d435_subscriber
{
private:
    ros::NodeHandle nh;  // 句柄
    cv::Mat value_cover; //（显示用）覆盖遮罩
    uint16_t dep_mindst_set;
    uint16_t dep_maxdst_set;

public:
    image_converter RGB_sub;
    image_converter Depth_sub;

    d435_subscriber(ros::NodeHandle &_nh, const std::string &RGB_topic, const std::string &Depth_topic, uint32_t queue_size);
    ~d435_subscriber();
    bool new_img_pair_come();
    cv::Mat getCam(D435_Cam_Type cam, uint16_t dep_mindst, uint16_t dep_maxdst);
    cv::Mat getDepDisplay(cv::Mat dep_16uc1);
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

// 构造函数
d435_subscriber::d435_subscriber(ros::NodeHandle &_nh,
                                 const std::string &RGB_topic = "/camera/color/image_raw",
                                 const std::string &Depth_topic = "/camera/aligned_depth_to_color/image_raw",
                                 uint32_t queue_size = 10)

    : RGB_sub(_nh, RGB_topic, queue_size, sensor_msgs::image_encodings::BGR8), Depth_sub(_nh, Depth_topic, queue_size, sensor_msgs::image_encodings::TYPE_32FC1)
{
    nh = _nh;
}

// 析构函数
d435_subscriber::~d435_subscriber()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////

bool d435_subscriber::new_img_pair_come()
{
    return (RGB_sub.new_img_come() && Depth_sub.new_img_come());
}

// 提取图像
cv::Mat d435_subscriber::getCam(D435_Cam_Type cam, uint16_t dep_mindst = 0, uint16_t dep_maxdst = 6000)
{
    switch (cam)
    {
    case D435_RGB_Cam:
        return this->RGB_sub.get_cv_img();

    case D435_Depth_Cam:
    {
        cv::Mat dep_32fc1 = this->Depth_sub.get_cv_img();
        this->dep_mindst_set = dep_mindst;
        this->dep_maxdst_set = dep_maxdst;
        //dep_16uc1是16bit单通道图像，与dep_32fc1之间等值(因为dep_32fc1只出现整数且最大值为65535)
        cv::Mat dep_16uc1(dep_32fc1.rows, dep_32fc1.cols, CV_16UC1);
        // cv::GaussianBlur(dep_32fc1,dep_32fc1,cv::Size(5,5),2,2);
        //去除不信任数据
        threshold(dep_32fc1, dep_32fc1, dep_mindst, 0xFFFF, CV_THRESH_TOZERO);     //将比最小信任距离小的点归零
        threshold(dep_32fc1, dep_32fc1, dep_maxdst, 0xFFFF, CV_THRESH_TOZERO_INV); //将比最大信任距离大的点归零
        dep_32fc1.convertTo(dep_16uc1, CV_16UC1);
        return dep_16uc1;
    }
    }
}

cv::Mat d435_subscriber::getDepDisplay(cv::Mat dep_16uc1)
{
    inRange(dep_16uc1, dep_mindst_set, dep_maxdst_set, this->value_cover);
    //显示用图像
    cv::Mat Dep_HSV(dep_16uc1.rows, dep_16uc1.cols, CV_8UC3);
    //dep_8uc1为8bit单通道图像，用于显示
    cv::Mat dep_8uc1(dep_16uc1.rows, dep_16uc1.cols, CV_8UC1);
    //转换格式，将最大信任值以内的数值（包括小于最小信任值的）归一化到120（仍然可能出现大于120的值）
    dep_16uc1.convertTo(dep_8uc1, CV_8UC1, 120.0 / this->dep_maxdst_set);

    //用于显示的部分
    std::vector<cv::Mat> channels_hsv;
    cv::Mat blank_ch, full_ch;
    // blank_ch = cv::Mat::zeros(cv::Size(dep_8uc1.cols, dep_8uc1.rows), CV_8UC1);
    full_ch = cv::Mat(cv::Size(dep_8uc1.cols, dep_8uc1.rows), CV_8UC1, 255);
    //合成HSV图像
    channels_hsv.push_back(dep_8uc1);
    channels_hsv.push_back(full_ch);
    channels_hsv.push_back(this->value_cover);
    cv::merge(channels_hsv, Dep_HSV);
    cv::cvtColor(Dep_HSV, Dep_HSV, CV_HSV2BGR);
    return Dep_HSV;
}

/////

//////////////////////////////////////////////////////////////////////////////////////////////
