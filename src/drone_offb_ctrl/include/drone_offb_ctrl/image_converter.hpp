#pragma once

#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <string.h>

class image_converter
{
private:
    ros::NodeHandle nh;            //ROS句柄
    ros::Subscriber ros_topic_sub; //订阅的ROS Topic
    std::string converted_format;  //转换的目标格式
    sensor_msgs::Image ros_img;    //存储ROS格式图像
    cv::Mat cv_img;                //存储OpenCV格式图像
    uint32_t ros_img_ind;          //ROS格式图像序号
    uint32_t cv_img_ind;           //OpenCV格式图像序号
    bool cv_img_read;              //本帧图像既阅标记

public:
    image_converter(ros::NodeHandle &_nh, const std::string &_topic, uint32_t queue_size, const std::string &_format);
    ~image_converter();
    void img_cb(const sensor_msgs::Image::ConstPtr &msg); //ROS Topic 回调函数（立刻更新ROS格式图像）
    void convert();                                       //格式转换为OpenCV（将OpenCV格式图像同步为ROS同一帧）
    cv::Mat get_cv_img(bool do_convert, bool do_mark);    //获取显示图像
    bool new_img_come();                                  //检查是否有新图像到来（ROS图像序号大于OpenCV图像）
};

image_converter::image_converter(ros::NodeHandle &_nh,
                                 const std::string &_topic,
                                 uint32_t queue_size = 10,
                                 const std::string &_format = sensor_msgs::image_encodings::BGR8)
{
    nh = _nh;

    converted_format = _format;
    ros_img_ind = 0;
    cv_img_ind = 0;
    cv_img_read = true;

    ros_topic_sub = nh.subscribe<sensor_msgs::Image>(_topic, queue_size, &image_converter::img_cb, this);
}

image_converter::~image_converter()
{
}

void image_converter::img_cb(const sensor_msgs::Image::ConstPtr &msg)
{
    ros_img = *msg;
    ros_img_ind++;
    return;
}

void image_converter::convert()
{
    if (cv_img_ind != ros_img_ind)
    {
        try
        {
            this->cv_img = cv_bridge::toCvCopy(ros_img, converted_format)->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to '%s'.", ros_img.encoding.c_str(), converted_format.c_str());
        }
        cv_img_read = false;
    }
    return;
}

cv::Mat image_converter::get_cv_img(bool do_convert = true, bool do_mark = true)
{
    if (do_convert)
    {
        convert();
    }
    if (do_mark)
    {
        cv_img_read = true;
    }
    return this->cv_img;
}

bool image_converter::new_img_come()
{
    return (ros_img_ind != cv_img_ind);
}
