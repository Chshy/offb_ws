#pragma once
#include <opencv2/opencv.hpp>

void FindVertivalBar(cv::Mat dep_img, std::vector<cv::Rect2i> &reslt)
{
    //先用（9，9）的闭运算清除深度图中的空洞
    cv::morphologyEx(dep_img, dep_img, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9)));
    //然后用非对称卷积核找竖直方向的countor
    //可视场地内容适当缩短长边
    cv::morphologyEx(dep_img, dep_img, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 159)));
    cv::morphologyEx(dep_img, dep_img, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 159)));

    
    cv::Rect2i();
    
}
