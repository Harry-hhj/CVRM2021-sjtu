//
// Created by Harry-hhj on 2021/5/4.
//

#ifndef CVRM2021_AUTOAIM_HPP
#define CVRM2021_AUTOAIM_HPP

#include <array>
#include <opencv2/opencv.hpp>
#include "detector/TRTModule.hpp"

struct Detection_pack{
    /*
     * 打包数据结构，将识别结果、对应的图像、陀螺仪和时间戳对应
     */
    std::vector<bbox_t> detection;
    cv::Mat img;
    std::array<double, 4> q;
    double timestamp;
};

#endif //CVRM2021_AUTOAIM_HPP
