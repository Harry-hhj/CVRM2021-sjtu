//
// Created by xinyang on 2021/3/6.
//

// Modified by Harry-hhj on 2021/05/04

#ifndef CVRM2021_SENSORS_HPP
#define CVRM2021_SENSORS_HPP

#include <opencv2/core.hpp>

struct AutoExposureParam {
    double min_brightness = 60;
    double max_brightness = 100;
    double step_exposure_us = 200;
    double min_exposure_us = 1000;
    double max_exposure_us = 9000;
};

struct SensorsData {
    cv::Mat im;
    std::array<double, 4> q;
    double timestamp; // ms
};

struct SensorParam {
    cv::Mat K;
    cv::Mat D;
    cv::Mat Tcb;
};


#endif //CVRM2021_SENSORS_HPP
