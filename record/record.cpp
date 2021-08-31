//
// Created by Harry-hhj on 2021/8/1.
//


#include <umt/umt.hpp>
#include <sensors.hpp>

#include <iostream>
#include <fmt/format.h>
#include <fmt/color.h>
#include <thread>
#include <string>
#include <ctime>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <pybind11/numpy.h>

using namespace std::chrono;
namespace py = pybind11;

void video_record(const std::string &storage_location = "../data/") {
    /*
     * 比赛视频录制函数，强制关机会出现未更新文件头的情况，数据不会丢失
     */
    std::cout << "============ video_writer ===========" << std::endl;

    char now[64];
    std::time_t tt;
    struct tm *ttime;
    tt = time(nullptr);
    ttime = localtime(&tt);
    strftime(now, 64, "%Y-%m-%d_%H_%M_%S", ttime);  // 以时间为名字
    std::string now_string(now);
    std::string path(std::string(storage_location + now_string).append(".avi"));
    try
    {
        auto writer = cv::VideoWriter(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20.0, cv::Size(1280, 768));
        if (!writer.isOpened()) {
            std::cerr << "can't open file" << std::endl;
            return;
        }
//        system(std::string("sudo chmod 777 ").append(path).c_str());
        umt::Subscriber<SensorsData> data_sub("sensors_data");
        while (true) {
            try {
                const auto &[im, q, t] = data_sub.pop();
                writer.write(im);
            }
            catch (umt::MessageError &e) {
                fmt::print(fmt::fg(fmt::color::orange), "[WARNING] 'im_data' {}\n", e.what());
                std::this_thread::sleep_for(500ms);
            }
        }
    } catch (...) {
        fmt::print(fmt::fg(fmt::color::red), "[ERROR] RECORD VIDEO FAILED!\n");
        return;
    }
}

void background_video_record(const std::string &storage_location) {
    std::cerr << "============video_writer===========\n";
    std::thread([=](){
        video_record(storage_location);
    }).detach();
}

PYBIND11_EMBEDDED_MODULE(Record, m) {
    namespace py = pybind11;
    m.def("background_video_record", background_video_record, py::arg("storage_location"));
}