//
// Created by xinyang on 2021/3/6.
//

// Modified by Harry-hhj on 2021/05/04

#include "sensors.hpp"
#include "MindVision/MindVision.hpp"
#include "ExtImu/ExtImu.hpp"
#include <serial/serial.h>
#include <umt/umt.hpp>
#include <common.hpp>
#include <opencv2/opencv.hpp>
#include <fmt/format.h>
#include <fmt/color.h>
#include <thread>
#include <chrono>
#include <queue>

using namespace serial;
using namespace std::chrono;

static std::mutex seq_q_mtx;
static std::queue<std::array<double, 4>> seq_q;
static bool imu_running = false;
static bool require_imu_stop = false;

static bool debug = true;

bool sync_once(MindVision &camera, ExtImu &imu, int sync_period_ms) {
    if (debug) std::cout << "============sync_once===========\n";
    camera.close();
    require_imu_stop = true;
    imu.stop_trigger();
    while (imu_running);
    seq_q = std::queue<std::array<double, 4>>();
    camera.open();
    if (!camera.isOpen()) {
        fmt::print(fmt::fg(fmt::color::red), "[ERROR]: camera reopen fail!\n");
        return false;
    }
    try {
        imu.periodic_trigger(sync_period_ms);
    } catch (SerialException &e) {
        fmt::print(fmt::fg(fmt::color::red), "[ERROR] {}\n", e.what());
        return false;
    }
    require_imu_stop = false;
    return true;
}

static void imu_capture_loop(ExtImu &imu, const bool &require_stop, bool &is_ok) {
    if (debug) std::cout << "============imu_capture_loop===========\n";
    ExtImu::sensor_data data{};
    imu_running = true;
    while (!require_stop) {
        if (require_imu_stop) {
            imu.flushInput();
            imu_running = false;
            while (require_imu_stop);
            imu_running = true;
        }
        try {
            imu.read_sensor(&data);
        } catch (SerialException &e) {
            fmt::print(fmt::fg(fmt::color::red), "[ERROR] {}\n", e.what());
            is_ok = false;
            break;
        }

        if (data.trigger == 1) {
            //std::cout << "lock acquire" << std::endl;
            std::unique_lock lock(seq_q_mtx);
            //std::cout << "lock release" << std::endl;
            if (!seq_q.empty()) seq_q.pop();
            seq_q.push({data.q[3], data.q[0], data.q[1], data.q[2]});
        }
    }
}

static void auto_exposure_loop(MindVision &camera, const bool &require_stop, bool &is_ok) {
    // disable auto exposure
    return;

    if (debug) std::cout << "============auto_exposure_loop===========\n";

    umt::Subscriber<SensorsData> inputs_sub("sensors_data");
    auto param = umt::ObjManager<AutoExposureParam>::find_or_create("auto_exposure_param");
    while (!require_stop) {
        cv::Mat im;
        try {
            const auto &data = inputs_sub.pop();
            im = data.im;
        } catch (umt::MessageError &e) {
            fmt::print(fmt::fg(fmt::color::red), "[ERROR] 'auto_exposure_loop' {}\n", e.what());
            is_ok = false;
            break;
        }

        double current_exposure_us;
        if (!camera.get_exposure_us(current_exposure_us)) {
            is_ok = false;
            break;
        }
        if (param->min_exposure_us < current_exposure_us &&
            current_exposure_us < param->max_exposure_us) {
            auto mean_channel_brightness = cv::mean(im);
            double mean_brightness = (mean_channel_brightness[0] +
                                      mean_channel_brightness[1] +
                                      mean_channel_brightness[2]) / 3.;
            if (mean_brightness > param->max_brightness) {
                current_exposure_us -= param->step_exposure_us;
                if (!camera.set_exposure_us(current_exposure_us)) {
                    is_ok = false;
                    break;
                }
                fmt::print("exposure set to {}us\n", current_exposure_us);
            } else if (mean_brightness < param->min_brightness) {
                current_exposure_us += param->step_exposure_us;
                if (!camera.set_exposure_us(current_exposure_us)) {
                    is_ok = false;
                    break;
                }
                fmt::print("exposure set to {}us\n", current_exposure_us);
            }
        }
        std::this_thread::sleep_for(200ms);
    }
}


// when imu is not used
/*
bool sensors_io(const std::string &camera_name = "",
                const std::string &camera_cfg = "",
                const std::string &sensor_param_file = "") {
    cv::FileStorage ifs;
    if (!ifs.open(sensor_param_file, cv::FileStorage::READ)) {
        fmt::print(fmt::fg(fmt::color::red), "[ERROR]: sensor parameter read fail!");
        return false;
    }
    auto sensor_param = umt::ObjManager<SensorParam>::find_or_create("sensor_param");
    try {
        ifs["K"] >> sensor_param->K;
        ifs["D"] >> sensor_param->D;
        ifs["Tcb"] >> sensor_param->Tcb;
        if (sensor_param->K.cols != 3 || sensor_param->K.rows != 3) {
            throw std::runtime_error("sensor parameter 'K' invalid format!");
        }
        if (sensor_param->D.cols != 5 || sensor_param->D.rows != 1) {
            throw std::runtime_error("sensor parameter 'D' invalid format!");
        }
        if (sensor_param->Tcb.cols != 3 || sensor_param->Tcb.rows != 3) {
            throw std::runtime_error("sensor parameter 'Tcb' invalid format!");
        }
    } catch (cv::Exception &e) {
        fmt::print(fmt::fg(fmt::color::red), "[ERROR]: {}", e.what());
        return false;
    }

    MindVision camera(camera_name.data(), camera_cfg.data());
    camera.open();
    if (!camera.isOpen()) {
        fmt::print(fmt::fg(fmt::color::red), "[ERROR]: camera init fail!\n");
        return false;
    }

    umt::Publisher<cv::Mat> data_pub("camera");

    bool auto_exposure_require_stop = false;
    bool auto_exposure_is_ok = true;
    std::thread auto_exposure_thread(auto_exposure_loop, std::ref(camera),
                                     std::ref(auto_exposure_require_stop), std::ref(auto_exposure_is_ok));

    auto t1 = high_resolution_clock::now();
    int fps = 0, fps_count = 0;

    while (auto_exposure_is_ok) {
        cv::Mat img;
        if (!camera.read(img)) {
            fmt::print(fmt::fg(fmt::color::red), "Camera read error!\n");
            break;
        }
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
        data_pub.push(img);
    }

    auto_exposure_require_stop = true;
    auto_exposure_thread.join();

    return false;
}


void background_sensors_io_auto_restart(const std::string &camera_name = "", 
                                        const std::string &camera_cfg = "",
                                        const std::string &sensor_param_file = "") {
    std::thread([=]() {
        while (!sensors_io(camera_name, camera_cfg, sensor_param_file)) {
            std::this_thread::sleep_for(500ms);
        }
    }).detach();
}

PYBIND11_EMBEDDED_MODULE(SensorsIO, m) {
    namespace py = pybind11;
    m.def("background_sensors_io_auto_restart", background_sensors_io_auto_restart,
          py::arg("camera_name") = "", py::arg("camera_cfg") = "", py::arg("sensor_param_file") = "");
}
*/

bool sensors_io(const std::string &camera_name = "", const std::string &camera_cfg = "",
                const std::string &sensor_param_file = "",
                const std::string &imu_usb_hid = "", int sync_period_ms = 10) {
    if (debug) std::cout << "============sensors_io===========\n";

    cv::FileStorage ifs;
    if (!ifs.open(sensor_param_file, cv::FileStorage::READ)) {
        fmt::print(fmt::fg(fmt::color::red), "[ERROR]: sensor parameter read fail!");
        return false;
    }
    auto sensor_param = umt::ObjManager<SensorParam>::find_or_create("sensor_param");
    try {
        ifs["K"] >> sensor_param->K;
        ifs["D"] >> sensor_param->D;
        ifs["Tcb"] >> sensor_param->Tcb;
        if (sensor_param->K.cols != 3 || sensor_param->K.rows != 3) {
            throw std::runtime_error("sensor parameter 'K' invalid format!");
        }
        if (sensor_param->D.cols != 5 || sensor_param->D.rows != 1) {
            throw std::runtime_error("sensor parameter 'D' invalid format!");
        }
        if (sensor_param->Tcb.cols != 3 || sensor_param->Tcb.rows != 3) {
            throw std::runtime_error("sensor parameter 'Tcb' invalid format!");
        }
    } catch (cv::Exception &e) {
        fmt::print(fmt::fg(fmt::color::red), "[ERROR]: {}", e.what());
        return false;
    }

    MindVision camera(camera_name.data(), camera_cfg.data());
    camera.open();
    if (!camera.isOpen()) {
        fmt::print(fmt::fg(fmt::color::red), "[ERROR]: camera init fail!\n");
        return false;
    }
    ExtImu imu;
    for (const auto &port_info : list_ports()) {
        std::cerr << "hardware_id: " << port_info.hardware_id << ", port: " << port_info.port << std::endl;
        if (port_info.hardware_id == imu_usb_hid) {
	        std::cerr << "enter_if 1" << std::endl;
            imu.setPort(port_info.port);
	        std::cerr << "after set id" << std::endl;
            auto timeout = Timeout::simpleTimeout(Timeout::max());
	        std::cerr << "after get time" << std::endl;
            //imu.setTimeout(timeout);
	        std::cerr << "after set time" << std::endl;
            break;
        }
	    std::cerr << "not in if\n";
    }
    std::cerr << "before open imu\n";
    imu.open();
    std::cerr << "after open imu\n";
    if (!imu.isOpen()) {
        fmt::print(fmt::fg(fmt::color::red), "[ERROR]: imu init fail!\n");
        return false;
    }

    umt::Publisher<SensorsData> data_pub("sensors_data");
    auto webview_checkbox = umt::ObjManager<CheckBox>::find_or_create("show raw");
    umt::Publisher<cv::Mat> webview_raw("raw");

    bool imu_require_stop = false;
    bool imu_is_ok = true;
    std::thread imu_capture_thread(imu_capture_loop, std::ref(imu),
                                   std::ref(imu_require_stop), std::ref(imu_is_ok));
    std::cerr << "after create imu_capture_thread\n";

    bool auto_exposure_require_stop = false;
    bool auto_exposure_is_ok = true;
    std::thread auto_exposure_thread(auto_exposure_loop, std::ref(camera),
                                     std::ref(auto_exposure_require_stop), std::ref(auto_exposure_is_ok));
    std::cerr << "after create auto_exposure thread\n";

    auto t1 = high_resolution_clock::now();
    int fps = 0, fps_count = 0;
    std::cerr << "before sync_once judge\n";
    if (!sync_once(camera, imu, sync_period_ms)) {
        goto stop;
    }
    std::cerr << "after sync_once judge, not goto\n";

    while (imu_is_ok && auto_exposure_is_ok) {
        cv::Mat img;
        double timestamp;
        if (!camera.read(img, timestamp)) {
            fmt::print(fmt::fg(fmt::color::red), "Camera read error!\n");
            break;
        }
        timestamp /= 1e3;
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
//        cv::rotate(img, img, cv::ROTATE_180);
//        if (seq_q.empty()) {
//            fmt::print(fmt::fg(fmt::color::red), "Imu read error!\n");
//            break;
//        }

// #ifndef NDEBUG
//         std::cout << "current imu data in waiting: " << seq_q.size() << std::endl;
//         std::cout.flush();
// #endif

        /* publish data */
        {
            std::unique_lock lock(seq_q_mtx);
            data_pub.push({img, seq_q.back(), timestamp});
        }

        /* show raw */
        if(webview_checkbox->checked){
            cv::Mat im2show = img.clone();
            fps_count++;
            auto t2 = high_resolution_clock::now();
            if (duration_cast<milliseconds>(t2 - t1).count() >= 1000) {
                fps = fps_count;
                fps_count = 0;
                t1 = t2;
            }
            cv::putText(im2show, fmt::format("fps={}", fps), {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0});
            webview_raw.push(im2show);
        }
    }

    stop:
	std::cerr << "start stop label\n";
        imu_require_stop = true;
        auto_exposure_require_stop = true;
        imu_capture_thread.join();
        auto_exposure_thread.join();
    std::cerr << "return\n";

    return false;
}


void background_sensors_io_auto_restart(const std::string &camera_name = "", const std::string &camera_cfg = "",
                                        const std::string &sensor_param_file = "",
                                        const std::string &imu_usb_hid = "", int sync_period_ms = 10) {
    if (debug) std::cout << "============background_sensors_io_auto_restart===========\n";
    std::thread([=]() {
        while (!sensors_io(camera_name, camera_cfg, sensor_param_file, imu_usb_hid, sync_period_ms)) {
            std::this_thread::sleep_for(500ms);
        }
    }).detach();
}

UMT_EXPORT_OBJMANAGER_ALIAS(AutoExposureParam, AutoExposureParam, c) {
    c.def_readwrite("min_brightness", &AutoExposureParam::min_brightness);
    c.def_readwrite("max_brightness", &AutoExposureParam::max_brightness);
    c.def_readwrite("step_exposure_us", &AutoExposureParam::step_exposure_us);
    c.def_readwrite("min_exposure_us", &AutoExposureParam::min_exposure_us);
    c.def_readwrite("max_exposure_us", &AutoExposureParam::max_exposure_us);
}

UMT_EXPORT_MESSAGE_ALIAS(SensorsData, SensorsData, c) {
    c.def_readwrite("im", &SensorsData::im, "image of current frame");
    c.def_readwrite("q", &SensorsData::q, "quaternion of current frame");
    c.def_readwrite("timestamp", &SensorsData::timestamp, "timestamp of current frame");
}

PYBIND11_EMBEDDED_MODULE(SensorsIO, m) {
    namespace py = pybind11;
    m.def("background_sensors_io_auto_restart", background_sensors_io_auto_restart,
          py::arg("camera_name") = "", py::arg("camera_cfg") = "",
          py::arg("sensor_param_file") = "",
          py::arg("imu_usb_hid") = "", py::arg("sync_period_ms") = 10);
}
