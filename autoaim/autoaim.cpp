//
// Created by xinyang on 2021/3/7.
//

// Modified by Harry-hhj on 2021/05/04

#include "autoaim.hpp"
#include <umt/umt.hpp>
#include <sensors.hpp>
#include <robot.hpp>
#include <thread>
#include <chrono>
#include <common.hpp>
#include <fmt/format.h>
#include <fmt/color.h>
#include <opencv2/opencv.hpp>
#include <pybind11/numpy.h>
#include <ctime>
#include <string>
#include <cmath>
#include <predictor/PredictorKalman.h>
#include <predictor/PredictorAdaptiveEKF.h>

using namespace std::chrono;
namespace py = pybind11;

static bool debug = true;

[[noreturn]] void detection_run(const std::string &onnx_file) {
    /*
     * 识别器
     */

    if (debug) std::cout << "============ detection_run ===========" << std::endl;
    TRTModule model(onnx_file);

    auto webview_checkbox = umt::ObjManager<CheckBox>::find_or_create("show detections");

    umt::Subscriber<SensorsData> sensor_sub("sensors_data");
    umt::Publisher<cv::Mat> webview_detections("detections");

    umt::Publisher<Detection_pack> detections_pub("detections_pack");

    int fps = 0, fps_count = 0;
    auto t1 = system_clock::now();
    int cnt_useless = -1;

    const cv::Scalar colors[3] = {{255, 0, 0}, {0, 0, 255}, {0, 255, 0}};

    while (true) {
        try {
            const auto& [img, q, timestamp] = sensor_sub.pop();
            auto detections = model(img);

            /* publish detection results */
            if (!detections.empty()) {
                detections_pub.push(Detection_pack{detections, img, q, timestamp});  // 打包数据
            } else {
                if (++cnt_useless == 50) {  // 避免输出太多
                    fmt::print(fmt::fg(fmt::color::blue), "No enemy detected!");
                    std::cout << std::endl;
                    cnt_useless = -1;
                }
                webview_detections.push(img);
                continue;
            }

            /* show detections */
            if(webview_checkbox->checked){
                cv::Mat im2show = img.clone();
                for (const auto &b: detections) {
                    cv::line(im2show, b.pts[0], b.pts[1], colors[2], 2);
                    cv::line(im2show, b.pts[1], b.pts[2], colors[2], 2);
                    cv::line(im2show, b.pts[2], b.pts[3], colors[2], 2);
                    cv::line(im2show, b.pts[3], b.pts[0], colors[2], 2);
                    cv::putText(im2show, std::to_string(b.tag_id), b.pts[0], cv::FONT_HERSHEY_SIMPLEX, 1, colors[b.color_id]);
                }
                fps_count++;
                auto t2 = system_clock::now();
                if (duration_cast<milliseconds>(t2 - t1).count() >= 1000) {
                    fps = fps_count;
                    fps_count = 0;
                    t1 = t2;
                }
                cv::putText(im2show, fmt::format("fps={}", fps), {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, colors[2]);
                webview_detections.push(im2show);
            }
        } catch (umt::MessageError &e) {
            fmt::print(fmt::fg(fmt::color::orange), "[WARNING] 'sensors_data' {}\n", e.what());
            std::this_thread::sleep_for(500ms);
        }
    }
}

void background_detection_run(const std::string &onnx_file) {
    if (debug) std::cout << "============ background_detection_run ===========" << std::endl;

    std::thread([=]() {
        detection_run(onnx_file);
    }).detach();
}

void none_predict_run() {
    /*
     * 仅具有跟随的效果，没有抬枪补偿，用来查看电控是否正确
     */
    if (debug) std::cout << "============ none_predict_run ===========" << std::endl;
    umt::Subscriber<Detection_pack> detections_sub("detections_pack");
    auto sensor_param = umt::ObjManager<SensorParam>::find_or_create("sensor_param");
    const cv::Mat& K = sensor_param->K;
    const cv::Mat& D = sensor_param->D;
    const cv::Mat& Tcb = sensor_param->Tcb;
    const double fx = K.at<double>(0, 0);
    const double fy = K.at<double>(1, 1);
    const double cx = K.at<double>(0, 2);
    const double cy = K.at<double>(1, 2);
    umt::Publisher<RobotCmd> robot_cmd_pub("robot_cmd");

    while (true) {
        try {
            const auto &detections_pack = detections_sub.pop();
            auto detections = detections_pack.detection;
            std::sort(detections.begin(), detections.end(), [](const auto &bx, const auto &by) {
                auto bx_a = sqrt(pow(bx.pts[0].x-bx.pts[1].x, 2)+pow(bx.pts[0].y-bx.pts[1].y, 2));
                auto bx_b = sqrt(pow(bx.pts[1].x-bx.pts[2].x, 2)+pow(bx.pts[1].y-bx.pts[2].y, 2));
                auto bx_c = sqrt(pow(bx.pts[2].x-bx.pts[3].x, 2)+pow(bx.pts[2].y-bx.pts[3].y, 2));
                auto bx_d = sqrt(pow(bx.pts[3].x-bx.pts[0].x, 2)+pow(bx.pts[3].y-bx.pts[0].y, 2));
                auto bx_z = (bx_a+bx_b+bx_c+bx_d)/2;
                auto bx_size = 2*sqrt((bx_z-bx_a)*(bx_z-bx_b)*(bx_z-bx_c)*(bx_z-bx_d));
                auto by_a = sqrt(pow(bx.pts[0].x-bx.pts[1].x, 2)+pow(bx.pts[0].y-bx.pts[1].y, 2));
                auto by_b = sqrt(pow(bx.pts[1].x-bx.pts[2].x, 2)+pow(bx.pts[1].y-bx.pts[2].y, 2));
                auto by_c = sqrt(pow(bx.pts[2].x-bx.pts[3].x, 2)+pow(bx.pts[2].y-bx.pts[3].y, 2));
                auto by_d = sqrt(pow(bx.pts[3].x-bx.pts[0].x, 2)+pow(bx.pts[3].y-bx.pts[0].y, 2));
                auto by_z = (by_a+by_b+by_c+by_d)/2;
                auto by_size = 2*sqrt((by_z-by_a)*(by_z-by_b)*(by_z-by_c)*(by_z-by_d));
                return bx_size > by_size;
            });

            auto &target = detections.front();
            float x = (target.pts[0].x + target.pts[1].x + target.pts[2].x + target.pts[3].x) / 4.f;
            float y = (target.pts[0].y + target.pts[1].y + target.pts[2].y + target.pts[3].y) / 4.f;
            RobotCmd robot_cmd;
            robot_cmd.pitch_angle = (float)atan2(y - cy, fy);
            robot_cmd.yaw_angle = (float)atan2(x - cx, fx);
            robot_cmd.pitch_speed = 0;
            robot_cmd.yaw_speed = 0;
            robot_cmd.distance = 0.;
            robot_cmd.shoot_mode = static_cast<uint8_t>(ShootMode::COMMON);
            robot_cmd_pub.push(robot_cmd);
        } catch (umt::MessageError &e) {
            fmt::print(fmt::fg(fmt::color::orange), "[WARNING] 'yolo_detections' {}\n", e.what());
            std::this_thread::sleep_for(500ms);
        }
    }
}

void background_none_predict_run() {
    if (debug) std::cout << "============ background_none_predict_run ===========" << std::endl;

    std::thread([]() {
        none_predict_run();
    }).detach();
}

void predict_run() {
    /*
     * 使用传统 Kalman 预测
     */
    if (debug) std::cout << "============ predict_run ===========" << std::endl;

    auto webview_checkbox = umt::ObjManager<CheckBox>::find_or_create("show prediction");
    umt::Publisher<cv::Mat> webview_predictions("prediction");

    umt::Subscriber<Detection_pack> detections_sub("detections_pack");
    umt::Publisher<RobotCmd> robot_cmd_pub("robot_cmd");
    PredictorKalman predictor;
    umt::MessageError_Timeout timeout_error;
    auto robot_status_short = umt::ObjManager<ShortRobotStatus>::find_or_create("robot_status_short");
    auto robot_status_long = umt::ObjManager<LongRobotStatus>::find_or_create("robot_status_long");

    int fps = 0, fps_count = 0;
    auto t1 = system_clock::now();

    while (true) {
        try {
            if (robot_status_long->program_mode == ProgramMode::AUTO_AIM){
                auto detections = detections_sub.pop_for(50);
                RobotCmd robot_cmd;
                cv::Mat im2show;
                bool ok = predictor.predict(detections, robot_cmd, im2show);

                /* show predictions */
                if (webview_checkbox->checked) {
                    fps_count++;
                    auto t2 = system_clock::now();
                    if (duration_cast<milliseconds>(t2 - t1).count() >= 1000) {
                        fps = fps_count;
                        fps_count = 0;
                        t1 = t2;
                    }
                    cv::putText(im2show, fmt::format("fps={}", fps), {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, {0,0,255});
                    webview_predictions.push(im2show);
                }

                if(!ok) {
                    throw timeout_error;
                }

                robot_cmd_pub.push(robot_cmd);
            }
            else {
                std::cout << "[Error] Program_mode:" << (unsigned int)robot_status_long->program_mode << " not implemented." << std::endl;
                std::this_thread::sleep_for(500ms);
            }
        } catch (umt::MessageError_Timeout &e) {
            RobotCmd robot_cmd;  // TODO
            robot_cmd.shoot_mode = static_cast<uint8_t>(ShootMode::CRUISE);
            robot_cmd_pub.push(robot_cmd);
        } catch (umt::MessageError &e) {
            fmt::print(fmt::fg(fmt::color::orange), "[WARNING] 'yolo_detections' {}\n", e.what());
            std::this_thread::sleep_for(500ms);
        }
    }
}

void background_predict_run() {
    if (debug) std::cout << "============ background_predict_run ===========" << std::endl;

    std::thread([]() {
        predict_run();
        std::cout<<"In background-predict-run" << std::endl;
    }).detach();
}

void predict_EKF_run() {
    /*
     * 使用 EKF 预测
     */
    if (debug) std::cout << "============ predict_EKF_run ===========" << std::endl;

    auto webview_checkbox = umt::ObjManager<CheckBox>::find_or_create("show predictionEKF");
    umt::Publisher<cv::Mat> webview_predictions("predictionEKF");

    umt::Subscriber<Detection_pack> detections_sub("detections_pack");
    umt::Publisher<RobotCmd> robot_cmd_pub("robot_cmd");
    PredictorAdaptiveEKF predictor;
    umt::MessageError_Timeout timeout_error;
    auto robot_status_short = umt::ObjManager<ShortRobotStatus>::find_or_create("robot_status_short");
    auto robot_status_long = umt::ObjManager<LongRobotStatus>::find_or_create("robot_status_long");

    int fps = 0, fps_count = 0;
    auto t1 = system_clock::now();

    bool last_mode_is_autoaim = true;

    while (true) {
        try {
            if (robot_status_long->program_mode == ProgramMode::AUTO_AIM){
                auto detections = detections_sub.pop_for(50);
                RobotCmd robot_cmd;
                cv::Mat im2show;

                if (!last_mode_is_autoaim) predictor.clear();
                last_mode_is_autoaim = true;

                bool ok = predictor.predict(detections, robot_cmd, im2show, webview_checkbox->checked);

                /* show predictions */
                if (webview_checkbox->checked) {
                    fps_count++;
                    auto t2 = system_clock::now();
                    if (duration_cast<milliseconds>(t2 - t1).count() >= 1000) {
                        fps = fps_count;
                        fps_count = 0;
                        t1 = t2;
                    }
                    cv::putText(im2show, fmt::format("fps={}", fps), {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, {0,0,255});
                    webview_predictions.push(im2show);
                }

                if(!ok) {
                    throw timeout_error;
                }

                robot_cmd_pub.push(robot_cmd);
            }
            else if (robot_status_long->program_mode == ProgramMode::SMALL_ENERGY || robot_status_long->program_mode == ProgramMode::BIG_ENERGY) {
                std::this_thread::sleep_for(1000ms);
            }
            else {
                std::cout << "[Error] Program_mode:" << (unsigned int) robot_status_long->program_mode
                          << " not implemented." << std::endl;
                std::this_thread::sleep_for(500ms);
            }
        } catch (umt::MessageError_Timeout &e) {
            RobotCmd robot_cmd;  // TODO
            robot_cmd.shoot_mode = static_cast<uint8_t>(ShootMode::CRUISE);
            robot_cmd_pub.push(robot_cmd);
        } catch (umt::MessageError &e) {
            fmt::print(fmt::fg(fmt::color::orange), "[WARNING] 'yolo_detections' {}\n", e.what());
            std::this_thread::sleep_for(500ms);
        }
    }
}

void background_predict_EKF_run() {
    if (debug) std::cout << "============ background_predict_EKF_run ===========" << std::endl;

    std::thread([]() {
        predict_EKF_run();
    }).detach();
}

PYBIND11_EMBEDDED_MODULE(AutoAim, m) {
    namespace py = pybind11;
    m.def("background_detection_run", background_detection_run, py::arg("onnx_file"));
    m.def("background_none_predict_run", background_none_predict_run);
    m.def("background_predict_run", background_predict_run);
    m.def("background_predict_EKF_run", background_predict_EKF_run);
}
