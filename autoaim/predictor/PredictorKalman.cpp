//
// Created by hhj on 2021/5/5.
//

#include "PredictorKalman.h"
#include "../detector/TRTModule.hpp"  // TODO：don't need?
#include <opencv2/core/eigen.hpp>
#include <fmt/format.h>
#include <robot.hpp>
#include <cmath>

constexpr double shoot_delay = 0.09;   // 射击延迟: 90ms

PredictorKalman::PredictorKalman() {
    cv::FileStorage fin(PROJECT_DIR"/asset/camera-param.yml", cv::FileStorage::READ);
    fin["Tcb"] >> R_CI_MAT;
    fin["K"] >> F_MAT;
    fin["D"] >> C_MAT;
    cv::cv2eigen(R_CI_MAT, R_CI);
    cv::cv2eigen(F_MAT, F);
    cv::cv2eigen(C_MAT, C);

    _Kalman::Matrix_xxd A = _Kalman::Matrix_xxd::Identity();
    _Kalman::Matrix_zxd H;
    H(0, 0) = 1;
    _Kalman::Matrix_xxd R;
    R(0, 0) = 0.01;
    for (int i = 1; i < S; i++) {
        R(i, i) = 100;
    }
    _Kalman::Matrix_zzd Q{4};
    _Kalman::Matrix_x1d init{0, 0};
    kalman = _Kalman(A, H, R, Q, init, 0);
}

// 计算任意四边形的中心
cv::Point2f points_center(cv::Point2f pts[4]){
    for (int i = 0; i < 4; ++i) {
        for (int j = i+1; j < 4; ++j) {
            if (pts[i] == pts[j]) {
                std::cout << "[Error] Unable to calculate center point." << std::endl;
                return cv::Point2f{0, 0};
            }
        }
    }
    cv::Point2f center(0, 0);
    if (pts[0].x == pts[2].x && pts[1].x == pts[3].x) {
        std::cout << "[Error] Unable to calculate center point." << std::endl;
    }
    else if (pts[0].x == pts[2].x && pts[1].x != pts[3].x) {
        center.x = pts[0].x;
        center.y = (pts[3].y-pts[1].y)/(pts[3].x-pts[1].x)*(pts[0].x-pts[3].x)+pts[3].y;
    }
    else if (pts[1].x == pts[3].x && pts[0].x != pts[2].x) {
        center.x = pts[1].x;
        center.y = (pts[2].y-pts[0].y)/(pts[2].x-pts[0].x)*(pts[1].x-pts[0].x)+pts[0].y;
    }
    else {
        center.x = (((pts[3].y-pts[1].y)/(pts[3].x-pts[1].x)*pts[3].x - pts[3].y + \
                    pts[0].y - (pts[2].y-pts[0].y)/(pts[2].x-pts[0].x)*pts[0].x)) / \
                    ((pts[3].y-pts[1].y)/(pts[3].x-pts[1].x)-(pts[2].y-pts[0].y)/(pts[2].x-pts[0].x));
        center.y = (pts[2].y-pts[0].y)/(pts[2].x-pts[0].x)*(center.x-pts[0].x)+pts[0].y;
    }

    return center;
}

bool PredictorKalman::predict(Detection_pack &data, RobotCmd &send, cv::Mat &im2show) {
//    std::cout << "In Kalman-predict" << std::endl;

    auto &[detections, img, q_, t] = data;
    im2show = img;

    Eigen::Quaternionf q_raw(q_[0], q_[1], q_[2], q_[3]);
    Eigen::Quaternionf q(q_raw.matrix().transpose());
//	std::cout<<qt[0]<<qt[1]<<qt[2]<<qt[3]<<std::endl;
    Eigen::Matrix3d R_IW = q.matrix().cast<double>();
    auto robot_status = umt::ObjManager<RobotStatus>::find_or_create("robot_status");

    if (detections.empty()) {
        roi.clear();
        return false;
    }

    std::vector<bbox_t> new_detections;
    // 过滤出敌方颜色的装甲板
    for (auto &d: detections) {
         if (int(robot_status->enemy_color) == d.color_id && d.confidence >=0.5f && d.tag_id != 0 && d.tag_id != 2 && d.tag_id != 6) new_detections.push_back(d);
    }
    if (new_detections.empty()) {
        roi.clear();
        return false;
    }

    // 按照装甲板的大小排序，运用海伦公式
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

    auto &armor = new_detections.front();
    if (roi.ROI_selected) {
        bool flag = false;  // 是否找到高优先级装甲板
        for (auto &d: new_detections) {
            auto center = points_center(d.pts);
            if (d.confidence >= 0.2f && (center.inside(roi.ROI_bbox))) {
                armor = d;
                flag = true;
                break;
            }
        }
        if (!flag) {
            for (auto &d: new_detections) {
                if (d.tag_id == roi.last_class) armor = d;
            }
        }
    }

    auto center = points_center(armor.pts);
    if (center.x != 0 || center.y != 0) {
        roi.ROI_selected = true;
        roi.last_class = armor.tag_id;
        roi.ROI_bbox = cv::Rect2f(center.x, center.y,
                                  (armor.pts[2].x + armor.pts[3].x - armor.pts[0].x - armor.pts[1].x) * 0.8, \
            (armor.pts[1].y + armor.pts[2].y - armor.pts[0].y - armor.pts[3].y) * 0.8);
    }
    else {
        roi.ROI_selected = false;
        std::cout << "[Warning] ROI is illegal!" << std::endl;
    }

    Eigen::Vector3d m_pc = pnp_get_pc(armor.pts, armor.tag_id);             // point camera: 目标在相机坐标系下的坐标
    Eigen::Vector3d m_pw = pc_to_pw(m_pc, R_IW);          // point world: 目标在世界坐标系下的坐标。（世界坐标系:陀螺仪的全局世界坐标系）

    static double last_yaw = 0, last_speed = 0;
    double mc_yaw = std::atan2(m_pc(1,0), m_pc(0,0));
    double m_yaw = std::atan2(m_pw(1, 0), m_pw(0, 0));   // yaw的测量值，单位弧度
    std::cout << "m_yaw=" << m_yaw * 180. / M_PI << std::endl;
    if(std::fabs(last_yaw - m_yaw) > 5. / 180. * M_PI){
        kalman.reset(m_yaw, t);
        last_yaw = m_yaw;
        std::cout << "reset" << std::endl;
        return false;
    }                            

    last_yaw = m_yaw;
    Eigen::Matrix<double, 1, 1> z_k{m_yaw};
    _Kalman::Matrix_x1d state = kalman.update(z_k, t);                        // 更新卡尔曼滤波
    last_speed = state(1, 0);
    double c_yaw = state(0, 0);                                   // current yaw: yaw的滤波值，单位弧度
    double c_speed = state(1, 0) * m_pw.norm();                      // current speed: 角速度转线速度，单位m/s
//    std::cout << "c_yaw: " << c_yaw << "c_speed:" << c_speed;
//    std::cout << "t: " << t << " state(1, 0): " << state(1, 0) << std::endl;

//    double compensate_speed_signal = c_speed > 0 ? 1 : -1;
//    double compensate_speed = compensate_speed_signal * c_speed * 0.1;
//    c_speed += compensate_speed;                                   // speed compensate

//    std::cout << "state: " << state << std::endl;
    double predict_time = m_pw.norm() / robot_status->robot_speed_mps + shoot_delay;           // 预测时间=发射延迟+飞行时间（单位:s）
    double p_yaw = c_yaw + atan2(predict_time * c_speed, m_pw.norm());     // predict yaw: yaw的预测值，直线位移转为角度，单位弧度

    double length = sqrt(m_pw(0, 0) * m_pw(0, 0) + m_pw(1, 0) * m_pw(1, 0));
    Eigen::Vector3d c_pw{length * cos(c_yaw), length * sin(c_yaw), m_pw(2, 0)};//反解位置(世界坐标系)
    Eigen::Vector3d p_pw{length * cos(p_yaw), length * sin(p_yaw), m_pw(2, 0)};

    double distance = p_pw.norm();                          // 目标距离（单位:m）
    double distance_xy = p_pw.topRows<2>().norm();
    double p_pitch = std::atan2(p_pw(2, 0), distance_xy);
    //std::cout << fmt::format("distance: {}", distance) << std::endl;
//    std::cout << state << std::endl;

    // 计算抛物线
    // 先解二次方程
//    std::cout << "p_pitch: " << p_pitch <<std::endl;
    double a = 9.8 * 9.8 * 0.25;
    double b = -robot_status->robot_speed_mps * robot_status->robot_speed_mps - distance * 9.8 * cos(M_PI_2 + p_pitch);
    double c = distance * distance;
    // 带入求根公式，解出t^2
    double t_2 = (- sqrt(b * b - 4 * a * c) - b) / (2 * a);
//    std::cout << fmt::format("a:{}, b:{}, c:{}", a, b, c) << std::endl;
//    std::cout << "t2:" << t_2 << std::endl;
    double fly_time = sqrt(t_2);                                       // 子弹飞行时间（单位:s）
    // 解出抬枪高度，即子弹下坠高度
    double height = 0.5 * 9.8 * t_2;

    //std::cout << m_pw << std::endl;
    float bs = robot_status->robot_speed_mps;
    //std::cout << fmt::format("bullet_speed:{}, distance: {}, height: {}, p_pitch:{}",
    //                         bs, distance, height, p_pitch / M_PI * 180) << std::endl;
	Eigen::Vector3d s_pw{p_pw(0, 0), p_pw(1, 0), p_pw(2, 0) + height}; // 抬枪后预测点


    /// re-project test
    /// 把世界坐标系中的点，重投影到图像中
	im2show = img.clone();
	re_project_point(im2show, c_pw, R_IW, {0, 255, 0});
	re_project_point(im2show, p_pw, R_IW, {255, 0, 0});
	// here is special change for no.4
	re_project_point(im2show, s_pw, R_IW, {0, 0, 255});
    for (int i = 0; i < 4; ++i)
        cv::circle(im2show, armor.pts[i], 3, {255, 0, 0});
    cv::circle(im2show, {im2show.cols / 2, im2show.rows / 2}, 3, {0, 255 ,0});

    Eigen::Vector3d s_pc = pw_to_pc(s_pw, R_IW);
    double s_yaw = atan(s_pc(0, 0) / s_pc(2, 0)) / M_PI * 180.;
    double s_pitch = atan(s_pc(1, 0) / s_pc(2, 0)) / M_PI * 180.;
    // 绘制角度波形图

	send.yaw_angle = (float) s_yaw;
	send.pitch_angle = (float) s_pitch;
	send.yaw_speed = c_speed;
    std::cout << "s_pitch: " << s_pitch << std::endl;
	
	if (std::abs(send.yaw_angle) < 90. && std::abs(send.pitch_angle) < 90.)
		send.shoot_mode = static_cast<uint8_t>(ShootMode::COMMON);
	else
		send.shoot_mode = static_cast<uint8_t>(ShootMode::CRUISE);
//	std::cout << "yaw: " << send.yaw_angle << " pitch: " << send.pitch_angle << " yaw-speed: " << send.yaw_speed << "pitch-speed" << send.pitch_speed << std::endl;
 
    return true;
}

Eigen::Vector3d PredictorKalman::pnp_get_pc(const cv::Point2f p[4], int armor_number) {
    static const std::vector<cv::Point3d> pw_small = {  // 单位：m
            {-0.066, 0.027,  0.},
            {-0.066, -0.027, 0.},
            {0.066,  -0.027, 0.},
            {0.066,  0.027,  0.}
    };
    static const std::vector<cv::Point3d> pw_big = {    // 单位：m
            {-0.115, 0.029,  0.},
            {-0.115, -0.029, 0.},
            {0.115,  -0.029, 0.},
            {0.115,  0.029,  0.}
    };
    std::vector<cv::Point2d> pu(p, p + 4);
    cv::Mat rvec, tvec;

    if (armor_number == 0 || armor_number == 1)
        cv::solvePnP(pw_big, pu, F_MAT, C_MAT, rvec, tvec);
    else
        cv::solvePnP(pw_small, pu, F_MAT, C_MAT, rvec, tvec);

    Eigen::Vector3d pc;
    cv::cv2eigen(tvec, pc);
    return pc;
}

//std::unique_ptr<PredictorBase> make_predictor_kalman() {
//    return std::make_unique<PredictorKalman>();
//}
