//
// Created by Harry-hhj on 2021/5/22.
//

#ifndef CVRM2021_PREDICTORADAPTIVEEKF_H
#define CVRM2021_PREDICTORADAPTIVEEKF_H

#include "AdaptiveEKF.hpp"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "../autoaim.hpp"
#include <robot.hpp>
#include <umt/umt.hpp>
#include <vector>
#include <cmath>
#include <ceres/ceres.h>
#include <opencv2/core/eigen.hpp>


struct Predict {
    /*
     * 此处定义匀速直线运动模型
     */
    template<class T>
    void operator()(const T x0[5], T x1[5]) {
        x1[0] = x0[0] + delta_t * x0[1];  //0.1
        x1[1] = x0[1];  //100
        x1[2] = x0[2] + delta_t * x0[3];  //0.1
        x1[3] = x0[3];  //100
        x1[4] = x0[4];  //0.01
    }

    double delta_t;
};

template<class T>
void xyz2pyd(T xyz[3], T pyd[3]) {
    /*
     * 工具函数：将 xyz 转化为 pitch、yaw、distance
     */
    pyd[0] = ceres::atan2(xyz[2], ceres::sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]));  // pitch
    pyd[1] = ceres::atan2(xyz[1], xyz[0]);  // yaw
    pyd[2] = ceres::sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]+xyz[2]*xyz[2]);  // distance
}

struct Measure {
    /*
     * 工具函数的类封装
     */
    template<class T>
    void operator()(const T x[5], T y[3]) {
        T x_[3] = {x[0], x[2], x[4]};
        xyz2pyd(x_, y);
    }
};


class PredictorAdaptiveEKF {
private:
    AdaptiveEKF<5, 3> ekf;  // 创建ekf

    double last_time;

    Eigen::Matrix3d R_CI;               // 陀螺仪坐标系到相机坐标系旋转矩阵EIGEN-Matrix
    Eigen::Matrix3d F;                  // 相机内参矩阵EIGEN-Matrix
    Eigen::Matrix<double, 1, 5> C;      // 相机畸变矩阵EIGEN-Matrix
    cv::Mat R_CI_MAT;                   // 陀螺仪坐标系到相机坐标系旋转矩阵CV-Mat
    cv::Mat F_MAT;                      // 相机内参矩阵CV-Mat
    cv::Mat C_MAT;                      // 相机畸变矩阵CV-Mat
    std::vector<bbox_t> last_boxes;     // 上一帧识别到的所有装甲板
    bbox_t last_sbbox;                  // 上一帧击打的装甲板
    Eigen::Matrix3d last_R_IW;          // 上一帧击陀螺仪数据
    Eigen::Vector3d last_m_pw;          // 上一帧击打的装甲板的世界坐标
    bool last_shoot = false;            // 判断上一次是否击打
    double last_yaw = 0, last_pitch = 0;

    bool distant = false, antitop = false, exist_hero = false;  // 远距离、反陀螺、英雄存在标志位
    bool last_is_one_armor = false, last_is_two_armors = false, right = true, anticlockwise = true;  // clockwise：逆时针

    int dead_buffer = 0;  // 灰色buffer

    struct Record {
        /*
         * 候选装甲板
         */
        bbox_t bbox;
        Eigen::Vector3d last_m_pw;
        AdaptiveEKF<5, 3> ekf;  // 创建ekf
        bool updated = false, init = false;
        double last_yaw = 0, last_pitch = 0;
        float yaw_angle = 0., pitch_angle = 0., yaw_speed = 0, pitch_speed = 0;
        int distance = 0.;
        bool distant = false;
        int age = 0;  // 装甲板存活时间

        Record(bbox_t &armor): bbox(armor), updated(false), init(true), last_yaw(0.), last_pitch(0.),
                               yaw_angle(0.), pitch_angle(0.), yaw_speed(0.), pitch_speed(0.),
                               distant(false), age(0) {}
    };
    std::vector<Record> antitop_candidates;

    bool debug = true;
    int DEAD_BUFFER = 20;

    // 相机坐标系内坐标--->世界坐标系内坐标
    inline Eigen::Vector3d pc_to_pw(const Eigen::Vector3d &pc, const Eigen::Matrix3d &R_IW) {
        auto R_WC = (R_CI * R_IW).transpose();
        return R_WC * pc;
    }

    // 世界坐标系内坐标--->相机坐标系内坐标
    inline Eigen::Vector3d pw_to_pc(const Eigen::Vector3d &pw, const Eigen::Matrix3d &R_IW) {
        auto R_CW = R_CI * R_IW;
        return R_CW * pw;
    }

    // 相机坐标系内坐标--->图像坐标系内像素坐标
    inline Eigen::Vector3d pc_to_pu(const Eigen::Vector3d &pc) {
        return F * pc / pc(2, 0);
    }

    // 将世界坐标系内一点，投影到图像中，并绘制该点
    inline void re_project_point(cv::Mat &image, const Eigen::Vector3d &pw,
                                 const Eigen::Matrix3d &R_IW, const cv::Scalar &color) {
        Eigen::Vector3d pc = pw_to_pc(pw, R_IW);
        Eigen::Vector3d pu = pc_to_pu(pc);
        cv::circle(image, {int(pu(0, 0)), int(pu(1, 0))}, 3, color, 2);
    }

    // 计算任意四边形的中心 -- 简单做法
    inline cv::Point2f points_center(cv::Point2f pts[4]) {
        cv::Point2f center;
        center.x = (pts[0].x + pts[1].x + pts[2].x + pts[3].x) / 4;
        center.y = (pts[0].y + pts[1].y + pts[2].y + pts[3].y) / 4;
        return center;
    }

    // 判断交并比
    float bbOverlap(const cv::Rect2f& box1,const cv::Rect2f& box2);

    // 四点转化为矩形
    inline cv::Rect2f get_ROI(bbox_t &armor, float coefficient = 1.0f) {
        auto center = points_center(armor.pts);
        auto w = std::max({armor.pts[0].x, armor.pts[1].x, armor.pts[2].x, armor.pts[3].x}) -
                std::min({armor.pts[0].x, armor.pts[1].x, armor.pts[2].x, armor.pts[3].x});
        auto h = std::max({armor.pts[0].y, armor.pts[1].y, armor.pts[2].y, armor.pts[3].y}) -
                std::min({armor.pts[0].y, armor.pts[1].y, armor.pts[2].y, armor.pts[3].y});
        return cv::Rect2f(center.x-w/2, center.y-h/2, w*coefficient, h*coefficient);
    }

    // 海伦公式获得四边形面积
    double get_quadrangle_size(const bbox_t &bx);

    // 根据距离判断是否是同一块装甲板
    inline bool is_same_armor_by_distance(const Eigen::Vector3d old_m_pw, const bbox_t &new_armor, const Eigen::Matrix3d &R_IW, const double distance_threshold = 0.15) {
        Eigen::Vector3d new_m_pc = pnp_get_pc(new_armor.pts, new_armor.tag_id);
        Eigen::Vector3d new_m_pw = pc_to_pw(new_m_pc, R_IW);
        Eigen::Vector3d m_pw_delta = new_m_pw - old_m_pw;
        double distance = m_pw_delta.norm();
        if (distance < distance_threshold) return true;
        else return false;
    }

    // 根据第五层防抖匹配两个装甲板
    void match_armors(bbox_t &armor, bool &selected, const std::vector<bbox_t> &detections, const Eigen::Matrix3d &R_IW, const bool right);

    // pnp解算:获取相机坐标系内装甲板坐标
    Eigen::Vector3d pnp_get_pc(const cv::Point2f p[4], int armor_number);

public:
    inline void load_param(bool update_all = true){
        /*
         * 通过文件读入，实现实时调餐
         */
        cv::FileStorage fin(PROJECT_DIR"/asset/autoaim-param.yml", cv::FileStorage::READ);

        if (!antitop) {
            // 设置对角线的值
            // 预测过程协方差
            fin["Q00"] >> ekf.Q(0, 0);
            fin["Q11"] >> ekf.Q(1, 1);
            fin["Q22"] >> ekf.Q(2, 2);
            fin["Q33"] >> ekf.Q(3, 3);
            fin["Q44"] >> ekf.Q(4, 4);
            // 观测过程协方差
            fin["R00"] >> ekf.R(0, 0);
            fin["R11"] >> ekf.R(1, 1);
            fin["R22"] >> ekf.R(2, 2);

            if (update_all) {
                fin["DEBUG"] >> debug;  // 对反陀螺几乎没什么用
                fin["DEAD_BUFFER"] >> DEAD_BUFFER;
                for (auto &a: antitop_candidates) {
                    // 预测过程协方差
                    fin["Q00"] >> a.ekf.Q(0, 0);
                    fin["Q11"] >> a.ekf.Q(1, 1);
                    fin["Q22"] >> a.ekf.Q(2, 2);
                    fin["Q33"] >> a.ekf.Q(3, 3);
                    fin["Q44"] >> a.ekf.Q(4, 4);
                    // 观测过程协方差
                    fin["R00"] >> a.ekf.R(0, 0);
                    fin["R11"] >> a.ekf.R(1, 1);
                    fin["R22"] >> a.ekf.R(2, 2);
                }
            }
        } else {
            // 设置对角线的值
            // 预测过程协方差
            fin["Q00_AC"] >> ekf.Q(0, 0);
            fin["Q11_AC"] >> ekf.Q(1, 1);
            fin["Q22_AC"] >> ekf.Q(2, 2);
            fin["Q33_AC"] >> ekf.Q(3, 3);
            fin["Q44_AC"] >> ekf.Q(4, 4);
            // 观测过程协方差
            fin["R00_AC"] >> ekf.R(0, 0);
            fin["R11_AC"] >> ekf.R(1, 1);
            fin["R22_AC"] >> ekf.R(2, 2);

            if (update_all) {
                fin["DEBUG"] >> debug;  // 对反陀螺几乎没什么用
                fin["DEAD_BUFFER"] >> DEAD_BUFFER;
                for (auto &a: antitop_candidates) {
                    // 预测过程协方差
                    fin["Q00_AC"] >> a.ekf.Q(0, 0);
                    fin["Q11_AC"] >> a.ekf.Q(1, 1);
                    fin["Q22_AC"] >> a.ekf.Q(2, 2);
                    fin["Q33_AC"] >> a.ekf.Q(3, 3);
                    fin["Q44_AC"] >> a.ekf.Q(4, 4);
                    // 观测过程协方差
                    fin["R00_AC"] >> a.ekf.R(0, 0);
                    fin["R11_AC"] >> a.ekf.R(1, 1);
                    fin["R22_AC"] >> a.ekf.R(2, 2);
                }
            }
        }
    }

    explicit PredictorAdaptiveEKF( ) {
        cv::FileStorage fin(PROJECT_DIR"/asset/camera-param.yml", cv::FileStorage::READ);
        fin["Tcb"] >> R_CI_MAT;
        fin["K"] >> F_MAT;
        fin["D"] >> C_MAT;
        cv::cv2eigen(R_CI_MAT, R_CI);
        cv::cv2eigen(F_MAT, F);
        cv::cv2eigen(C_MAT, C);

        load_param();

        std::cout << "Finish create a new EKF." << std::endl;
    }

    bool predict(Detection_pack &, RobotCmd &, cv::Mat &, bool);

    inline void clear() {
        last_boxes.clear();
        last_shoot = false;
        antitop = false;
        dead_buffer = 0;
        antitop_candidates.clear();
    }

    ~PredictorAdaptiveEKF() = default;
};


#endif //CVRM2021_PREDICTORADAPTIVEEKF_H
