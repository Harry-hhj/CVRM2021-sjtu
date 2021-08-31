//
// Created by Harry-hhj on 2021/5/22.
//

#include "PredictorAdaptiveEKF.h"


////////////////////////// 调参区 start //////////////////////////
/// 射击延迟
// 旧哨兵下云台：0.105
// 新哨兵下云台：0.090
// 新哨兵上云台：0.110
constexpr double shoot_delay = 0.110;  // 上下云台的射击延迟是不一样的

/// 选择高度限制
constexpr double height_thres = 0.;
/// 识别双阈值
constexpr float high_thres = 0.6f;
constexpr float low_thres = 0.2f;

/// dead_buffer_max_size
int dead_buffer_max_size = 20;

/// 最重要装甲板
constexpr int important_id = 1;

/// 斩杀线
constexpr uint8_t killer_point = 80;

/// 远距离弹量控制
constexpr double distant_threshold = 6.;

/// 反陀螺模式下速度限制幅度
constexpr double antitop_x_v_proportion = 0.;
constexpr double antitop_y_v_proportion = 0.;

/// 反陀螺速度继承
constexpr float ac_x_v_coefficient = 0.5f;
constexpr float ac_y_v_coefficient = 0.5f;

/// 切换装甲板大小限制
/// 预测
constexpr float switch_armor_size_proportion = 1.1;

/// 初始化候选ekf大小阈值比例
constexpr double ac_init_min_age = 1;  // +1

/// 反陀螺频率统计记忆窗口
constexpr double beta = 0.25;  // 记忆窗口大小 T = 1/beta

/// DEBUG
bool DEBUG = true;
////////////////////////// 调参区 end //////////////////////////


struct LastTimeHelper {
    LastTimeHelper(double current_time, double &ref_time) : c_time(current_time), r_time(ref_time) {};

    ~LastTimeHelper() { r_time = c_time; }

    double c_time;
    double &r_time;
};

bool PredictorAdaptiveEKF::predict(Detection_pack &data, RobotCmd &send, cv::Mat &im2show, bool is_show) {
    /*
     * 根据 /// 注释可以快速理清整个逻辑
     */
    std::cout << "===============predict=================" << std::endl;
    /// 获取数据
    auto &[detections, img, q_, t] = data;
    LastTimeHelper helper(t, last_time);  // 自动更新上次时间
    im2show = img.clone();
    dead_buffer_max_size = DEAD_BUFFER;

    antitop = false;  // TODO: 相关代码已删除，可以接收电控指令切换

    distant = false;
    exist_hero = false;

    Eigen::Quaternionf q_raw(q_[0], q_[1], q_[2], q_[3]);
    Eigen::Quaternionf q(q_raw.matrix().transpose());
    if (DEBUG) std::cout << "q = " << q_[0] << '\t' << q_[1] << '\t' << q_[2] << '\t' << q_[3] << std::endl;
    Eigen::Matrix3d R_IW = q.matrix().cast<double>();
    last_R_IW = R_IW;  // 生成旋转矩阵
    auto robot_status_short = umt::ObjManager<ShortRobotStatus>::find_or_create("robot_status_short");
    auto robot_status_long = umt::ObjManager<LongRobotStatus>::find_or_create("robot_status_long");

    double delta_t = t - last_time;  // 计算距离上次预测的时间

    bool selected = false;
    bbox_t armor;

    /// 识别器未检测到车辆，清空部分历史状态量，返回
    if (detections.empty()) {
        clear();
        return false;
    }

    /// 分别对应：本次是否击打上次的装甲板、本次是否击打另一块同号装甲板、本次是否切换了装甲板、本次是否需要重置预测器
    bool same_armor = false, same_id = false, switch_armor = false, need_init = false;
    /// 分别对应：本次没有同 id 装甲板、本次存在一块同 id 装甲板
    bool this_is_one_armor = false, this_is_two_armors = false;

    /// 过滤出敌方颜色的装甲板 && 判断是否有英雄出现
    std::vector<bbox_t> new_detections;  // new_detection: vector 是经过过滤后所有可能考虑的装甲板
    for (auto &d: detections) {
        if (int(robot_status_short->enemy_color) == d.color_id && d.tag_id != 0 && d.tag_id != 6 && (d.tag_id != 2 || (d.tag_id == 2 && ((last_shoot && last_sbbox.tag_id != 2) || (robot_status_long->enemy.at(2)*10 <= killer_point && robot_status_long->enemy.at(2)*10 > 0))))) {  // 不能随意修改，否则会数组越界0-5
            /* 放行正确颜色的装甲板 */
            if (last_shoot && d.tag_id == last_sbbox.tag_id) dead_buffer = 0;  // 如果存在上一次击打装甲板同 id 且有颜色的，表示车辆存活，清空灰色缓冲

            Eigen::Vector3d m_pc = pnp_get_pc(d.pts, d.tag_id); // point camera: 目标在相机坐标系下的坐标
            Eigen::Vector3d m_pw = pc_to_pw(m_pc, R_IW);        // point world: 目标在世界坐标系下的坐标。（世界坐标系:陀螺仪的全局世界坐标系）
            if (m_pw[2] > height_thres) continue;               // 高度限制

            if (d.tag_id == 1) exist_hero = true;

            if (int(robot_status_long->game_state) == 0) {
                double distance = m_pw.norm();
                if (distance > distant_threshold) continue;
            }

            if (d.confidence >= high_thres)
                /* 阈值大于 high_thres 直接放行 */
                new_detections.push_back(d);
            else if (d.confidence >= low_thres) {
                /* 阈值介于 [low_thres, high_thres) 之间需要上次相同位置存在过装甲板才可放行 */
                auto center = points_center(d.pts);
                for (auto &tmp: last_boxes) {
                    Eigen::Vector3d tmp_last_m_pc = pnp_get_pc(tmp.pts, tmp.tag_id);
                    Eigen::Vector3d tmp_last_m_pw = pc_to_pw(tmp_last_m_pc, last_R_IW);
                    if (center.inside(get_ROI(tmp)) || is_same_armor_by_distance(tmp_last_m_pw, d, R_IW)) {
                        new_detections.push_back(d);
                        break;
                    }
                }
            }
        } else if (d.color_id == 2 && last_shoot && d.tag_id == last_sbbox.tag_id && dead_buffer <= dead_buffer_max_size) {
            /* 放行因为被击打灭的装甲板 */
            if (robot_status_long->enemy.at(d.tag_id) == 0) {
                dead_buffer = dead_buffer_max_size + 1;
                continue;
            }
            ++dead_buffer;
            new_detections.push_back(d);
        }
    }

    /// 没有有效的装甲板
    if (new_detections.empty()) {
        clear();
        return false;
    }

    /// 以下代码按照顺序一次构成优先级筛选
    /// 上下云台联动击打同一个目标
    if (robot_status_short->target_id != 255 && ((last_shoot && last_sbbox.tag_id != robot_status_short->target_id) || !last_shoot)) {
        for (auto &d: new_detections) {
            if (d.tag_id == robot_status_short->target_id) {
                if (DEBUG) std::cout << "The other need tag_id: " << robot_status_short->target_id << ". " << std::endl;

                if (int(robot_status_long->game_state) == 0) {
                    Eigen::Vector3d m_pc = pnp_get_pc(d.pts, d.tag_id);     // point camera: 目标在相机坐标系下的坐标
                    Eigen::Vector3d m_pw = pc_to_pw(m_pc, R_IW);            // point world: 目标在世界坐标系下的坐标。（世界坐标系:陀螺仪的全局世界坐标系）
                    double distance = m_pw.norm();
                    if (distance > distant_threshold) continue;
                }

                armor = d;
                selected = true;
                same_armor = false;
                same_id = false;
                switch_armor = false;
                need_init = true;
            }
        }
    }

    /// 击打上一次击打的目标
    if (!selected && last_shoot) {
        for (auto &d: new_detections) {
            auto center = points_center(d.pts);
            if (last_shoot && (center.inside(get_ROI(last_sbbox)) || is_same_armor_by_distance(last_m_pw, d, R_IW))) {
                armor = d;
                selected = true;
                same_armor = true;
                same_id = false;
                switch_armor = false;
                need_init = false;
                if (DEBUG) std::cout << "selected same armor!" << std::endl;
                break;
            }
        }
    }

    /// 防抖判断尝试确定上一次击打装甲板的位置
    if (!selected && last_shoot) {
        // 判断是否是一块装甲板或两块装甲板情况，更多装甲板不考虑防抖
        // 这一部分在 README 中有详细讲解
        unsigned int i = 0;
        for (auto &d: detections) {
            if (last_sbbox.tag_id == d.tag_id) ++i;
        }
        if (i == 1) {
            this_is_one_armor = true;
            this_is_two_armors = false;
        } else if (i == 2) {
            this_is_one_armor = false;
            this_is_two_armors = true;
        } else {
            this_is_one_armor = false;
            this_is_two_armors = false;
            std::cout << "[WARNING] multiple armors detected!" << std::endl;
        }
        if (last_is_one_armor && this_is_one_armor) {
            /* 如果出现一块装甲板，选择该装甲板更新 */
            unsigned int i = 0;
            for (auto &d: new_detections) {
                if (d.tag_id == last_sbbox.tag_id) {
                    armor = d;
                    break;
                }
            }
            selected = true;
            same_armor = true;
            same_id = false;
            switch_armor = false;
            need_init = false;
        }
        else if (last_is_two_armors && this_is_two_armors) {
            /* 如果出现两块装甲板，选择同侧的装甲板更新 */
            match_armors(armor, selected, new_detections, R_IW, right);
            if (selected) {
                same_armor = true;
                same_id = false;
                switch_armor = false;
                need_init = false;
            }
        }
        else if (last_is_two_armors && this_is_one_armor && antitop) {
            /* 反陀螺模式下，如果出现一块装甲板，选择上次陀螺转向落后的一块更新 */
            selected = true;
            if ((clockwise && !right) || (!clockwise && right)) {
                /* 上一次击打的装甲板就是落后的一块 */
                for (auto &d: new_detections) {
                    if (d.tag_id == last_sbbox.tag_id) {
                        armor = d;
                        break;
                    }
                }
                same_armor = true;
                same_id = false;
                switch_armor = false;
                need_init = false;
            } else if ((clockwise && right) || (!clockwise && !right)) {
                /* 上一次击打的装甲板不是落后的一块，此时需要切换装甲板，调用候选预测器 */
                same_id = true;
                switch_armor = true;

                // 选中本次击打的装甲板
                for (auto &d: new_detections) {
                    if (d.tag_id == last_sbbox.tag_id) {
                        armor = d;
                        break;
                    }
                }

                if (antitop_candidates.size() != 0) {
                    /* 查找对应的候选预测器，设置为主预测器，将主预测器放入候选队列 */
                    Eigen::Vector3d m_pc = pnp_get_pc(armor.pts, armor.tag_id); // point camera: 目标在相机坐标系下的坐标
                    Eigen::Vector3d m_pw = pc_to_pw(m_pc, R_IW);                // point world: 目标在世界坐标系下的坐标。（世界坐标系:陀螺仪的全局世界坐标系）
                    unsigned long int i = -1, j = -1;
                    double min_distance = 100;
                    for (auto &ac: antitop_candidates) {
                        /* 实际操作选择距离上次最近的那一个，避免上次误识别 */
                        ++j;
                        Eigen::Vector3d tmp_m_pc = pnp_get_pc(ac.bbox.pts, ac.bbox.tag_id); // point camera: 目标在相机坐标系下的坐标
                        Eigen::Vector3d tmp_m_pw = pc_to_pw(tmp_m_pc, R_IW);                // point world: 目标在世界坐标系下的坐标。（世界坐标系:陀螺仪的全局世界坐标系）
                        double distance = (tmp_m_pw - m_pw).norm();
                        if (distance < min_distance) {
                            i = j;
                            min_distance = distance;
                        }
                    }
                    Record r(armor);
                    r.ekf = ekf;
                    r.updated = false;
                    r.init = need_init;
                    r.last_m_pw = last_m_pw;
                    r.last_yaw = last_yaw;
                    r.last_pitch = last_pitch;
                    antitop_candidates.push_back(r);

                    ekf = antitop_candidates[i].ekf;
                    load_param(false);
                    last_m_pw = antitop_candidates[i].last_m_pw;
                    last_pitch = antitop_candidates[i].last_pitch;
                    last_yaw = antitop_candidates[i].last_yaw;

                    antitop_candidates.erase(antitop_candidates.begin() + i);

                    same_armor = true;
                    need_init = false;
                } else {
                    selected = false;
                    same_armor = false;
                    same_id = false;
                    switch_armor = false;
                    need_init = true;
                }
            }
        }
        else if (last_is_one_armor && this_is_two_armors && antitop) {
            /* 反陀螺模式下，如果出现两块装甲板，选择这一次靠前的装甲板更新 */
            selected = true;
            same_armor = true;
            same_id = false;
            need_init = false;
            bbox_t t1, t2;
            Eigen::Vector3d m_pw1, m_pw2;
            bool flag = false;
            for (auto &t: new_detections) {
                if (t.tag_id == last_sbbox.tag_id) {
                    Eigen::Vector3d tmp_m_pc = pnp_get_pc(t.pts, t.tag_id);
                    Eigen::Vector3d tmp_m_pw = pc_to_pw(tmp_m_pc, R_IW);
                    if (!flag) {
                        t1 = t;
                        m_pw1 = tmp_m_pw;
                        flag = true;
                    } else {
                        t2 = t;
                        m_pw2 = tmp_m_pw;
                        flag = false;
                        break;
                    }
                }
            }
            if (clockwise) {
                if (m_pw1[0] > m_pw2[0]) {
                    armor = t1;
                    right = true;
                }
                else {
                    armor = t2;
                    right = false;
                }
            } else {
                if (m_pw1[0] < m_pw2[0]) {
                    armor = t1;
                    right = false;
                }
                else {
                    armor = t2;
                    right = true;
                }
            }
        } else {
            selected = false;
        }
    }

    /// 切换最重要装甲板，不管此刻击打的是谁
    if ((!selected && last_shoot && last_sbbox.tag_id != important_id) || (selected && armor.tag_id != important_id && robot_status_long->enemy.at(armor.tag_id)*10 <= 100)) {
        for (auto &d: new_detections) {
            if (d.tag_id == important_id) {
                Eigen::Vector3d m_pc = pnp_get_pc(d.pts, d.tag_id);     // point camera: 目标在相机坐标系下的坐标
                Eigen::Vector3d m_pw = pc_to_pw(m_pc, R_IW);            // point world: 目标在世界坐标系下的坐标。（世界坐标系:陀螺仪的全局世界坐标系）
                double distance = m_pw.norm();
                if (distance > distant_threshold) break;
                armor = d;
                selected = true;
                same_armor = false;
                same_id = false;
                switch_armor = false;
                need_init = true;
                break;
            }
        }
    }

    /// 选择同一辆车的其他装甲板击打
    if (!selected && last_shoot) {
        for (auto &d: new_detections) {
            if (d.tag_id == last_sbbox.tag_id) {
                armor = d;
                selected = true;
                same_armor = false;
                same_id = true;
                switch_armor = true;
                if (DEBUG) std::cout << "same id." << std::endl;
                break;
            }
        }
    }

    /// 重置预测器，选择新的最临近装甲板
    if (!selected) {
        // 按照装甲板的大小排序，运用海伦公式
        // 海伦公式计算不规则四边形面积
        // 任意四边形的四条边分别为：AB=a,BC=b,CD=c,DA=d
        // 假设一个系数z,其中z=(a+b+c+d)/2
        // 那么任意四边形的面积S=2*[√(z-a)*(z-b)*(z-c)*(z-d)]
        double max_size = -1.;
        for (auto &d: new_detections) {
            auto size = get_quadrangle_size(d);
            if (size > max_size) {
                armor = d;
                max_size = size;
            }
            if (robot_status_long->enemy.at(d.tag_id)*10 <= killer_point) {
                armor = d;
                break;
            }
        }
        // 我知道可以用 sort ，但比赛前发现了很神奇的 bug
        selected = true;
        same_armor = false;
        same_id = false;
        switch_armor = false;
        need_init = true;
    }

    /// 判断本次是否是一块装甲板或两块装甲板情况，更多装甲板，用来记录左右侧，用于下次防抖
    unsigned int i = 0;
    for (auto &d: detections) {
        if (armor.tag_id == d.tag_id) ++i;
    }
    if (i == 1) {
        this_is_one_armor = true;
        this_is_two_armors = false;
    } else if (i == 2) {
        this_is_one_armor = false;
        this_is_two_armors = true;
    } else {
        this_is_one_armor = false;
        this_is_two_armors = false;
        std::cout << "[WARNING] multiple armors detected!" << std::endl;
    }
    if (DEBUG) std::cout << "last_is_one_armor = " << last_is_one_armor << ", last_is_two_armors = " << last_is_two_armors << std::endl;

    if (this_is_two_armors) {
        Eigen::Vector3d armor_m_pc = pnp_get_pc(armor.pts, armor.tag_id);
        Eigen::Vector3d armor_m_pw = pc_to_pw(armor_m_pc, R_IW);
        for (auto &d: new_detections) {
            if (d.tag_id == armor.tag_id && d != armor) {
                Eigen::Vector3d tmp_m_pc = pnp_get_pc(d.pts, d.tag_id);
                Eigen::Vector3d tmp_m_pw = pc_to_pw(tmp_m_pc, R_IW);
                if (armor_m_pw(0, 0) > tmp_m_pw(0, 0)) right = true;
                else right = false;
                break;
            }
        }
    }

    /// 重置所有预测器的标志位，置为非更新+非初始化
    for (auto &a : antitop_candidates) {
        a.updated = false;
        a.init = false;
    }

    /// 如果需要击打同 id 装甲板，从候选队列中寻找候选预测器，设为主预测器
    if (same_id && !antitop) {
        long unsigned int i = 0;
        for (i = 0; i < antitop_candidates.size(); ++i) {
            auto center = points_center(armor.pts);
            if (center.inside(get_ROI(antitop_candidates[i].bbox)) || is_same_armor_by_distance(antitop_candidates[i].last_m_pw, armor, R_IW)) {
                ekf = antitop_candidates[i].ekf;
                load_param(false);
                last_m_pw = antitop_candidates[i].last_m_pw;
                last_pitch = antitop_candidates[i].last_pitch;
                last_yaw = antitop_candidates[i].last_yaw;
                if (DEBUG) std::cout << "choose candidates." << std::endl;

                if (this_is_two_armors) right = !right;
                break;
            }
        }
        if (i < antitop_candidates.size()) {
            antitop_candidates.erase(antitop_candidates.begin() + i);
            same_armor = true;      // 这个操作以后实际上对以后就是击打同一块装甲板了！
            need_init = false;
        } else {
            need_init = true;
        }
    }
    /// 如果需要击打同一块装甲板，使用 EKF 预测解算
    if (same_armor) {
        /* 解算世界坐标系 */
        Eigen::Vector3d m_pc = pnp_get_pc(armor.pts, armor.tag_id);     // point camera: 目标在相机坐标系下的坐标
        Eigen::Vector3d m_pw = pc_to_pw(m_pc, R_IW);                    // point world: 目标在世界坐标系下的坐标。（世界坐标系:陀螺仪的全局世界坐标系）

        double mc_yaw = std::atan2(m_pc(1, 0), m_pc(0, 0));
        double m_yaw = std::atan2(m_pw(1, 0), m_pw(0, 0));              // yaw的测量值，单位弧度
        double mc_pitch = std::atan2(m_pc(2, 0), sqrt(m_pc(0, 0) * m_pc(0, 0) + m_pc(1, 0) * m_pc(1, 0)));
        double m_pitch = std::atan2(m_pw(2, 0), sqrt(m_pw(0, 0) * m_pw(0, 0) + m_pw(1, 0) * m_pw(1, 0)));   // pitch的测量值，单位弧度

        last_m_pw = m_pw;
        last_yaw = m_yaw;
        last_pitch = m_pitch;

        Predict predictfunc;
        Measure measure;

        Eigen::Matrix<double, 5, 1> Xr;
        Xr << m_pw(0, 0), 0, m_pw(1, 0), 0, m_pw(2, 0);
        Eigen::Matrix<double, 3, 1> Yr;
        measure(Xr.data(), Yr.data());      // 转化成相机求坐标系 Yr
        predictfunc.delta_t = delta_t;      // 设置距离上次预测的时间
        ekf.predict(predictfunc);           // 更新预测器，此时预测器里的是预测值
        Eigen::Matrix<double, 5, 1> Xe = ekf.update(measure, Yr);   // 更新滤波器，输入真实的球面坐标 Yr
        double predict_time = m_pw.norm() / robot_status_long->robot_speed_mps + shoot_delay;   // 预测时间=发射延迟+飞行时间（单位:s）
        // TODO：在反陀螺模式下对速度做衰减因子
        predictfunc.delta_t = predict_time;     // 设置本次预测时间
        Eigen::Matrix<double, 5, 1> Xp;
        predictfunc(Xe.data(), Xp.data());      // 使用匀速直线模型直接预测 Xp
        Eigen::Vector3d c_pw{Xe(0, 0), Xe(2, 0), Xe(4, 0)};
        Eigen::Vector3d p_pw{Xp(0, 0), Xp(2, 0), Xp(4, 0)};

        // 抬枪补偿
        double p_pitch = atan2(p_pw(2, 0), p_pw.topRows<2>().norm());
        double distance = p_pw.norm();
        if (DEBUG) std::cout << "distance: " << distance << std::endl;
        if (distance > distant_threshold) distant = true;
        double a = 9.8 * 9.8 * 0.25;
        double b = -robot_status_long->robot_speed_mps * robot_status_long->robot_speed_mps -
                   distance * 9.8 * cos(M_PI_2 + p_pitch);
        double c = distance * distance;
        double t_2 = (-sqrt(b * b - 4 * a * c) - b) / (2 * a);
        double fly_time = sqrt(t_2);            // 子弹飞行时间（单位:s）
        // 解出抬枪高度，即子弹下坠高度
        double height = 0.5 * 9.8 * t_2;
        Eigen::Vector3d s_pw{p_pw(0, 0), p_pw(1, 0), p_pw(2, 0) + height};  // 抬枪后预测点

        Eigen::Vector3d s_pc = pw_to_pc(s_pw, R_IW);
        double s_yaw = atan(s_pc(0, 0) / s_pc(2, 0)) / M_PI * 180.;
        double s_pitch = atan(s_pc(1, 0) / s_pc(2, 0)) / M_PI * 180.;

        if (is_show) {
            /// 把世界坐标系中的点，重投影到图像中
            re_project_point(im2show, c_pw, R_IW, {0, 255, 0});
            re_project_point(im2show, p_pw, R_IW, {255, 0, 0});
            re_project_point(im2show, s_pw, R_IW, {0, 0, 255});
            for (int i = 0; i < 4; ++i)
                cv::circle(im2show, armor.pts[i], 3, {255, 0, 0});
            cv::circle(im2show, {im2show.cols / 2, im2show.rows / 2}, 3, {0, 255, 0});
        }

        // 求导解算角速度
        predictfunc.delta_t = 0.001;
        Eigen::Matrix<double, 5, 1> Xd;
        predictfunc(Xp.data(), Xd.data());
        Eigen::Vector3d d_pw{Xd(0, 0), Xd(2, 0), Xd(4, 0)};
        double d_pitch = atan2(d_pw(2, 0), d_pw.topRows<2>().norm());
        double d_distance = d_pw.norm();
        double d_a = 9.8 * 9.8 * 0.25;
        double d_b = -robot_status_long->robot_speed_mps * robot_status_long->robot_speed_mps -
                     d_distance * 9.8 * cos(M_PI_2 + d_pitch);
        double d_c = d_distance * d_distance;
        double d_t_2 = (-sqrt(d_b * d_b - 4 * d_a * d_c) - d_b) / (2 * d_a);
        double d_fly_time = sqrt(d_t_2);        // 子弹飞行时间（单位:s）
        // 解出抬枪高度，即子弹下坠高度
        double d_height = 0.5 * 9.8 * d_t_2;
        Eigen::Vector3d ds_pw{d_pw(0, 0), d_pw(1, 0), d_pw(2, 0) + d_height};   // 抬枪后预测点
        Eigen::Vector3d ds_pc = pw_to_pc(ds_pw, R_IW);
        double ds_yaw = atan(ds_pc(0, 0) / ds_pc(2, 0)) / M_PI * 180.;
        double ds_pitch = atan(ds_pc(1, 0) / ds_pc(2, 0)) / M_PI * 180.;

        send.distance = (int) (distance * 10);
        send.yaw_angle = (float) s_yaw;
        send.pitch_angle = (float) s_pitch;
        // 速度环是弧度
        send.yaw_speed = (float) (ds_yaw - s_yaw) / 0.001 / 180. * M_PI;
        send.pitch_speed = (float) (ds_pitch - s_pitch) / 0.001 / 180. * M_PI;
    }
    /// 需要重新设置预测器，此时无法发送给电控有效的预测值
    if (need_init) {
        if (DEBUG) std::cout << "need init" << std::endl;
        Eigen::Vector3d m_pc = pnp_get_pc(armor.pts, armor.tag_id);             // point camera: 目标在相机坐标系下的坐标
        Eigen::Vector3d m_pw = pc_to_pw(m_pc, R_IW);          // point world: 目标在世界坐标系下的坐标。（世界坐标系:陀螺仪的全局世界坐标系）
        double mc_yaw = std::atan2(m_pc(1, 0), m_pc(0, 0));
        double m_yaw = std::atan2(m_pw(1, 0), m_pw(0, 0));   // yaw的测量值，单位弧度
        double mc_pitch = std::atan2(m_pc(2, 0), sqrt(m_pc(0, 0) * m_pc(0, 0) + m_pc(1, 0) * m_pc(1, 0)));
        double m_pitch = std::atan2(m_pw(2, 0),
                                    sqrt(m_pw(0, 0) * m_pw(0, 0) + m_pw(1, 0) * m_pw(1, 0)));   // pitch的测量值，单位弧度
        Eigen::Matrix<double, 5, 1> Xr;
        Xr << m_pw(0, 0), 0, m_pw(1, 0), 0, m_pw(2, 0);
        ekf.init(Xr);
        last_m_pw = m_pw;
        last_yaw = m_yaw;
        last_pitch = m_pitch;

        double distance = m_pw.norm();
        if (distance > distant_threshold) distant = true;
        double a = 9.8 * 9.8 * 0.25;
        double b = -robot_status_long->robot_speed_mps * robot_status_long->robot_speed_mps -
                   distance * 9.8 * cos(M_PI_2 + m_pitch);
        double c = distance * distance;
        double t_2 = (-sqrt(b * b - 4 * a * c) - b) / (2 * a);
        double fly_time = sqrt(t_2);            // 子弹飞行时间（单位:s）
        // 解出抬枪高度，即子弹下坠高度
        double height = 0.5 * 9.8 * t_2;
        Eigen::Vector3d s_pw{m_pw(0, 0), m_pw(1, 0), m_pw(2, 0) + height}; // 抬枪后预测点

        Eigen::Vector3d s_pc = pw_to_pc(s_pw, R_IW);
        double s_yaw = atan(s_pc(0, 0) / s_pc(2, 0)) / M_PI * 180.;
        double s_pitch = atan(s_pc(1, 0) / s_pc(2, 0)) / M_PI * 180.;

        send.distance = (int) (distance * 10);
        send.yaw_angle = (float) s_yaw;
        send.pitch_angle = (float) s_pitch;
        // 速度环是弧度
        send.yaw_speed = 0.;
        send.pitch_speed = 0.;
    }

    /// 预测完毕，开始更新候选预测器，更新逻辑与主预测器几乎完全一致，不再注释
    if (DEBUG) std::cout << "update ekf vector" << std::endl;
    if (need_init) antitop_candidates.clear();

    for (auto &d: new_detections) {
        if (DEBUG) std::cout << "---\ttag_id = " << d.tag_id << "\t---" << std::endl;
        if (d.tag_id != armor.tag_id || d == armor) continue;  // 如果装甲板id与当前装甲板不同或就是目标装甲板
        bool exist = false;
        auto center = points_center(d.pts);
        for (auto &ac: antitop_candidates) {
            if (ac.updated) continue;
            bool flag = false;
            bbox_t bx = d;
            if (center.inside(get_ROI(ac.bbox)) || is_same_armor_by_distance(ac.last_m_pw, d, R_IW)) {
                flag = true;
            } else if (last_is_two_armors && this_is_two_armors) {
                match_armors(bx, flag, new_detections, R_IW, !right);
            }
            if (flag) {
                /* 存在和候选装甲板匹配的候选滤波器 */
                exist = true;
                if (DEBUG) std::cout << "updated!" << std::endl;

                // 更新
                ac.bbox = bx;
                ac.updated = true;

                ++ac.age;
                if (ac.age <= ac_init_min_age) {
                    ac.init = true;
                    Eigen::Vector3d m_pc = pnp_get_pc(d.pts, d.tag_id);     // point camera: 目标在相机坐标系下的坐标
                    Eigen::Vector3d m_pw = pc_to_pw(m_pc, R_IW);            // point world: 目标在世界坐标系下的坐标。（世界坐标系:陀螺仪的全局世界坐标系）
                    double mc_yaw = std::atan2(m_pc(1, 0), m_pc(0, 0));
                    double m_yaw = std::atan2(m_pw(1, 0), m_pw(0, 0));      // yaw的测量值，单位弧度
                    double mc_pitch = std::atan2(m_pc(2, 0), sqrt(m_pc(0, 0) * m_pc(0, 0) + m_pc(1, 0) * m_pc(1, 0)));
                    double m_pitch = std::atan2(m_pw(2, 0), sqrt(m_pw(0, 0) * m_pw(0, 0) + m_pw(1, 0) * m_pw(1, 0)));   // pitch的测量值，单位弧度
                    Eigen::Matrix<double, 5, 1> Xr;
                    Xr << m_pw(0, 0), -ac_x_v_coefficient * ekf.Xe(1, 0), m_pw(1, 0), -ac_y_v_coefficient * ekf.Xe(3, 0), m_pw(2, 0);  // 给予一个合理的初值，便于反陀螺
                    ac.ekf.init(Xr);
                    ac.last_m_pw = m_pw;
                    ac.last_yaw = m_yaw;
                    ac.last_pitch = m_pitch;
                } else {
                    ac.init = false;
                    Eigen::Vector3d m_pc = pnp_get_pc(d.pts, d.tag_id);     // point camera: 目标在相机坐标系下的坐标
                    Eigen::Vector3d m_pw = pc_to_pw(m_pc, R_IW);            // point world: 目标在世界坐标系下的坐标。（世界坐标系:陀螺仪的全局世界坐标系）

                    double mc_yaw = std::atan2(m_pc(1, 0), m_pc(0, 0));
                    double m_yaw = std::atan2(m_pw(1, 0), m_pw(0, 0));      // yaw的测量值，单位弧度
                    double mc_pitch = std::atan2(m_pc(2, 0), sqrt(m_pc(0, 0) * m_pc(0, 0) + m_pc(1, 0) * m_pc(1, 0)));
                    double m_pitch = std::atan2(m_pw(2, 0), sqrt(m_pw(0, 0) * m_pw(0, 0) + m_pw(1, 0) * m_pw(1, 0)));   // pitch的测量值，单位弧度

                    ac.last_m_pw = m_pw;
                    ac.last_yaw = m_yaw;
                    ac.last_pitch = m_pitch;

                    Predict predictfunc;
                    Measure measure;
                    predictfunc.delta_t = delta_t;
                    ac.ekf.predict(predictfunc);
                    Eigen::Matrix<double, 5, 1> Xr;
                    Xr << m_pw(0, 0), 0, m_pw(1, 0), 0, m_pw(2, 0);
                    Eigen::Matrix<double, 3, 1> Yr;
                    measure(Xr.data(), Yr.data());
                    Eigen::Matrix<double, 5, 1> Xe = ac.ekf.update(measure, Yr);
                    double predict_time = m_pw.norm() / robot_status_long->robot_speed_mps + shoot_delay;
                    predictfunc.delta_t = predict_time;
                    Eigen::Matrix<double, 5, 1> Xp;
                    predictfunc(Xe.data(), Xp.data());
                    Eigen::Vector3d c_pw{Xe(0, 0), Xe(2, 0), Xe(4, 0)};
                    Eigen::Vector3d p_pw{Xp(0, 0), Xp(2, 0), Xp(4, 0)};

                    double p_pitch = atan2(p_pw(2, 0), p_pw.topRows<2>().norm());
                    double distance = p_pw.norm();
                    if (distance > distant_threshold) ac.distant = true;
                    double a = 9.8 * 9.8 * 0.25;
                    double b = -robot_status_long->robot_speed_mps * robot_status_long->robot_speed_mps -
                               distance * 9.8 * cos(M_PI_2 + p_pitch);
                    double c = distance * distance;
                    double t_2 = (-sqrt(b * b - 4 * a * c) - b) / (2 * a);
                    double fly_time = sqrt(t_2);
                    double height = 0.5 * 9.8 * t_2;
                    Eigen::Vector3d s_pw{p_pw(0, 0), p_pw(1, 0), p_pw(2, 0) + height};

                    Eigen::Vector3d s_pc = pw_to_pc(s_pw, R_IW);
                    double s_yaw = atan(s_pc(0, 0) / s_pc(2, 0)) / M_PI * 180.;
                    double s_pitch = atan(s_pc(1, 0) / s_pc(2, 0)) / M_PI * 180.;

                    if (is_show) {
                        /// 把世界坐标系中的点，重投影到图像中
                        re_project_point(im2show, c_pw, R_IW, {0, 255, 0});
                        re_project_point(im2show, p_pw, R_IW, {255, 0, 0});
                        re_project_point(im2show, s_pw, R_IW, {0, 0, 255});
                        for (int i = 0; i < 4; ++i)
                            cv::circle(im2show, d.pts[i], 3, {255, 0, 0});
                    }

                    // 求导
                    predictfunc.delta_t = 0.001;
                    Eigen::Matrix<double, 5, 1> Xd;
                    predictfunc(Xp.data(), Xd.data());
                    Eigen::Vector3d d_pw{Xd(0, 0), Xd(2, 0), Xd(4, 0)};
                    double d_pitch = atan2(d_pw(2, 0), d_pw.topRows<2>().norm());
                    double d_distance = d_pw.norm();
                    double d_a = 9.8 * 9.8 * 0.25;
                    double d_b = -robot_status_long->robot_speed_mps * robot_status_long->robot_speed_mps -
                                 d_distance * 9.8 * cos(M_PI_2 + d_pitch);
                    double d_c = d_distance * d_distance;
                    // 带入求根公式，解出t^2
                    double d_t_2 = (-sqrt(d_b * d_b - 4 * d_a * d_c) - d_b) / (2 * d_a);
                    double d_fly_time = sqrt(d_t_2);
                    double d_height = 0.5 * 9.8 * d_t_2;
                    Eigen::Vector3d ds_pw{d_pw(0, 0), d_pw(1, 0), d_pw(2, 0) + d_height};
                    Eigen::Vector3d ds_pc = pw_to_pc(ds_pw, R_IW);
                    double ds_yaw = atan(ds_pc(0, 0) / ds_pc(2, 0)) / M_PI * 180.;
                    double ds_pitch = atan(ds_pc(1, 0) / ds_pc(2, 0)) / M_PI * 180.;

                    ac.distance = (int) (distance * 10);
                    ac.yaw_angle = (float) s_yaw;
                    ac.pitch_angle = (float) s_pitch;
                    ac.yaw_speed = (float) (ds_yaw - s_yaw) / 0.001 / 180. * M_PI;
                    ac.pitch_speed = (float) (ds_pitch - s_pitch) / 0.001 / 180. * M_PI;
                }
                break;
            }
        }
        if (!exist) {
            /* 不存在和候选装甲板匹配的候选滤波器，需要初始化 */
            std::cout << "no suitable. try add new armor." << std::endl;
            Record r(d);
            Eigen::Vector3d m_pc = pnp_get_pc(d.pts, d.tag_id);             // point camera: 目标在相机坐标系下的坐标
            Eigen::Vector3d m_pw = pc_to_pw(m_pc, R_IW);          // point world: 目标在世界坐标系下的坐标。（世界坐标系:陀螺仪的全局世界坐标系）
            double mc_yaw = std::atan2(m_pc(1, 0), m_pc(0, 0));
            double m_yaw = std::atan2(m_pw(1, 0), m_pw(0, 0));   // yaw的测量值，单位弧度
            double mc_pitch = std::atan2(m_pc(2, 0), sqrt(m_pc(0, 0) * m_pc(0, 0) + m_pc(1, 0) * m_pc(1, 0)));
            double m_pitch = std::atan2(m_pw(2, 0),
                                        sqrt(m_pw(0, 0) * m_pw(0, 0) + m_pw(1, 0) * m_pw(1, 0)));   // pitch的测量值，单位弧度
            Eigen::Matrix<double, 5, 1> Xr;
            Xr << m_pw(0, 0), 0.5 * ekf.Xe(1, 0), m_pw(1, 0), 0.25 * ekf.Xe(3, 0), m_pw(2, 0);
            r.ekf.init(Xr);
            r.last_m_pw = m_pw;
            r.last_yaw = m_yaw;
            r.last_pitch = m_pitch;

            r.updated = true;
            r.init = true;

            antitop_candidates.push_back(r);
        }
    }

    /// 删除所有未更新的候选预测器，代表其可能已消失
    int len = antitop_candidates.size();
    for (long unsigned int i = 0; i < antitop_candidates.size(); ++i) {
        if (!antitop_candidates[i].updated) {
            antitop_candidates.erase(antitop_candidates.begin() + i);
            --i;
            --len;
        }
    }

    /// 选择较大的一块装甲板进行击打，提高预测命中率
    double size1 = get_quadrangle_size(armor);
    for (long unsigned int i = 0; i < antitop_candidates.size(); ++i) {
        if (antitop_candidates[i].init) continue;
        double size2 = get_quadrangle_size(antitop_candidates[i].bbox);
        if (size2 > switch_armor_size_proportion * size1) {
            /* 主预测器放入候选队列，候选预测器放入主预测器，更新状态变量 */
            if (DEBUG) std::cout << "Switch armor due to size scale." << std::endl;
            switch_armor = true;
            Record r(armor);
            r.ekf = ekf;
            r.updated = true;
            r.init = need_init;
            r.last_m_pw = last_m_pw;
            r.last_yaw = last_yaw;
            r.last_pitch = last_pitch;
            r.pitch_angle = send.pitch_angle;
            r.pitch_speed = send.pitch_speed;
            r.yaw_angle = send.yaw_angle;
            r.yaw_speed = send.yaw_speed;
            r.distance = send.distance;
            r.distant = distant;
            antitop_candidates.push_back(r);

            armor = antitop_candidates[i].bbox;
            ekf = antitop_candidates[i].ekf;
            last_m_pw = antitop_candidates[i].last_m_pw;
            last_yaw = antitop_candidates[i].last_yaw;
            last_pitch = antitop_candidates[i].last_pitch;
            send.pitch_angle = antitop_candidates[i].pitch_angle;
            send.pitch_speed = antitop_candidates[i].pitch_speed;
            send.yaw_angle = antitop_candidates[i].yaw_angle;
            send.yaw_speed = antitop_candidates[i].yaw_speed;
            send.distance = r.distance;
            distant = antitop_candidates[i].distant;
            antitop_candidates.erase(antitop_candidates.begin() + i);

            if (this_is_two_armors) right = !right;
            if (send.yaw_angle > 0) clockwise = false;
            else clockwise = true;

            break;
        }
    }

    /// 热调参数 && 设置替换后预测器和新增预测器参数
    load_param(true);

    /// 突出强调本次击打的装甲板
    if (is_show) {
        for (int i = 0; i < 4; ++i)
            cv::circle(im2show, armor.pts[i], 3, {0, 255, 0}, 2);
    }

    /// 记录历史
    last_is_one_armor = this_is_one_armor;
    last_is_two_armors = this_is_two_armors;

    last_boxes = std::move(new_detections);
    last_sbbox = armor;
    last_shoot = true;

    /// 发送电控指令
    send.shoot_mode = static_cast<uint8_t>(ShootMode::COMMON);
    if (distant) send.shoot_mode += static_cast<uint8_t>(ShootMode::DISTANT);
    if (antitop) send.shoot_mode += static_cast<uint8_t>(ShootMode::ANTITOP);
    if (switch_armor) send.shoot_mode += static_cast<uint8_t>(ShootMode::SWITCH);
    if (!(std::abs(send.yaw_angle) < 90. && std::abs(send.pitch_angle) < 90.))
        send.shoot_mode = static_cast<uint8_t>(ShootMode::CRUISE);
    if (exist_hero) send.shoot_mode += static_cast<uint8_t>(ShootMode::EXIST_HERO);

    send.target_id = armor.tag_id;
    send.priority = Priority::EMERGENCY;

    return true;
}

float PredictorAdaptiveEKF::bbOverlap(const cv::Rect2f& box1,const cv::Rect2f& box2)
{
    if (box1.x > box2.x+box2.width) { return 0.0; }
    if (box1.y > box2.y+box2.height) { return 0.0; }
    if (box1.x+box1.width < box2.x) { return 0.0; }
    if (box1.y+box1.height < box2.y) { return 0.0; }
    float colInt = std::min(box1.x+box1.width,box2.x+box2.width) - std::max(box1.x, box2.x);
    float rowInt = std::min(box1.y+box1.height,box2.y+box2.height) - std::max(box1.y,box2.y);
    float intersection = colInt * rowInt;
    float area1 = box1.width*box1.height;
    float area2 = box2.width*box2.height;
    return intersection / (area1 + area2 - intersection);
}

double PredictorAdaptiveEKF::get_quadrangle_size(const bbox_t &bx) {
    auto bx_a = sqrt(pow(bx.pts[0].x - bx.pts[1].x, 2) + pow(bx.pts[0].y - bx.pts[1].y, 2));
    auto bx_b = sqrt(pow(bx.pts[1].x - bx.pts[2].x, 2) + pow(bx.pts[1].y - bx.pts[2].y, 2));
    auto bx_c = sqrt(pow(bx.pts[2].x - bx.pts[3].x, 2) + pow(bx.pts[2].y - bx.pts[3].y, 2));
    auto bx_d = sqrt(pow(bx.pts[3].x - bx.pts[0].x, 2) + pow(bx.pts[3].y - bx.pts[0].y, 2));
    auto bx_z = (bx_a + bx_b + bx_c + bx_d) / 2;
    auto bx_size = 2 * sqrt((bx_z - bx_a) * (bx_z - bx_b) * (bx_z - bx_c) * (bx_z - bx_d));
    return bx_size;
}

void PredictorAdaptiveEKF::match_armors(bbox_t &armor, bool &selected, const std::vector<bbox_t> &detections, const Eigen::Matrix3d &R_IW, const bool right) {
    if (!last_shoot) {
        selected = false;
        return;
    }

    std::vector<bbox_t> temp_armors;
    for (auto &d: detections) {
        bbox_t tmp = d;
        if (d.tag_id == last_sbbox.tag_id) temp_armors.push_back(tmp);
    }
    if (temp_armors.size() == 2) {
        selected = false;
        Eigen::Vector3d m_pw1, m_pw2;
        bool flag = false;
        for (auto &t: temp_armors) {
            if (t.tag_id == last_sbbox.tag_id) {
                Eigen::Vector3d tmp_m_pc = pnp_get_pc(t.pts, t.tag_id);
                Eigen::Vector3d tmp_m_pw = pc_to_pw(tmp_m_pc, R_IW);
                if (!flag) {
                    m_pw1 = tmp_m_pw;
                    flag = true;
                }
                else {
                    m_pw2 = tmp_m_pw;
                    flag = false;
                }
            }
        }
        if (flag) std::cout << "[Error] in match_armors." << std::endl;
        if (right) {
            if (m_pw1(0, 0) > m_pw2(0, 0)) armor = temp_armors.front();
            else armor = temp_armors.back();
        }
        else {
            if (m_pw1(0, 0) < m_pw2(0, 0)) armor = temp_armors.front();
            else armor = temp_armors.back();
        }
        selected = true;
    }
    else {
        selected = false;
        return;
    }
}

Eigen::Vector3d PredictorAdaptiveEKF::pnp_get_pc(const cv::Point2f p[4], int armor_number) {
    static const std::vector <cv::Point3d> pw_small = {  // 单位：m
            {-0.066, 0.027,  0.},
            {-0.066, -0.027, 0.},
            {0.066,  -0.027, 0.},
            {0.066,  0.027,  0.}
    };
    static const std::vector <cv::Point3d> pw_big = {    // 单位：m
            {-0.115, 0.029,  0.},
            {-0.115, -0.029, 0.},
            {0.115,  -0.029, 0.},
            {0.115,  0.029,  0.}
    };
    std::vector <cv::Point2d> pu(p, p + 4);
    cv::Mat rvec, tvec;

    if (armor_number == 0 || armor_number == 1 || armor_number == 8)
        cv::solvePnP(pw_big, pu, F_MAT, C_MAT, rvec, tvec);
    else
        cv::solvePnP(pw_small, pu, F_MAT, C_MAT, rvec, tvec);

    Eigen::Vector3d pc;
    cv::cv2eigen(tvec, pc);
    /// 旧哨兵下云台
//    pc[1] += 0.045;
    /// 新哨兵下云台
//    pc[1] += 0.058;
//    pc[2] += 0.02385;
    /// 新哨兵上云台
    pc[0] += 0.0475;
    pc[1] += 0.0165;
    pc[2] += 0.02385;
    return pc;
}
