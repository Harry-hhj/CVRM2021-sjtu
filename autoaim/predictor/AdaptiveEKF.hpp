//
// Created by xinyang on 2021/3/15.
//

#ifndef _ADAPTIVE_EKF_HPP_
#define _ADAPTIVE_EKF_HPP_

#include <ceres/jet.h>
#include <Eigen/Dense>

template<int N_X, int N_Y>
class AdaptiveEKF {
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixYX = Eigen::Matrix<double, N_Y, N_X>;
    using MatrixXY = Eigen::Matrix<double, N_X, N_Y>;
    using MatrixYY = Eigen::Matrix<double, N_Y, N_Y>;
    using VectorX = Eigen::Matrix<double, N_X, 1>;
    using VectorY = Eigen::Matrix<double, N_Y, 1>;

public:
    explicit AdaptiveEKF(const VectorX &X0 = VectorX::Zero())
            : Xe(X0), P(MatrixXX::Identity()), Q(MatrixXX::Identity()), R(MatrixYY::Identity()) {}

    void init(const VectorX &X0 = VectorX::Zero()) {
        Xe = X0;
    }

    template<class Func>
    VectorX predict(Func &&func) {
        ceres::Jet<double, N_X> Xe_auto_jet[N_X];
        for (int i = 0; i < N_X; i++) {
            Xe_auto_jet[i].a = Xe[i];
            Xe_auto_jet[i].v[i] = 1;
        }
        ceres::Jet<double, N_X> Xp_auto_jet[N_X];
        func(Xe_auto_jet, Xp_auto_jet);
        for (int i = 0; i < N_X; i++) {
            Xp[i] = Xp_auto_jet[i].a;
            F.block(i, 0, 1, N_X) = Xp_auto_jet[i].v.transpose();
        }
        P = F * P * F.transpose() + Q;
        return Xp;
    }

    template<class Func>
    VectorX update(Func &&func, const VectorY &Y) {
        ceres::Jet<double, N_X> Xp_auto_jet[N_X];
        for (int i = 0; i < N_X; i++) {
            Xp_auto_jet[i].a = Xp[i];
            Xp_auto_jet[i].v[i] = 1;
        }
        ceres::Jet<double, N_X> Yp_auto_jet[N_Y];
        func(Xp_auto_jet, Yp_auto_jet);
        for (int i = 0; i < N_Y; i++) {
            Yp[i] = Yp_auto_jet[i].a;
            H.block(i, 0, 1, N_X) = Yp_auto_jet[i].v.transpose();
        }
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        Xe = Xp + K * (Y - Yp);
        P = (MatrixXX::Identity() - K * H) * P;
        return Xe;
    }

    void estimate() {
        /// TODO:
    }

    VectorX Xe;     // 估计状态变量
    VectorX Xp;     // 预测状态变量
    MatrixXX F;     // 预测雅克比
    MatrixYX H;     // 观测雅克比
    MatrixXX P;     // 状态协方差
    MatrixXX Q;     // 预测过程协方差
    MatrixYY R;     // 观测过程协方差
    MatrixXY K;     // 卡尔曼增益
    VectorY Yp;     // 预测观测量
};


#endif /* _ADAPTIVE_EKF_HPP_ */
