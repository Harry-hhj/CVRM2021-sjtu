#ifndef _KALMAN_H_
#define _KALMAN_H_

#include <Eigen/Dense>

template<int V_Z = 1, int V_X = 3>
class Kalman {
public:
    using Matrix_zzd = Eigen::Matrix<double, V_Z, V_Z>;
    using Matrix_xxd = Eigen::Matrix<double, V_X, V_X>;
    using Matrix_zxd = Eigen::Matrix<double, V_Z, V_X>;
    using Matrix_xzd = Eigen::Matrix<double, V_X, V_Z>;
    using Matrix_x1d = Eigen::Matrix<double, V_X, 1>;
    using Matrix_z1d = Eigen::Matrix<double, V_Z, 1>;
private:
    Matrix_x1d x_k1; // k-1时刻的滤波值，即是k-1时刻的值
    Matrix_xzd K;    // Kalman增益
    Matrix_xxd A;    // 转移矩阵
    Matrix_zxd H;    // 观测矩阵
    Matrix_xxd R;    // 预测过程噪声偏差的方差
    Matrix_zzd Q;    // 测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
    Matrix_xxd P;    // 估计误差协方差
    double last_t{0};
public:
    Kalman() = default;

    Kalman(Matrix_xxd A, Matrix_zxd H, Matrix_xxd R, Matrix_zzd Q, Matrix_x1d init, double t) {
        reset(A, H, R, Q, init, t);
    }

    void reset(Matrix_xxd A, Matrix_zxd H, Matrix_xxd R, Matrix_zzd Q, Matrix_x1d init, double t) {
        this->A = A;
        this->H = H;
        this->P = Matrix_xxd::Zero();
        this->R = R;
        this->Q = Q;
        x_k1 = init;
        last_t = t;
    }

    void reset(Matrix_x1d init, double t) {
        x_k1 = init;
        last_t = t;
    }

    void reset(double x, double t){
        x_k1(0,0) = x;
        last_t = t;
    }

    Matrix_x1d update(Matrix_z1d z_k, double t) {
        // 设置转移矩阵中的时间项
        for (int i = 1; i < V_X; i++) {
            A(i - 1, i) = t - last_t;
        }
        last_t = t;

        // 预测下一时刻的值
        Matrix_x1d p_x_k = A * x_k1;   //x的先验估计由上一个时间点的后验估计值和输入信息给出，此处需要根据基站高度做一个修改

        //求协方差
        P = A * P * A.transpose() + R;  //计算先验均方差 p(n|n-1)=A^2*p(n-1|n-1)+q

        //计算kalman增益
        K = P * H.transpose() * (H * P * H.transpose() + Q).inverse();  //Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R)
        //修正结果，即计算滤波值
        x_k1 = p_x_k +
               K * (z_k - H * p_x_k);  //利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))
        //更新后验估计
        P = (Matrix_xxd::Identity() - K * H) * P;//计算后验均方差  P[n|n]=(1-K[n]*H)*P[n|n-1]

        return x_k1;
    }
};

#endif /* _KALMAN_H_ */
