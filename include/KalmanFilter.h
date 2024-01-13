//
// Created by johan on 2024/1/11.
//

#ifndef EKF_KALMANFILTER_H
#define EKF_KALMANFILTER_H

#include "iostream"
#include "eigen3/Eigen/Dense"

class KalmanFilter {
private:
    Eigen::VectorXd x; //状态估计向量
    Eigen::MatrixXd P; //估计误差协方差矩阵
    Eigen::MatrixXd Q; //系统噪声协方差矩阵
    Eigen::MatrixXd R; //测量噪声协方差矩阵
    Eigen::MatrixXd F; //状态转移矩阵
    Eigen::MatrixXd H; //观测矩阵

public:
    KalmanFilter(const Eigen::VectorXd& initial_x,const Eigen::MatrixXd& initial_P ,
                    const Eigen::MatrixXd& process_noise,const Eigen::MatrixXd& measurement_noise,
                    const Eigen::MatrixXd& state_transition,const Eigen::MatrixXd& observation)
                    :x(initial_x),P(initial_P),Q(process_noise),R(measurement_noise),F(state_transition),H(observation){}
    void predict();
    void update(const Eigen::VectorXd& measurement);
    Eigen::VectorXd getState() const
    {
        return x;
    }
};


#endif //EKF_KALMANFILTER_H
