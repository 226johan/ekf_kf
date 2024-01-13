//
// Created by johan on 2024/1/11.
//

#include "../include/KalmanFilter.h"


void KalmanFilter::predict()
{
    //预测
    //更新误差估计和估计误差协方差
    x = F * x;
    P = F * P * F.transpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXd &measurement) {
    //更新
    //计算卡尔曼增益
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    //更新状态估计矩阵和估计误差协方差
    x = x + K * (measurement - H * x);
    P = (Eigen::MatrixXd::Identity(x.size(),x.size()) - K * H) * P;
}