//
// Created by johan on 2024/1/11.
//

#ifndef EKF_EXTENDEDKARMANFILTER_H
#define EKF_EXTENDEDKARMANFILTER_H
#include "iostream"
#include "eigen3/Eigen/Dense"

using namespace std;
using namespace Eigen;


class ExtendedKarmanFilter {
private:
    VectorXd x_;  //状态向量
    MatrixXd P_;  //误差协方差矩阵
    VectorXd x_pred_;  //预测的状态向量
    MatrixXd P_pred_;  //预测的协方差矩阵
    MatrixXd F_;  //状态转移矩阵的雅可比矩阵
    MatrixXd H_;  //观测函数的雅可比矩阵
    MatrixXd R_;  //观测噪声协方差矩阵
    MatrixXd S_;  //观测残差的协方差矩阵
    MatrixXd K_;  //卡尔曼增益
    MatrixXd Q_;

public:
    ExtendedKarmanFilter()
    {
        x_ = VectorXd::Zero(3);
        P_ = MatrixXd::Identity(3,3);
        Q_ = MatrixXd::Zero(3,3);
    }
    void setInitialState(const VectorXd& x0)
    {
        x_= x0;
    }
    void setInitialCovariance(const MatrixXd& P0)
    {
        P_ = P0;
    }
    void predict();
    void update(const VectorXd& z);
    void setProcessNoiseCovariance(const MatrixXd& Q)
    {
        Q_= Q;
    }
    VectorXd getState() const
    {
        return x_;
    }
    MatrixXd getCovariance() const
    {
        return P_;
    }
    VectorXd stateTransitionFunction(const VectorXd& x);
    MatrixXd computeStateTransitionJacobian(const VectorXd& x);
    VectorXd observationFunction(const VectorXd& x);
    MatrixXd computeObservationJacobian(const VectorXd& x);

};


#endif //EKF_EXTENDEDKARMANFILTER_H
