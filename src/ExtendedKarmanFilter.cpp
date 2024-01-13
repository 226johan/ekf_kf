//
// Created by johan on 2024/1/11.
//

#include "../include/ExtendedKarmanFilter.h"
void ExtendedKarmanFilter::predict()
{
    x_pred_= stateTransitionFunction(x_);
    F_ = computeStateTransitionJacobian(x_);

    P_pred_ = F_ * P_ * F_.transpose() + Q_;
}
void ExtendedKarmanFilter::update(const VectorXd& z)
{
    H_ = computeObservationJacobian(x_pred_);
    R_ = MatrixXd::Identity(z.size(),z.size());
    S_ = H_ * P_pred_ * H_.transpose() + R_;
    K_ = P_pred_ * H_.transpose() * S_.inverse();

    x_ = x_pred_ + K_ * (z - observationFunction(x_pred_));
    P_ = (MatrixXd::Identity(3,3) - K_ * H_) * P_pred_;
}
VectorXd ExtendedKarmanFilter::stateTransitionFunction(const VectorXd& x)
{
    VectorXd x_next(3);
    x_next(0) = x(0) * x(0) + 3 * x(0) * x(1) +2 * x(2);
    x_next(1) = 2 * x(0) - 2 * x(0) * x(1) + x(2) * x(2);
    x_next(2) = x(2);
    return x_next;
}
MatrixXd ExtendedKarmanFilter::computeStateTransitionJacobian(const VectorXd& x)
{
    MatrixXd F(3,3);
    F<<2 * x(0) + 3 * x(1), 3 * x(0), 2,
        2 - 2 * x(2), -2 * x(0) , 2 * x(2),
        0, 0, 1;
    return F;

}

//
VectorXd ExtendedKarmanFilter::observationFunction(const VectorXd& x)
{
    VectorXd z(2);
    z(0) = x(0) * x(0) + 3 * x(0) * x(1) + 2 * x(2);
    z(1) = 2 - 2 * x(1) - 2 * x(0) * x(1) + x(2) * x(2);
    return z;
}
MatrixXd ExtendedKarmanFilter::computeObservationJacobian(const VectorXd& x)
{
    MatrixXd H(2, 3);
    H << 2 * x(0) + 3 * x(1), 3 * x(0), 2 * x(2),
            -2 * x(1), -2 * x(0), 0;
    return H;
}