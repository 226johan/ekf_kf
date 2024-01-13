#include "../include/KalmanFilter.h"
#include "vector"
#include "../gnuplot-iostream/gnuplot-iostream.h"
#include "random"
#include "../include/ExtendedKarmanFilter.h"

//kf

int main() {
    // 创建卡尔曼滤波器对象
    Eigen::VectorXd initial_x(2);
    //状态估计向量（反映当前信息）
    initial_x << 0.0, 0.0;
    //估计误差协方差矩阵（反映估计的不确定性）
    //（对每一次观测结果的置信度）
    Eigen::MatrixXd initial_P(2, 2);
    initial_P << 1.0, 0.0,
                 0.0, 1.0;

    //调节Q 越大越接近测量值（误差越小）
    //     越小越远离测量值（误差越大）
    Eigen::MatrixXd process_noise(2, 2);
    process_noise << 0.1, 0.0,
                     0.0, 0.1;

    //调节R 越大越远离测量值（误差越大）
    //     越小越接近测量值（误差越小）
    Eigen::MatrixXd measurement_noise(1, 1);
    measurement_noise << 0.1;

    //系统的动力学模型
    Eigen::MatrixXd state_transition(2, 2);
    state_transition << 1.0, 0.1,
                        0.0, 1.0;

    //观测矩阵M*N M为测量向量   N为状态向量维度
    Eigen::MatrixXd observation(1, 2);
    observation << 1.0, 0.0;

    KalmanFilter filter(initial_x, initial_P, process_noise, measurement_noise, state_transition, observation);

    Eigen::VectorXd measurements(5);
//    measurements << 1.2,1.4,1.6,1.8,2.0;

    std::vector<double> x_values ;
    std::vector<double> y1_values;
    std::vector<double> y2_values;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-0.5,0.5);

    for(float i = 0;i < /*measurements.size()*/ 10;i+=0.1)
    {
        filter.predict();
        Eigen::VectorXd measurement(1);
//        measurement <<measurements(i);
        double noise=dis(gen);
        measurement <<sin(i)+noise;
        filter.update(measurement);

        Eigen::VectorXd filtered_state = filter.getState();
        std::cout << "Measurement: " << measurement(0) << ", Filtered State: " << filtered_state.transpose() << std::endl;
        x_values.push_back(i);
        y1_values.push_back(measurement(0));
        y2_values.push_back(filtered_state(0));
    }

    Gnuplot gp;
    gp<<"set terminal png\n";
    gp<<"set output 'line_plot.png'\n";
    gp<<"plot '-' with lines title 'Data 1','-' with lines title 'Data 2'\n";
    gp.send1d(boost::make_tuple(x_values,y1_values));

//    gp<<"plot '-' with lines title 'Data 2'\n";
    gp.send1d(boost::make_tuple(x_values,y2_values));

//    std::cin.get();
    return 0;
}

//ekf
//int main()
//{
//    ExtendedKarmanFilter ekf;
//
//    MatrixXd Q(3,3);
//    Q<<0.1,0,0,
//       0,0.1,0,
//       0,0,0.1;
//    ekf.setProcessNoiseCovariance(Q);
//    // 设置初始状态
//    VectorXd x0(3);
//    x0 << 1, 1, 1;
//    ekf.setInitialState(x0);
//
//
//    // 测量值
//    VectorXd z(2);
//    z << 2.5, 0.7;
//
//    // 预测和更新
//    ekf.predict();
//    ekf.update(z);
//
//    // 获取估计状态和协方差矩阵
//    VectorXd state = ekf.getState();
//    MatrixXd covariance = ekf.getCovariance();
//
//    // 打印结果
//    cout << "Estimated state: " << state.transpose() << endl;
//    cout << "Covariance matrix: " << endl << covariance << endl;
//
//    return 0;
//}