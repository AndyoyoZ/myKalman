#include "kalman.h"

int main()
{
    Kalman kalman;
    std::cout<<"start kalman filter---"<<std::endl;

    Eigen::Vector4d currentStateVector;//状态向量
    Eigen::Matrix4d transitionMatrix_F;//状态转移矩阵
    Eigen::Matrix4d contolMatrix_B = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d processNoiseCov_Q;//过程噪声协方差矩阵
    Eigen::Matrix<double,2,4> measurementMatrix_H;//观测矩阵
    Eigen::Matrix4d errorCov_Pk;//预测误差协方差矩阵
    Eigen::Matrix2d measurmentNoiseCov_R;//观测噪声协方差矩阵
    Eigen::Vector4d uk;//控制量
    Eigen::Vector2d zk;//观测量

    currentStateVector << 10,10,10,10;//状态变量:x,y,vx,vy

    transitionMatrix_F << 1,0,0.01,0,  //time step is 0.01
                          0,1,0,0.01,
                          0,0,1,0,
                          0,0,0,1;

    processNoiseCov_Q << 1e-5,0,0,0, //
                         0,1e-5,0,0,
                         0,0,1e-5,0,
                         0,0,0,1e-5;

    measurementMatrix_H << 0,0,1,0,
                           0,0,0,1;

    errorCov_Pk <<  1,0,0,0,
                    0,1,0,0,
                    0,0,1,0,
                    0,0,0,1;

    measurmentNoiseCov_R << 1e-1,0, 
                            0,1e-1;
                            

    //初始化
    kalman.initKalman(transitionMatrix_F,contolMatrix_B,processNoiseCov_Q,measurementMatrix_H,measurmentNoiseCov_R);

    uk << 0,0,0,0;
    zk << 10.0,20.0;
    kalman.kalmanFilter(uk,zk,currentStateVector,errorCov_Pk);
    std::cout<<"currentStateVector\n"<<currentStateVector<<std::endl;
    std::cout<<"measurment\n"<<zk<<std::endl;
    std::cout<<"newBestStateVector\n"<<kalman.newBestStateVector_Xk<<std::endl;
    
    return 0;
}