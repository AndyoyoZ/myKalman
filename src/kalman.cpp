/*-------------------------------------------------------
		file:   kalman.cpp
		author:	Andyoyo
		date:	2018/5/8
-------------------------------------------------------*/
#include "kalman.h"

//初始化Kalman参数
void Kalman::initKalman(Eigen::MatrixXd Matrix_F, Eigen::MatrixXd Matrix_B,
                        Eigen::MatrixXd Matrix_Q, Eigen::MatrixXd Matrix_H, Eigen::MatrixXd Matrix_R)
{
    F=Matrix_F;
    B=Matrix_B;
    Q=Matrix_Q;
    H=Matrix_H;
    R=Matrix_R;
}

//公式1：预测下一时刻的状态向量
void Kalman::predictStateVector_Xk(Eigen::VectorXd Vector_Xk_1,Eigen::VectorXd Vector_Uk)
{
    currentStateVector_Xk = F*Vector_Xk_1+B*Vector_Uk;
}
	
//公式2：预测协方差矩阵
void Kalman::predictCov_Pk(Eigen::MatrixXd Cov_Pk_1)
{
    currentCov_Pk=F*Cov_Pk_1*F.inverse()+Q;
}

//公式3：计算卡尔曼增益
void Kalman::computeKalmanGain_K(Eigen::MatrixXd Cov_Pk)
{  
    Eigen::MatrixXd transpose_H;
    transpose_H=H.transpose();
    Eigen::MatrixXd tempMatrix;
    tempMatrix=H*Cov_Pk*transpose_H+R;
    KalmanGain_K=Cov_Pk*transpose_H*tempMatrix.inverse();
    //std::cout<<"KalmanGain_K\n"<<KalmanGain_K<<std::endl;
}

//公式4：更新状态向量
void Kalman::updateStateVector_Xk(Eigen::VectorXd Vector_Xk,Eigen::VectorXd Vector_Zk)
{
    Eigen::VectorXd tempVector;
    tempVector=Vector_Zk-H*Vector_Xk;
    //std::cout<<"tempVector\n"<<tempVector<<std::endl;
    newBestStateVector_Xk=Vector_Xk+KalmanGain_K*(Vector_Zk-H*Vector_Xk);
}

//公式5：更新协方差矩阵
void Kalman::updateCov_Pk(Eigen::MatrixXd Cov_Pk)
{
    currentCov_Pk=Cov_Pk-KalmanGain_K*H*Cov_Pk;
}

	
void Kalman::kalmanFilter(Eigen::VectorXd u,Eigen::VectorXd z,Eigen::VectorXd Vector_Xk,Eigen::MatrixXd Cov_Pk)
{
    predictStateVector_Xk(Vector_Xk,u);
    predictCov_Pk(Cov_Pk);
    computeKalmanGain_K(currentCov_Pk);
    updateStateVector_Xk(currentStateVector_Xk,z);
    //std::cout<<"currentCov_Pk\n"<<currentCov_Pk<<std::endl;
    updateCov_Pk(currentCov_Pk);
    //std::cout<<"currentCov_Pk\n"<<currentCov_Pk<<std::endl;
}
