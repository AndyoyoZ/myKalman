/*-------------------------------------------------------
		file:   kalman.h
		author:	Andyoyo
		date:	2018/5/8
-------------------------------------------------------*/
#ifndef _KALMAN_H_
#define _KALMAN_H_

#include <iostream>
#include <ctime>

#include <Eigen/Core>
#include <Eigen/Dense>

class Kalman{	
public:
	Kalman(){}//构造函数
	~Kalman(){}//析构函数
	//初始化Kalman参数
	void initKalman(Eigen::MatrixXd Matrix_F,Eigen::MatrixXd Matrix_B,Eigen::MatrixXd Matrix_Q,Eigen::MatrixXd Matrix_H,Eigen::MatrixXd Matrix_R);
	//Kalman
	void kalmanFilter(Eigen::VectorXd u,Eigen::VectorXd z,Eigen::VectorXd Vector_Xk,Eigen::MatrixXd Cov_Pk);
private:
	//公式1：预测下一时刻的状态向量
	void predictStateVector_Xk(Eigen::VectorXd Vector_Xk_1,Eigen::VectorXd Vector_Uk);
	//公式2：预测协方差矩阵
	void predictCov_Pk(Eigen::MatrixXd Cov_Pk_1);
	//公式3：计算卡尔曼增益
	void computeKalmanGain_K(Eigen::MatrixXd Cov_Pk);
	//公式4：更新状态向量
	void updateStateVector_Xk(Eigen::VectorXd Vector_Xk,Eigen::VectorXd Vector_Zk);
	//公式5：更新协方差矩阵
	void updateCov_Pk(Eigen::MatrixXd Cov_Pk);
public:
	Eigen::VectorXd newBestStateVector_Xk;//状态向量的最优估计
private:
	Eigen::MatrixXd KalmanGain_K;//卡尔曼增益
	Eigen::VectorXd currentStateVector_Xk;//当前时刻的状态向量
	Eigen::MatrixXd currentCov_Pk;//预测估计协方差矩阵

	//运动模型参数
	Eigen::MatrixXd F;//状态转移矩阵
	Eigen::MatrixXd B;//
	Eigen::MatrixXd Q;//过程噪声协方差矩阵
	Eigen::VectorXd u;//控制量

	//观测模型参数
	Eigen::MatrixXd H;//观测矩阵
	Eigen::MatrixXd R;//测量噪声协方差矩阵
	Eigen::VectorXd z;//观测量

};


#endif
