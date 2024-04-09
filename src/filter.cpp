/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2024-03-28 23:22:59
 * @LastEditors: moxiaosang_vec moxiaosang_vec@163.com
 * @LastEditTime: 2024-04-05 21:36:58
 * @FilePath: /IW_CAL/src/filter.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
* Implementation of KalmanFilter class.
*
* @author: sanghao97 
* @date: 2024.03.26
*/


#include "filter.h"

KalmanFilter::KalmanFilter()
{
    // Initialize the state and covariance matrices
}

KalmanFilter::~KalmanFilter() {}


void KalmanFilter::init(double t0, const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0)
{
    // Initialize the state and covariance matrices
    time_stamp_ = t0;
    X_ = x0;
    P_ = P0;
    initialized_ = true;
}

void KalmanFilter::check_system_resonable()
{
    // Check if the filter is valid 
    if (delta_time_ <= 0)
    {
        system_resonable_ = false;
        return;
    }
    if (A_.rows()!= X_.size() || A_.rows()!= X_.size())
    {
        system_resonable_ = false;
        return;
    }
    if (B_.rows()!= X_.size() || B_.cols()!= u_.size())
    {
        system_resonable_ = false;
        return;
    }
    if (F_.rows()!= X_.cols() || F_.cols()!= Q_.rows())
    {
        system_resonable_ = false;
        return;
    }
    if (Q_.rows() != Q_.cols())
    {
        system_resonable_ = false;
        return;
    }
    system_resonable_ = true;
}

void KalmanFilter::check_observation_resonable()
{
    // Check if the observation is valid 
    if (H_.rows()!= Z_.size() || H_.cols()!= X_.size())
    {
        observation_resonable_ = false;
        return;
    }
    if (R_.rows() != Z_.size() || R_.cols() != Z_.size())
    {
        observation_resonable_ = false;
        return;
    }
    observation_resonable_ = true;
}


void KalmanFilter::predict()
{
    // Predict the state and covariance matrices
    Eigen::MatrixXd Phik, Gk, Gammak;
    Eigen::MatrixXd I;

    if (!initialized_ || !system_resonable_) return;
    I = Eigen::MatrixXd::Identity(X_.size(), X_.size());
    time_stamp_ += delta_time_;

    Phik = I + A_ * delta_time_;
    Gk = B_ * delta_time_;
    Gammak = F_ * delta_time_;

    if (!nonlinear_prediction_)
    {
        X_ = A_ * X_ + B_ * u_;
    }
    P_ = A_ * P_ * A_.transpose() + Q_;
}


void KalmanFilter::update(const Eigen::VectorXd& Z)
{
    // Update the state and covariance matrices with the observation
    Eigen::MatrixXd K, S, Ht, Vt;
    Eigen::VectorXd y;
    Eigen::MatrixXd I;

    I = Eigen::MatrixXd::Identity(X_.size(), X_.size());

    if (!initialized_ || !system_resonable_ || !observation_resonable_) return;


    y = Z - H_ * X_;
    S = H_ * P_ * H_.transpose() + R_;  /* Innovation covariance */
    K = P_ * H_.transpose() * S.inverse();  /* Kalman gain */

    X_ = X_ + K * y;
    P_ = (I - K * H_) * P_;
}
