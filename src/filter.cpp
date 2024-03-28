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

    X_ = A_ * X_ + B_ * u_;
    P_ = A_ * P_ * A_.transpose() + Q_;
}