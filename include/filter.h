/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2024-03-23 23:26:16
 * @LastEditors: moxiaosang_vec moxiaosang_vec@163.com
 * @LastEditTime: 2024-04-06 22:13:00
 * @FilePath: /IW_CAL/include/filter.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _FILTER_H_
#define _FILTER_H_

#include <Eigen/Dense>

class KalmanFilter {

public:
    /* continuous system: dot_X = A(t) * X(t) + B(t) * u(t) + F(t) * w(t); w(t) ~ N(0, Q(t))*/
    /* discrete system: X_{k+1} = Phi_k * X_k + G_k * u_k + Gamma_k * w_k; w_k ~ N(0, Q(t) / delta_t) */
    /* observation continous: Z(t) = H(t) * X(t) + v(t); v(t) ~ N(0, R(t)); */
    /* observation discrete : Z_k = H_k * X_k + v_k; v_k ~ N(0, R(t) / delta_t); */
    /* Phi_k = expm(A(t) * delta_t) = I + A(t) * delta_t + 0.5 * A(t) * A(t) * delta_t^2 + ... */
    /* G_k = B(t) * delta_t */
    /* Gamma_k = F(t) * delta_t */

    KalmanFilter();

    void init(double t0, const Eigen::VectorXd& X0, const Eigen::MatrixXd& P0);

    void check_system_resonable();

    void check_observation_resonable();

    /* Set the system to be discrete time or continuous time */
    void set_discrete(bool discrete) { discrete_ = discrete; }; 
    
    void set_nonlinear_prediction_enabled(bool enabled) { nonlinear_prediction_ = enabled; };

    void set_A(const Eigen::MatrixXd& A) { A_ = A; };

    void set_B(const Eigen::MatrixXd& B) { B_ = B; };

    void set_F(const Eigen::MatrixXd& F) { F_ = F; };

    void set_Q(const Eigen::MatrixXd& Q) { Q_ = Q; };

    void set_u(const Eigen::VectorXd& u) { u_ = u; };

    void set_H(const Eigen::MatrixXd& H) { H_ = H; };

    void set_R(const Eigen::MatrixXd& R) { R_ = R; };

    void set_Z(const Eigen::VectorXd& Z) { Z_ = Z; };

    void set_delta_time(double delta_time) { delta_time_ = delta_time; };

    size_t state_size() { return X_.size(); };

    size_t observation_size() { return Z_.size(); };
    
    double time() { return time_stamp_; };

    Eigen::VectorXd state() { return X_; };

    Eigen::MatrixXd covariance() { return P_; };

public:

    void set_nonlinear_prediction(const Eigen::VectorXd& X) { X_ = X; };

    void predict();

    void update(const Eigen::VectorXd& Z);



private:

    // Matrices for computation
    Eigen::MatrixXd A_, B_, F_, Q_;
    Eigen::MatrixXd H_, R_;
    Eigen::MatrixXd K_;
    Eigen::VectorXd u_;
    Eigen::VectorXd Z_;

    // Estimated states
    Eigen::VectorXd X_;
    Eigen::MatrixXd P_;

    // Initial and current time
    double time_stamp_ = 0.0;

    // Discrete time step
    double delta_time_ = 0.0;

    // Is the filter initialized?
    bool initialized_ = false;

    // Is the system resonable?
    bool system_resonable_ = false;

    // Is the observation resonable?
    bool observation_resonable_ = false;

    // Is the system discrete time?
    bool discrete_ = false;

    // Is the prediction step done?
    bool nonlinear_prediction_ = false;
};


#endif