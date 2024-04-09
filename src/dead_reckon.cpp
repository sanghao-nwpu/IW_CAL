/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2024-03-23 23:11:19
 * @LastEditors: moxiaosang_vec moxiaosang_vec@163.com
 * @LastEditTime: 2024-04-09 23:22:09
 * @FilePath: /IW_CAL/src/dead_reckon.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "types.h"

#include "dead_reckon.h"
#include "rotation.h"

void DeadReckon::init_nav_and_filter()
{
    /* 1. variable declaration and definition */
    /* 1.1. declare variables */
    double cur_time_stamp;
    Eigen::VectorXd X0;
    Eigen::VectorXd w0;
    Eigen::MatrixXd P0;
    size_t state_size;

    /* 1.2 define variables */
    cur_time_stamp = cur_time_stamp_;
    state_size = param_.algorithm_para.filter_state_size;
    X0 = Eigen::VectorXd::Zero(state_size);
    w0 = Eigen::VectorXd::Zero(state_size);
    P0 = Eigen::MatrixXd::Zero(state_size, state_size);

    /* 2. check initialization condition */
    if (!record_.wvel_derived.vel_valid || !record_.gravity_obs.att_valid)
    {
        return ;
    }

    /* 3. initialize nav */
    nav_.cur_time_stamp = cur_time_stamp_;
    nav_.pos.setZero();
    nav_.vel.setZero();
    nav_.acc.setZero();
    nav_.att.setZero();
    nav_.omg.setZero();
    nav_.wcc.setZero();


    if (record_.wvel_derived.vel_valid)
    {
        nav_.vel(0) = 0.5 * (record_.wvel_derived.vel(2) + record_.wvel_derived.vel(3));
    }
    if (record_.wvel_derived.lin_acc_valid)
    {
        nav_.acc = record_.wvel_derived.lin_acc;
    }
    if (record_.gravity_obs.att_valid)
    {
        nav_.att = record_.gravity_obs.att;
    }
    if (record_.imu_derived_curr.is_valid && record_.imu_derived_curr.use_gyro_bias)
    {
        nav_.omg = record_.imu_derived_curr.ang_vel;
    } 

    /* 4. initialize filter */
    X0.block(0, 0, 3, 1) = nav_.vel;
    X0.block(3, 0, 3, 1) = nav_.att;
    X0.block(6, 0, 3, 1) = Eigen::Vector3d::Zero();
    /* velocity init cov */
    w0.block(0, 0, 3, 1) = 0.5 * Eigen::VectorXd::Ones(3);
    if (record_.wvel_derived.vel_valid)
    {
        w0(0) = 0.5 * sqrt(pow(record_.wvel_derived.vel_w(2), 2) + pow(record_.wvel_derived.vel_w(3), 2));
    }
    /* attitude init cov */
    w0.block(3, 0, 3, 1) = DEG2RAD(3.0) * Eigen::VectorXd::Ones(3);
    if (record_.gravity_obs.att_valid)
    {
        w0.block(3, 0, 3, 1) = record_.gravity_obs.att_w;
    }
    /* acc bias init cov */
    w0.block(6, 0, 3, 1) = 0.2 * Eigen::VectorXd::Ones(3);
    /* filter init cov */
    P0 = w0.cwiseProduct(w0).asDiagonal();
    filter_.init(cur_time_stamp, X0, P0);

    /* 5. set filter init flag */
    record_.init_flg = true;

    return ;
}

bool DeadReckon::check_containter_enough()
{
    /* 1. check container size */

    if (container_.imus.size() < param_.algorithm_para.imu_container_size)
    {
        return false;
    }
    if (container_.wvels.size() < param_.algorithm_para.wvel_container_size)
    {
        return false;
    }
    if (container_.wpuls.size() < param_.algorithm_para.wpul_container_size)
    {
        return false;
    }

    return true;
}

void DeadReckon::caculate_imu_derived(imu_derived_t &imu_derived_curr)
{
    /* 1. variable declaration and definition */
    /* 1.1 declare variables */
    size_t size;
    imu_t imu;
    Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();

    /* 1.2 define variables */
    size = container_.imus.size();

    /* 1.3 initialize variables */
    imu_derived_curr.is_valid = false;
    imu_derived_curr.use_acc_bias = false;
    imu_derived_curr.use_gyro_bias = false;

    /* 2. check */
    if (size < 2)
        return;
    imu = container_.imus.back();

    /* 3. caculate imu derived data */
    if (param_.calib_para.imu_inpara.gyro_bias_valid)
    {
        gyro_bias = param_.calib_para.imu_inpara.gyro_bias;
        imu_derived_curr.use_gyro_bias = true;
    }
    if (param_.calib_para.imu_inpara.acc_bias_valid)
    {
        acc_bias = param_.calib_para.imu_inpara.acc_bias;
        imu_derived_curr.use_acc_bias = true;
    }
    imu_derived_curr.ang_vel = imu.gyro - gyro_bias;
    imu_derived_curr.lin_acc = imu.acc - acc_bias;
    imu_derived_curr.is_valid = true;
    imu_derived_curr.time_stamp = imu.time_stamp;

    return ;
}

void DeadReckon::caculate_wvel_derived(wvel_derived_t &wvel_derived)
{
    /* 1. variable declaration and definition */
    /* 1.1 declare variables */
    size_t size, smooth_size;
    wvel_t wvel;
    double cur_time_stamp;
    double delta_time;
    double vel_w;           /* vel_w is the variance of vel */
    int sign;
    double ave_axle_width;  /* ave_axle_width is the average of front and rear axle width */

    /* 1.2 define variables */
    sign = 0;
    if (record_.veh_statuses.forward_backward.status == 1)
    {
        sign = 1;
    }
    else if (record_.veh_statuses.forward_backward.status == 2)
    {
        sign = -1;
    }
    size = container_.wvels.size();
    smooth_size = (size_t)param_.algorithm_para.smooth_duration * param_.config_para.wvel_sample_rate;
    vel_w = 0.2;
    ave_axle_width = 0.5 * (param_.config_para.front_width + param_.config_para.rear_width);

    /* 1.3 initialize variables */
    wvel_derived.is_valid = false;
    wvel_derived.vel_valid = false;
    wvel_derived.acc_valid = false;
    wvel_derived.ang_vel_valid = false;
    wvel_derived.lin_acc_valid = false;
    /* 2. check */
    /* 2.1 check container size */
    if (size < 2)
        return;
    /* 2.2 check sign */
    if (sign == 0)
        return;

    /* 3. caculate wvel derived data */
    /* 3.1 init time_stamp */
    wvel = container_.wvels.back();
    cur_time_stamp = wvel.time_stamp;
    delta_time = cur_time_stamp - wvel_derived.cur_time_stamp;
    wvel_derived.cur_time_stamp = cur_time_stamp;

    /* 3.2 caculate ang vel */
    wvel_derived.ang_vel = Eigen::Vector3d::Zero();
    wvel_derived.ang_vel_w = Eigen::Vector3d::Zero();
    /* (rear left - rear right) / rear width  */
    wvel_derived.ang_vel(2) = (wvel_derived.vel(2) - wvel_derived.vel(3)) / param_.config_para.rear_width;
    wvel_derived.ang_vel_w(2) = vel_w * sqrt(2) / param_.config_para.rear_width;
    wvel_derived.ang_vel_valid = true;

    /* 3.3 caculate vel */
    wvel_derived.vel = sign * param_.config_para.wvel_scale * wvel.vel;
    wvel_derived.vel_w = vel_w * Eigen::Vector4d::Ones();
    // TODO: while stright line, front wheel vel may be used
    wvel_derived.vel_rc = 0.5 * (wvel_derived.vel(2) + wvel_derived.vel(3));
    wvel_derived.vel_rc_w = 0.5 * sqrt(pow(wvel_derived.vel_w(2), 2) + pow(wvel_derived.vel_w(3), 2));
    wvel_derived.vel_valid = true;

    /* 3.3 caculate acc */
    if (size >= smooth_size)
    {
        // TODO: ceres optimization may be used to get the acc
        delta_time = cur_time_stamp - container_.wvels[size - smooth_size].time_stamp;
        wvel_derived.acc = (wvel_derived.vel - container_.wvels[size - smooth_size].vel) / delta_time;
        wvel_derived.acc_w = vel_w * sqrt(2) / delta_time * Eigen::Vector3d::Ones();
        wvel_derived.acc_valid = true;
    }
    /* 3.4 caculate lin acc */
    if (size >= smooth_size)
    {
        delta_time = cur_time_stamp - container_.wvels[size - smooth_size].time_stamp;
        // TODO: while stright line, front wheel acc may be used
        wvel_derived.lin_acc(0) = 0.5 * (wvel_derived.acc(2) + wvel_derived.acc(3));
        wvel_derived.lin_acc(1) = wvel_derived.vel_rc * + wvel_derived.ang_vel(2);
        wvel_derived.lin_acc_w(0) = 0.5 * sqrt(pow(wvel_derived.acc_w(2), 2) + pow(wvel_derived.acc_w(3), 2));
        /* TODO: think the ang_vel is const， in fact, the result is lognormal distribution */
        wvel_derived.lin_acc_w(1) = wvel_derived.vel_rc_w * wvel_derived.ang_vel(2);
        wvel_derived.lin_acc_valid = true;
    }

    return ;
}

void DeadReckon::caculate_wpul_derived(wpul_derived_t &wpul_derived)
{
    /* 1. variable declaration and definition */
    /* 1.1 declare variables */
    size_t size;
    size_t smooth_size;
    wpul_t wpul;
    double cur_time_stamp;
    double delta_time;
    double pul_w;           /* pul_w is the variance of pul */
    int sign;
    double ave_axle_width;  /* ave_axle_width is the average of front and rear axle width */

    /* 1.2 define variables */
    sign = 0;
    if (record_.veh_statuses.forward_backward.status == 1)
    {
        sign = 1;
    }
    else if (record_.veh_statuses.forward_backward.status == 2)
    {
        sign = -1;
    }
    size = container_.wpuls.size();
    pul_w = 1;  /* pul_w is the variance of pul */
    ave_axle_width = 0.5 * (param_.config_para.front_width + param_.config_para.rear_width);

    /* 1.3 initialize variables */
    wpul_derived.is_valid = false;
    wpul_derived.vel_valid = false;
    wpul_derived.acc_valid = false;
    wpul_derived.ang_vel_valid = false;
    wpul_derived.lin_acc_valid = false;

    /* 2. check */
    /* 2.1 check container size */
    if (size < 2)
        return;
    /* 2.2 check sign */
    if (sign == 0)
        return;

    /* 3. caculate wpul derived data */
    /* 3.1 init time_stamp */
    wpul = container_.wpuls.back();
    cur_time_stamp = wpul.time_stamp;
    delta_time = cur_time_stamp - wpul_derived.cur_time_stamp;
    wpul_derived.cur_time_stamp = cur_time_stamp;

    /* 3.2 caculate ang vel */
    /* TODO: don't caculate ang vel */

    /* 3.3 caculate vel */
    if (size >= smooth_size)
    {
        delta_time = cur_time_stamp - container_.wpuls[size - smooth_size].time_stamp;
        wpul_derived.vel = sign * param_.config_para.wpul_scale / delta_time *
                           (wpul.pul - container_.wpuls[size - smooth_size].pul);
        wpul_derived.vel_w = pul_w * sqrt(2) / delta_time * Eigen::Vector3d::Ones();
    }
    wpul_derived.vel_rc = 0.5 * (wpul_derived.vel(2) + wpul_derived.vel(3));
    wpul_derived.vel_rc_w = 0.5 * sqrt(pow(wpul_derived.vel_w(2), 2) + pow(wpul_derived.vel_w(3), 2));
    wpul_derived.vel_valid = true;

    /* 3.3 caculate acc */
    if (size >= 2)
    {
        // TODO: calculate acc
    }
    /* 3.4 caculate lin acc */
    if (size >= 2)
    {
        // TODO: calculate lin acc
    }

    return ;
}

void DeadReckon::recur_by_imu_derived(const imu_derived_t &imu_derived_prev,
                                      const imu_derived_t &imu_derived_curr)
{ 
    /* 1. variable declaration and definition */
    /* 1.1 declare variables */
    Eigen::Vector3d acc_pre = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_cur = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_pre = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_cur = Eigen::Vector3d::Zero();

}



void DeadReckon::set_config_para(const config_para_t &config_para)
{
    /* 1. config para */
    param_.config_para = config_para;

    /* 2. algorithm para */
    param_.algorithm_para.filter_state_size = 9;    /* vel, att, acc bias */
    param_.algorithm_para.containter_duration = 6.0;
    param_.algorithm_para.smooth_duration = 0.5;
    param_.algorithm_para.imu_container_size =
        size_t(param_.config_para.imu_sample_rate * param_.algorithm_para.containter_duration);
    param_.algorithm_para.wvel_container_size =
        size_t(param_.config_para.wvel_sample_rate * param_.algorithm_para.containter_duration);
    param_.algorithm_para.wpul_container_size =
        size_t(param_.config_para.wpul_sample_rate * param_.algorithm_para.containter_duration);
    
    /* 3. calibration para */
    param_.calib_para.imu_inpara.acc_bias_valid = false;
    param_.calib_para.imu_inpara.gyro_bias_valid = false;    
    param_.calib_para.imu_inpara.acc_scale_valid = false;
    param_.calib_para.imu_inpara.gyro_scale_valid = false;
    param_.calib_para.imu_inpara.non_orth_mat_valid = false;
    
    param_.calib_para.imu_time_offset = 0.0;
    param_.calib_para.wvel_time_offset = 0.0;
    param_.calib_para.wpul_time_offset = 0.0;

}


void DeadReckon::process_imu(const imu_t &imu_data) 
{
    /* 1. variable declaration and definition */
    /* 1.1 declare variables */
    imu_t imu;
    size_t size;
    Eigen::VectorXd X;
    Eigen::MatrixXd P;
    Eigen::VectorXd w;
    Eigen::MatrixXd F, Q;
    size_t state_size;
    Eigen::Vector3d X_att, X_ba;
    Eigen::Matrix3d X_R;

    /* 1.2 define variables */
    imu = imu_data;

    /* 2. check */
    /* 2.1 check container size */
    if (!record_.collect_enough_flg)
    {
        container_.imus.push_back(imu);
        /* just collect enough data , still return */
        record_.collect_enough_flg = check_containter_enough();
        return;
    }
    else
    {
        /* donnot need to check container size again */
        container_.imus.pop_front();
        container_.imus.push_back(imu);
    }

    /* 2.2 check initialization */
    if (!record_.init_flg)
    {
        /* flg is cacualted in init_nav_and_filter */
        init_nav_and_filter();
        return;
    }
    /* 3. process data */
    /* 3.1 caculate imu derived data */
    caculate_imu_derived(record_.imu_derived_curr);
    /* 3.2 process nominal state, recur  */
    if (record_.imu_derived_prev.is_valid)
    {
        recur_by_imu_derived(record_.imu_derived_prev, record_.imu_derived_curr);
    }
    /* 3.3 process error state, filter */
    /* 3.3.1 caculate F, Q */
    state_size = param_.algorithm_para.filter_state_size;
    X_att = X.block(0, 0, 3, 1);
    X_ba = X.block(6, 0, 3, 1);
    X_R = Rotation::euler2rotmat(X_att);
    F = Eigen::MatrixXd::Zero(state_size, state_size);
    /* delta theta relative to delta theta */
    F.block(0, 0, 3, 3) = -Rotation::vec2skewsmt(record_.imu_derived_curr.ang_vel);
    /* delta vel relative to delta theta */
    F.block(3, 0, 3, 3) = -X_R * Rotation::vec2skewsmt(record_.imu_derived_curr.lin_acc);
    /* delta vel relative to delta b_a */
    F.block(3, 6, 3, 3) = -X_R;
    w = Eigen::VectorXd::Zero(state_size);
    w.block(0, 0, 3, 1) = DEG2RAD(0.2) * Eigen::Vector3d::Ones();
    w.block(3, 0, 3, 1) = 0.2 * Eigen::Vector3d::Ones();
    w.block(6, 0, 3, 1) = 0.02 * Eigen::Vector3d::Ones();
    Q = w.cwiseProduct(w).asDiagonal();
    
    /* 3.3.2 update state, error predict is zero */
    X = filter_state_.state;
    X = F * X;  /* in fact, error state is zero, the predict is zero, too*/
    filter_state_.state = X;
    /* 3.3.3 update covariance */
    P = filter_state_.cov;
    P = F * P * F.transpose() + Q;
    filter_state_.cov = P;

    /* 4. update record */
    /* 4.1 update time stamp */
    record_.cur_time_stamp = imu.time_stamp;
    /* 4.2 update imu derived data */
    record_.imu_derived_prev = record_.imu_derived_curr;

    return ;
}



void DeadReckon::process_wvel(const wvel_t &wvel_data)
{
    /* 1. variable declaration and definition */
    /* 1.1 declare variables */
    wvel_t wvel;         
    size_t size;         
    Eigen::Vector3d vel = Eigen::Vector3d::Zero(); 
    Eigen::Vector3d vel_w = Eigen::Vector3d::Zero(); 
    Eigen::VectorXd Z;
    Eigen::MatrixXd R; 

    /* 1.2 define variables */
    wvel = wvel_data;  

    /* 2. check */
    /* 2.1 check container size */
    if (!record_.collect_enough_flg)
    {
        container_.wvels.push_back(wvel);
        /* just collect enough data , still return */
        record_.collect_enough_flg = check_containter_enough();
        return;
    }
    else
    {
        /* donnot need to check container size again */
        container_.wvels.pop_front();
        container_.wvels.push_back(wvel);
    }

    /* 2.2 check initialization */
    if (!record_.init_flg)
    {
        init_nav_and_filter();
        record_.init_flg = true;
        return;
    }
    /* 3. process data */
    /* 3.1 caculate wvel derived data */
    caculate_wvel_derived(record_.wvel_derived);

    /* 3.2 update filter */
    if (record_.wvel_derived.is_valid && record_.wvel_derived.vel_valid)
    {
        /* only use back axis ceneter velocity */
        vel(0) = 0.5 * (record_.wvel_derived.vel(2) + record_.wvel_derived.vel(3));
        vel_w(0) = 0.5 * sqrt(pow(record_.wvel_derived.vel_w(2), 2) + pow(record_.wvel_derived.vel_w(3), 2));
        Z = vel;
        R = vel_w.cwiseProduct(vel_w);
        filter_.update(Z);
    }

    /* 4. update record */
    /* 4.1 update time stamp */
    record_.cur_time_stamp = wvel.time_stamp;

    return ;
}


void DeadReckon::process_wpul(const wpul_t &wpul_data)
{
    /* 1. variable declaration and definition */
    /* 1.1 declare variables */
    double cur_time_stamp;
    wpul_t wpul;         
    size_t size;         
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();  
    Eigen::Vector3d acc = Eigen::Vector3d::Zero(); 
    Eigen::Vector3d vel_w = Eigen::Vector3d::Zero(); 
    Eigen::Vector3d acc_w = Eigen::Vector3d::Zero(); 
    Eigen::VectorXd Z;
    Eigen::MatrixXd R; 

    /* 1.2 define variables */
    wpul = wpul_data;  
    cur_time_stamp = wpul.time_stamp;

    /* 2. check */
    /* 2.1 check container size */
    if (!record_.collect_enough_flg)
    {
        container_.wpuls.push_back(wpul);
        /* just collect enough data , still return */
        record_.collect_enough_flg = check_containter_enough();
        return;
    }
    else
    {
        /* donnot need to check container size again */
        container_.wpuls.pop_front();
        container_.wpuls.push_back(wpul);
    }

    /* 2.2 check initialization */
    if (!record_.init_flg)
    {
        init_nav_and_filter();
        record_.init_flg = true;
        return;
    }

    /* 3. process data */
    /* 3.1 caculate wpul derived data */
    caculate_wpul_derived(record_.wpul_derived);

    /* 3.2 update filter */
    if (record_.wpul_derived.is_valid && record_.wpul_derived.lin_acc_valid && record_.wpul_derived.vel_valid)
    {
        /* only use back axis ceneter acceleration */
        vel(0) = 0.5 * (record_.wpul_derived.acc(2) + record_.wpul_derived.acc(3));
        vel_w(0) = 0.5 * sqrt(pow(record_.wpul_derived.acc_w(2), 2) + pow(record_.wpul_derived.acc_w(3), 2));
        acc(0) = 0.5 * (record_.wpul_derived.acc(2) + record_.wpul_derived.acc(3));
        acc_w(0) = 0.5 * sqrt(pow(record_.wpul_derived.acc_w(2), 2) + pow(record_.wpul_derived.acc_w(3), 2));
        /* compensate for linear velocity */
        Z = vel + acc * (cur_time_stamp - record_.wpul_derived.vel_time_stamp);
        vel_w = vel_w + acc_w * (cur_time_stamp - record_.wpul_derived.vel_time_stamp);
        R = vel_w.cwiseProduct(vel_w);
        filter_.update(Z);
    }

    /* 4. update record */
    /* 4.1 update time stamp */
    record_.cur_time_stamp = wpul.time_stamp;

    return ;
}