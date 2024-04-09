/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2024-03-23 21:56:07
 * @LastEditors: moxiaosang_vec moxiaosang_vec@163.com
 * @LastEditTime: 2024-04-09 22:23:55
 * @FilePath: /IW_CAL/include/types.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _TYPES_H_
#define _TYPES_H_

#include <deque>
#include <Eigen/Core>

#define PI 3.14159265358979323846
#define DEG2RAD(x) ((x) * PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / PI)

typedef enum frame_type_t {
    FRAME_TYPE_UNKNOWN = 0x00,
    FRAME_TYPE_FRD = 0x01,
    FRAME_TYPE_RFU = 0x02,
    FRAME_TYPE_FLU = 0x03,

    FRAME_TYPE_NED = 0x11,
    FRAME_TYPE_ENU = 0x12,
    FRAME_TYPE_NWU = 0x13,
    
    FRAME_TYPE_END = 0xFF,
} frame_type_t;


typedef struct veh_status_t
{
    double cur_time_stamp = 0;
    int status = 0;
    double duration = 0.0;
} veh_status_t;

typedef struct veh_statuses_t
{
    double cur_time_stamp = 0;
    veh_status_t move_stat;         /* 1: moving, 2: stopped */
    veh_status_t straight_curve;    /* 1: straight, 2: curve */
    veh_status_t forward_backward;  /* 1: forward, 2: backward */
    veh_status_t height_up_down;    /* 1: up, 2: down */
    veh_status_t speed_up_down;     /* 1: speed up, 2: speed down */
    veh_status_t deadzone_in_out;   /* 1: in, 2: out */
} veh_statuses_t;

typedef struct imu_t
{
    /* data */
    double time_stamp = 0;
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
} imu_t;

typedef struct wvel_t
{
    double time_stamp = 0.0;
    Eigen::Vector4d vel = Eigen::Vector4d::Zero();
    int gear = 0;
} wvel_t;

typedef struct wpul_t
{
    double time_stamp = 0.0;
    Eigen::Vector4i pul = Eigen::Vector4i::Zero();
    Eigen::Vector4i dir = Eigen::Vector4i::Zero();
} wpul_t;


typedef struct imu_derived_t
{
    double time_stamp = 0;
    Eigen::Vector3d ang_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d lin_acc = Eigen::Vector3d::Zero(); /* specific force acceleration */

    bool is_valid = false;
    bool use_gyro_bias = false;
    bool use_acc_bias = false;
    bool use_ext_calib = false;
} imu_derived_t;


typedef struct wvel_derived_t
{
    double cur_time_stamp = 0;
    Eigen::Vector4d vel = Eigen::Vector4d::Zero();
    Eigen::Vector4d vel_w = Eigen::Vector4d::Zero();
    double vel_rc = 0.0;
    double vel_rc_w = 0.0;
    bool vel_valid = false;

    double acc_time_stamp = 0;
    Eigen::Vector4d acc = Eigen::Vector4d::Zero();
    Eigen::Vector4d acc_w = Eigen::Vector4d::Zero();
    bool acc_valid = false;

    double lin_acc_time_stamp = 0;
    Eigen::Vector3d lin_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d lin_acc_w = Eigen::Vector3d::Zero();
    bool lin_acc_valid = false;

    double ang_vel_time_stamp = 0;
    Eigen::Vector3d ang_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d ang_vel_w = Eigen::Vector3d::Zero();
    bool ang_vel_valid = false;

    int gear = 0;
    
    bool is_valid = false;
} wvel_derived_t;

typedef struct wpul_derived_t
{
    double cur_time_stamp = 0;

    double vel_time_stamp = 0;
    Eigen::Vector4d vel = Eigen::Vector4d::Zero();
    Eigen::Vector4d vel_w = Eigen::Vector4d::Zero();
    double vel_rc = 0.0;
    double vel_rc_w = 0.0;
    bool vel_valid = false;

    double acc_time_stamp = 0;
    Eigen::Vector4d acc = Eigen::Vector4d::Zero();
    Eigen::Vector4d acc_w = Eigen::Vector4d::Zero();
    bool acc_valid = false;

    double lin_acc_time_stamp = 0;
    Eigen::Vector3d lin_acc = Eigen::Vector3d::Zero();  /* rear axis center */
    Eigen::Vector3d lin_acc_w = Eigen::Vector3d::Zero();
    bool lin_acc_valid = false;

    double ang_vel_time_stamp = 0;
    Eigen::Vector3d ang_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d ang_vel_w = Eigen::Vector3d::Zero();
    bool ang_vel_valid = false;
    
    bool is_valid = false;
} wpul_derived_t;

typedef struct gravity_obs_t
{
    double cur_time_stamp = 0;
    double att_time_stamp = 0;
    Eigen::Vector3d att = Eigen::Vector3d::Zero();
    Eigen::Vector3d att_w = Eigen::Vector3d::Zero();
    bool att_valid = false;
    bool use_gyro_bias = false;
    bool use_acc_bias = false;
} gravity_obs_t;

typedef struct container_t
{
    double cur_time_stamp = 0;
    std::deque<imu_t> imus;
    std::deque<wvel_t> wvels;
    std::deque<wpul_t> wpuls;
} container_t;


typedef struct imu_inpara_t
{
    double cur_time_stamp = 0;
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_bias_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_scale = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_scale = Eigen::Vector3d::Zero();
    Eigen::Matrix3d non_orth_mat = Eigen::Matrix3d::Identity();
    bool gyro_bias_valid = false;
    bool acc_bias_valid = false;
    bool gyro_scale_valid = false;
    bool acc_scale_valid = false;
    bool non_orth_mat_valid = false;
    double gyro_bias_time_stamp = 0;
    double acc_bias_time_stamp = 0;
    double gyro_scale_time_stamp = 0;
    double acc_scale_time_stamp = 0;
    double non_orth_mat_time_stamp = 0;

} imu_inpara_t;

/* config para, donnot change in runtime, set in outside */
typedef struct config_para_t
{
    int imu_sample_rate;
    int wvel_sample_rate;
    int wpul_sample_rate;
    frame_type_t imu_frame_type;
    frame_type_t veh_frame_type;
    frame_type_t nav_frame_type;
    Eigen::Matrix4d t_imu_over_veh;
    Eigen::Vector4d wvel_scale;
    Eigen::Vector4d wpul_scale;
    double front_width; 
    double rear_width;
    double axle_length;
} config_para_t;

/* algorithm para, donnot change in runtime */
typedef struct alg_para_t
{
    double containter_duration;
    double smooth_duration;
    size_t imu_container_size;
    size_t wvel_container_size;
    size_t wpul_container_size;
    size_t filter_state_size;
} alg_para_t;

/* calibration para, can change in runtime */
typedef struct calib_para_t
{
    double imu_time_offset;
    double wvel_time_offset;
    double wpul_time_offset;
    imu_inpara_t imu_inpara;
} calib_para_t;

typedef struct param_t
{
    config_para_t config_para;
    alg_para_t algorithm_para;
    calib_para_t calib_para;
} param_t;

typedef struct record_t
{
    double cur_time_stamp = 0.0;
    bool collect_enough_flg = false;   
    bool init_flg = false;
    veh_statuses_t veh_statuses;
    imu_derived_t imu_derived_prev;
    imu_derived_t imu_derived_curr;
    wvel_derived_t wvel_derived;
    wpul_derived_t wpul_derived;
    gravity_obs_t gravity_obs;

} record_t;

typedef struct nav_t
{
    double cur_time_stamp;
    int status;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Vector3d att;
    Eigen::Vector3d omg;
    Eigen::Vector3d wcc;
} nav_t;

typedef struct filter_state_t
{
    double cur_time_stamp = 0.0;
    Eigen::VectorXd state;
    Eigen::MatrixXd cov;
} filter_state_t;

#endif 