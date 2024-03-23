/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2024-03-23 21:56:07
 * @LastEditors: moxiaosang_vec moxiaosang_vec@163.com
 * @LastEditTime: 2024-03-23 23:09:35
 * @FilePath: /IW_CAL/include/types.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _TYPES_H_
#define _TYPES_H_

#include <deque>
#include <Eigen/Core>

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
    Eigen::Vector4d vels = Eigen::Vector4d::Zero();
    int gear = 0;
} wvel_t;

typedef struct wpul_t
{
    double time_stamp = 0.0;
    Eigen::Vector4i puls = Eigen::Vector4i::Zero();
    Eigen::Vector4i dirs = Eigen::Vector4i::Zero();
} wvel_t;

typedef struct container_t
{
    double cur_time_stamp = 0;
    std::deque<imu_t> imus;
    std::deque<wvel_t> wvels;
    std::deque<wpul_t> wpuls;
} container_t;

typedef struct config_t
{
    int imu_sample_rate;
    int wvel_sample_rate;
    int wpul_sample_rate;
    frame_type_t veh_frame_type;
    frame_type_t global_frame_type;
    Eigen::Matrix4d t_imu_over_veh;
    double wvel_scale[4];
    double wpul_scale[4];
} config_t;

typedef struct param_t
{
    double containter_duration;
    double smooth_duration;
} param_t;

typedef struct record_t
{
    double time_stamp;

} record_t;

typedef struct nav_t
{
    double time_stamp;
    int status;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Vector3d att;
    Eigen::Vector3d omg;
    Eigen::Vector3d wcc;
} nav_t;


#endif 