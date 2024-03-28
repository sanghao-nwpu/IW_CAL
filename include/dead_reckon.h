/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2024-03-23 22:00:33
 * @LastEditors: moxiaosang_vec moxiaosang_vec@163.com
 * @LastEditTime: 2024-03-23 23:17:01
 * @FilePath: /IW_CAL/include/dead_reckon.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _DEAD_RECKON_H_
#define _DEAD_RECKON_H_

#include "types.h"
#include "filter.h"

class DeadReckon
{
private:
    double time_stamp_;
    KalmanFilter filter_;
    nav_t nav_;
    container_t container_;
    config_t config_;
    param_t param_;
    record_t record_;
public:

    void set_config(const config_t &config);
    void process_imu(const imu_t &imu);

    void process_wvel(const wvel_t &wvel);

    void process_wpul(const wpul_t &wpul);
   
};



#endif