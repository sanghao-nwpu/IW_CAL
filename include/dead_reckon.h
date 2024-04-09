/*
 * @Author: moxiaosang_vec moxiaosang_vec@163.com
 * @Date: 2024-03-23 22:00:33
 * @LastEditors: moxiaosang_vec moxiaosang_vec@163.com
 * @LastEditTime: 2024-04-09 22:25:09
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
    double cur_time_stamp_;
    KalmanFilter filter_;   /* process in class */
    filter_state_t filter_state_;   /* process directly in runtime */
    nav_t nav_;
    container_t container_;
    param_t param_;
    record_t record_;

private:

    void init_nav_and_filter();

    bool check_containter_enough();

    void caculate_imu_derived(imu_derived_t &imu_derived_curr);

    void caculate_wvel_derived(wvel_derived_t &wvel_derived);

    void caculate_wpul_derived(wpul_derived_t &wpul_derived);

    void recur_by_imu_derived(const imu_derived_t &imu_derived_prev,
                              const imu_derived_t &imu_derived_curr);

public:

    void set_config_para(const config_para_t &config_para);

    void process_imu(const imu_t &imu);

    void process_wvel(const wvel_t &wvel);

    void process_wpul(const wpul_t &wpul);
   
};



#endif