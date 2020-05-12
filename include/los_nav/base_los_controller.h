/*
 * @Author: your name
 * @Date: 2020-05-07 13:08:08
 * @LastEditTime: 2020-05-09 17:22:40
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: /los_nav/include/los_nav/base_los_controller.h
 */
#ifndef BASE_LOS_CONTROLLER_H_
#define BASE_LOS_CONTROLLER_H_

#include <utility>
#include <los_nav/common.h>

namespace los_nav{
/**
 * @brief Offer pid parameters for PID controller
 */
struct PIDParam{
    PIDParam(double kp = 0.8, double ki = 0.0, double kd = 0.0):kp_(kp), ki_(ki), kd_(kd){}
    PIDParam(const PIDParam& pid_param){
        kp_ = pid_param.kp_;
        ki_ = pid_param.ki_;
        kd_ = pid_param.kd_;
    }
    double kp_;
    double ki_;
    double kd_;
};

/**
 * @brief LOS implementation
 */
class BaseLosController{
public:
    BaseLosController(PIDParam pid_param, double stop_tolerance) : 
        pid_param_(pid_param), stop_tolerance_(stop_tolerance) {}
  
    /**
     * @brief Compute control quantity for control mission
     * @param double x Coordination x of robot
     * @param double y Coordination y of robot
     * @param double tx Coordination x of target point
     * @param double ty Coordination y of target point
     * @param double yaw The euler angle yaw of robot
     * @return Compute result
     */ 
    virtual std::pair<double, int> computeCtrlQuantity(double x, double y, double tx, double ty, double yaw) = 0;

protected:    
    PIDParam pid_param_;
    double ctrl_quat_;
    double stop_tolerance_;
}; // end of class
}; // end of ns

#endif