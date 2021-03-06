/*
 * @Author: Zhao Wang
 * @Date: 2020-05-07 
 * @LastEditTime: 2020-05-13
 * @LastEditors: Zhao Wang
 * @Description: Definition of PFLosController class
 * @FilePath: /los_nav/src/pf_los_controller.cpp
 */
#ifndef PF_LOS_CONTROLLER_H_
#define PF_LOS_CONTROLLER_H_

#include <los_nav/base_los_controller.h>

namespace los_nav{
/**
 * @brief Point follow LOS implementation
 */
class PFLosController : public BaseLosController{
using Point = std::pair<double, double>;
public:
    /**
     * @brief Constructor
     * @param LosCtrlParam pid_param PID parameters for LOS controller
     * @param Point Coordinate of target point
     */ 
    PFLosController(LosCtrlParam los_ctrl_param, double stop_tolerance) : 
        BaseLosController(los_ctrl_param, stop_tolerance){}

    /**
     * @brief Compute control quantity for point follow
     * @param double x Coordination x of robot
     * @param double y Coordination y of robot
     * @param double tx Coordination x of target point
     * @param double ty Coordination y of target point
     * @param double yaw The euler angle yaw of robot
     * @return Compute result
     */ 
    std::pair<double, int> computeCtrlQuantity(double x, double y, double tx, double ty, double yaw)override;

private:
    double det_yaw_; 
};
};

#endif