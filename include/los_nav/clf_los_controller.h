/*
 * @Author: Zhao Wang
 * @Date: 2020-05-13 11:13:33
 * @LastEditTime: 2020-05-29 22:30:16
 * @LastEditors: Please set LastEditors
 * @Description: Definition of CLFLosController class
 * @FilePath: /los_nav/include/los_nav/slf_los_controller.h
 */
#ifndef CLF_LOS_CONTROLLER_H_
#define CLF_LOS_CONTROLLER_H_

#include <los_nav/base_los_controller.h>
#include <iostream>

namespace los_nav{
/**
 * @brief Common line follow LOS implementation
 */
class CLFLosController : public BaseLosController{
public:
    /**
     * @brief Constructor
     * @param LosCtrlParam pid_param PID parameters for LOS controller
     * @param Point Coordinate of target point
     */ 
    CLFLosController(LosCtrlParam pid_param, const CLine& line, double los_factor = 3.5, double stop_tolerance = 1.0) : 
        BaseLosController(pid_param, stop_tolerance), line_(line), factor_(los_factor){ std::cout << "In CLine Controller: " << factor_ << std::endl;}

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
    CLine line_;
    double factor_;
    double det_phi_, det_phi_diff_, det_phi_int_; 
}; // end of class
}; // end of ns

#endif