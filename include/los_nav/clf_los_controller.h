/*
 * @Author: Zhao Wang
 * @Date: 2020-05-13 11:13:33
 * @LastEditTime: 2020-05-13 13:36:21
 * @LastEditors: Zhao Wang
 * @Description: Definition of CLFLosController class
 * @FilePath: /los_nav/include/los_nav/slf_los_controller.h
 */
#ifndef CLF_LOS_CONTROLLER_H_
#define CLF_LOS_CONTROLLER_H_

#include <los_nav/base_los_controller.h>

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
    CLFLosController(LosCtrlParam pid_param, double los_factor, double stop_tolerance) : 
        BaseLosController(pid_param, stop_tolerance), factor_(los_factor){}

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
    double det_phi_{0.0}, det_phi_diff_{0.0};    
}; // end of class
}; // end of ns

#endif