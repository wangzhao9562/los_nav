/*
 * @Author: Zhao Wang
 * @Date: 2020-05-07 
 * @LastEditTime: 2020-05-10 
 * @LastEditors: Zhao Wang
 * @Description: Definition of LosNav class
 * @FilePath: /los_nav/src/pf_los_controller.cpp
 */
#ifndef LOS_NAV_H_
#define LOS_NAV_H_

#include <los_nav/base_los_controller.h>
#include <los_nav/common.h>
#include <utility>

namespace los_nav{
class LosNav{
public:
    LosNav(double kp, double kd, double ki, double dx_err, double dy_err, double los_factor);
    ~LosNav();

    bool initialize(double stop_tolerance);
    bool initialize(const CLine& line, double los_factor,  double stop_tolerance);
    bool initialize(Circle circle, double los_factor, double stop_tolerance);

    bool isControllerAvailable()const{
        return is_init_;
    }

    MissionType getMissionType()const{
        return mission_type_;
    }

    /* Reserved interface to modify the mission type from outside */
    
    /**
     * @brief Compute controlling quantity
     * @param double x Coordination x of robot
     * @param double y Coordination y of robot
     * @param double tx Coordination x of target point
     * @param double ty Coordination y of target point
     * @param double yaw Euler angle yaw of robot
     * @return return controlling quantity and is stop the robot
     */
    std::pair<double, int> computeCtrlQuantity(double x, double y, double tx, double ty, double yaw);

private:
    MissionType mission_type_;
    LosCtrlParam los_ctrl_param_;
    BaseLosController* base_controller_;
    double los_factor_;
    bool is_init_;
}; // end of class
}; // end of ns

#endif