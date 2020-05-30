/*
 * @Author: Zhao Wang
 * @Date: 2020-05-07 15:03:37
 * @LastEditTime: 2020-05-29 21:53:05
 * @LastEditors: Please set LastEditors
 * @Description: Implementation of interface of PFLosController class
 * @FilePath: /los_nav/src/pf_los_controller.cpp
 */
#include <los_nav/pf_los_controller.h>

namespace los_nav{
    std::pair<double, int> PFLosController::computeCtrlQuantity(double x, double y, double tx, double ty, double yaw)
    {
        double dist = distance(x, y, tx, ty);
        
        if(dist <= this->stop_tolerance_){
            return std::make_pair<double, int>(0.0, 1);
        }

        double target_yaw = std::atan2(y - ty, x - tx);
        double det_yaw = yaw - target_yaw;

        if(std::abs(det_yaw) > PI){
            if(det_yaw > 0){
                det_yaw -= 2.0 * PI;
            }  
            else{
                det_yaw += 2.0 * PI;
            }
        }

        /*
        double target_yaw = slopeToAngle(y - ty, x - tx);
        double det_yaw = target_yaw - yaw;

        if(target_yaw * yaw < -90 * 90){
            det_yaw = 360 - std::abs(target_yaw) - std::abs(yaw);
            if(target_yaw > 0){
                det_yaw = -det_yaw;
            }
        }

        if(std::abs(det_yaw) > 180){
            if(det_yaw > 0){
                det_yaw -= 360; 
            }
            else{
                det_yaw += 360;
            }
        }
        */

        double r = this->los_ctrl_param_.kp_ * det_yaw;
        return std::make_pair(r, 0);
    }
};