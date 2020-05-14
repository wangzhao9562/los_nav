/*
 * @Author: Zhao Wang
 * @Date: 2020-05-14 11:17:22
 * @LastEditTime: 2020-05-14 12:32:15
 * @LastEditors: Please set LastEditors
 * @Description: Implementation of CirFLosController class for circle following
 * @FilePath: /los_nav/src/cirf_los_controller.cpp
 */
#include <los_nav/cirf_los_controller.h>

namespace los_nav{
    std::pair<double, int> CirFLosController::computeCtrlQuantity(double x, double y, double tx, double ty, double yaw){
        double ye = distance(x, y, circle_.origin_x_, circle_.origin_y_) - circle_.r_ + this->los_ctrl_param_.dy_err_;
        double usv_xy = std::atan2(y - circle_.origin_y_, x - circle_.origin_x_) / PI * 180;
        double dir_r = usv_xy - 90;
        if(dir_r > 180){
            dir_r = dir_r - 360;
        }
        else if(dir_r < -180){
            dir_r = dir_r + 360;
        }

        double err_usv_pos = yaw - usv_xy;

        double ref_phi = -std::atan(ye / factor_) / PI * 180;
        if((err_usv_pos > 0 && err_usv_pos < 180) || (err_usv_pos < -180)){
            if(ref_phi < 0){
                ref_phi = -180 - ref_phi;
            }
            else{
                ref_phi = 180 - ref_phi;
            }
        }
        det_phi_ = ref_phi - (yaw - dir_r);
        if(det_phi_ > 180){
            det_phi_ = det_phi_ - 360;
        }
        else if(det_phi_ < -180){
            det_phi_ = det_phi_ + 360;
        }

        if(std::abs(ye) < 0.8){
            det_phi_diff_ += det_phi_;
        }

        double r = this->los_ctrl_param_.kp_ * det_phi_ + this->los_ctrl_param_.ki_ * det_phi_diff_;
        return std::make_pair(r, 1);
    }
}; // end of ns

