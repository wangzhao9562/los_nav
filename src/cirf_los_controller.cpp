/*
 * @Author: Zhao Wang
 * @Date: 2020-05-14 11:17:22
 * @LastEditTime: 2020-05-30 12:36:35
 * @LastEditors: Please set LastEditors
 * @Description: Implementation of CirFLosController class for circle following
 * @FilePath: /los_nav/src/cirf_los_controller.cpp
 */
#include <los_nav/cirf_los_controller.h>
#include <iostream>

namespace los_nav{
    std::pair<double, int> CirFLosController::computeCtrlQuantity(double x, double y, double tx, double ty, double yaw){
        std::cout << "x: " << x << " y: " << y << " center x: " << circle_.origin_x_ << " center y: " << circle_.origin_y_ << " radius: " << circle_.r_<< std::endl;
        double ye = distance(x, y, circle_.origin_x_, circle_.origin_y_) - circle_.r_;
        double usv_xy = std::atan2(y - circle_.origin_y_, x - circle_.origin_x_);
        double dir_r = usv_xy - PI / 2;
        if(dir_r > PI){
            dir_r = dir_r - 2 * PI;
        }
        else if(dir_r < -PI){
            dir_r = dir_r + 2 * PI;
        }

        std::cout << "ye: " << ye << " dir_r: " << dir_r << " factor: " << factor_  << std::endl;

        double err_usv_pos = yaw - usv_xy;

        double ref_phi = -std::atan(ye / factor_);
        if((err_usv_pos > 0 && err_usv_pos < PI) || (err_usv_pos < -PI)){
            if(ref_phi < 0){
                ref_phi = -PI - ref_phi;
            }
            else{
                ref_phi = PI - ref_phi;
            }
        }
        det_phi_ = ref_phi - (yaw - dir_r);

        std::cout << "err_usv_pos: " << err_usv_pos << " ref_phi: " << ref_phi << " original det_phi: " << det_phi_ << std::endl;
        if(det_phi_ > PI){
            det_phi_ = det_phi_ - 2 * PI;
        }
        else if(det_phi_ < -PI){
            det_phi_ = det_phi_ + 2 * PI;
        }

        std::cout << "yaw: " << yaw << " det_phi_: " << det_phi_ << std::endl;

        // det_phi_ = det_phi_ / 64;

        if(std::abs(ye) < 0.8){
            det_phi_diff_ += det_phi_;
        }

        // det_phi_ = det_phi_ / 32;

        double r = this->los_ctrl_param_.kp_ * det_phi_ + this->los_ctrl_param_.ki_ * det_phi_diff_;
        return std::make_pair(r, 0);
    }
}; // end of ns

