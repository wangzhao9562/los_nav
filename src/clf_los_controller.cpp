/*
 * @Author: Zhao Wang
 * @Date: 2020-05-13 13:01:19
 * @LastEditTime: 2020-05-13 14:50:19
 * @LastEditors: Zhao Wang
 * @Description: Implementation of interface of CLFLosController class
 * @FilePath: /los_nav/src/clf_los_controller.cpp
 */
#include <los_nav/clf_los_controller.h>

namespace los_nav{
    std::pair<double, int> CLFLosController::computeCtrlQuantity(double x, double y, double tx, double ty, double yaw){
        double dist = distance(x, y, tx, ty);
        
        if(dist <= this->stop_tolerance_){
            return std::make_pair<double, int>(0.0, 1);
        }
        
        double target_y;
        if(line_.is_reverse_){
            target_y = line_.k_ * x + line_.b_ - this->los_ctrl_param_.dy_err_ * std::sqrt(line_.k_ * line_.k_ + 1);
        }
        else{
            target_y = line_.k_ * x + line_.b_ + this->los_ctrl_param_.dy_err_ * std::sqrt(line_.k_ * line_.k_ + 1);
        }

        double ref_dir = std::atan(line_.k_);
        double det_y = y - target_y;
        double ye = det_y * std::cos(ref_dir / 180 * PI);
        double ref_phi = -std::atan(ye / factor_) / PI * 180;
        
        if(line_.is_reverse_){
           if(ref_phi < 0){
               ref_phi = -180 - ref_phi;
           }
           else{
               ref_phi = 180 - ref_phi;
           }
        }

        det_phi_ = ref_phi - (yaw * 180 / PI - ref_dir);
        if(det_phi_ > 180){
            det_phi_ -= 360;
        }        
        if(det_phi_ < -180){
            det_phi_ += 360;
        }

        if(std::abs(ye) < 0.5){
            det_phi_diff_ += det_phi_;
        }

        double r = this->los_ctrl_param_.kp_ * det_phi_ + this->los_ctrl_param_.ki_ * det_phi_diff_;
        
        return std::make_pair(r, 0);
    }
}; // end of ns