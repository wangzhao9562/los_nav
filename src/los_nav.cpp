/*
 * @Author: Zhao Wang
 * @Date: 2020-05-07 
 * @LastEditTime: 2020-05-14 12:27:44
 * @LastEditors: Please set LastEditors
 * @Description: Implementation of interface of LosNav class
 * @FilePath: /los_nav/src/pf_los_controller.cpp
 */
#include <los_nav/los_nav.h>
#include <los_nav/pf_los_controller.h>
#include <los_nav/clf_los_controller.h>
#include <los_nav/cirf_los_controller.h>

namespace los_nav{
    LosNav::LosNav(double kp, double kd, double ki, 
        double dx_err, double dy_err, double los_factor) : 
        base_controller_(nullptr), los_factor_(los_factor), is_init_(false)
    {
        los_ctrl_param_.kp_ = kp;
        los_ctrl_param_.kd_ = kd;
        los_ctrl_param_.ki_ = ki;
        los_ctrl_param_.dx_err_ = dx_err;
        los_ctrl_param_.dy_err_ = dy_err;

        // mission_type_ = MissionType::NONE;
        mission_type_ = MissionType::POINT;
    }

    LosNav::~LosNav(){
        if(base_controller_){
            delete base_controller_;
            base_controller_ = nullptr;
        }
        is_init_ = false;
    }

    bool LosNav::initialize(double stop_tolerance){
        mission_type_ = MissionType::POINT;
        if(base_controller_){
            delete base_controller_;
            is_init_ = false;
        }
        base_controller_ = new PFLosController(los_ctrl_param_, stop_tolerance);
        is_init_ = true;
        return true;
    }

    bool LosNav::initialize(CLine line, double los_factor, double stop_tolerance){
        mission_type_ = MissionType::C_LINE;
        if(base_controller_){
            delete base_controller_;
            is_init_ = false;
        }
        /* Reserved for implementation*/
        base_controller_ = new CLFLosController(los_ctrl_param_, los_factor, stop_tolerance);
        is_init_ = true;
        return true;
    }

    bool LosNav::initialize(Circle circle, double los_factor, double stop_tolerance){
        mission_type_ = MissionType::CIRCLE;
        if(base_controller_){
            delete base_controller_;
            is_init_ = false;
        }
        /* Reserved for implementation*/
        base_controller_ = new CirFLosController(los_ctrl_param_, los_factor, stop_tolerance);
        is_init_ = true;
        return true;
    }
    
    std::pair<double, int> LosNav::computeCtrlQuantity(double x, double y, double tx, double ty, double yaw){
        if(base_controller_){
            if(mission_type_ != MissionType::NONE){
                return base_controller_->computeCtrlQuantity(x, y, tx, ty, yaw);
            }
            return std::make_pair<double, int>(0.0, -1);
        }
        return std::make_pair<double, int>(0.0, 1);
    }
}; // end of ns