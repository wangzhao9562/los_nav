#include <los_nav/los_nav.h>

namespace los_nav{
    LosNav::LosNav(double kp, double kd, double ki) : 
        base_controller_(nullptr), is_init_(false)
    {
        pid_param_.kp_ = kp;
        pid_param_.kd_ = kd;
        pid_param_.ki_ = ki;

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
        base_controller_ = new PFLosController(pid_param_, stop_tolerance);
        is_init_ = true;
        return true;
    }

    bool LosNav::initialize(double s_x, double s_y, double k, double b, bool is_reverse, double stop_tolerance){
        mission_type_ = MissionType::C_LINE;
        if(base_controller_){
            delete base_controller_;
            is_init_ = false;
        }
        /* Reserved */
        is_init_ = true;
        return true;
    }

    bool LosNav::initialize(double o_x, double o_y, double radius, double stop_tolerance){
        mission_type_ = MissionType::CIRCLE;
        if(base_controller_){
            delete base_controller_;
            is_init_ = false;
        }
        /* Reserved */
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