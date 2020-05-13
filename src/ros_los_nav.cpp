/*
 * @Author: Zhao Wang
 * @Date: 2020-05-11 
 * @LastEditTime: 2020-05-13
 * @LastEditors: Zhao Wang
 * @Description: Implementation of interface of RosLosNav class
 * @FilePath: /los_nav/src/clf_los_controller.cpp
 */
#include <los_nav/ros_los_nav.h>

namespace los_nav{
    RosLosNav::RosLosNav(tf::TransformListener& tf) : tf_(tf), performer_(nullptr), los_nav_as_(nullptr)
    {
        ROS_INFO("Start initialization");
        flock_ = PTHREAD_RWLOCK_INITIALIZER; // rw lock initialization
        ros::NodeHandle private_nh("~");
        ros::NodeHandle nh;
        ros::NodeHandle simple_nh("move_base_simple"); // 2d nav goal in rviz publish the parameter on topic move_base_simple/goal
        ros::NodeHandle action_nh("los_nav");

        // ros parameters setting
        private_nh.param("velocity", vel_, 0.5);
        private_nh.param("stop_tolerance", stop_tolerance_, 1.0);
        private_nh.param("transform_tolerance", transform_tolerance_, 0.2);
        private_nh.param("global_frame", global_frame_, std::string("odom"));
        private_nh.param("base_frame", base_frame_, std::string("base_link"));
        private_nh.param("control_frequency", control_frequency_, 5.0); // hz
        private_nh.param("kp", kp_, 0.8);
        private_nh.param("kd", kd_, 0.0);
        private_nh.param("ki", ki_, 0.0);
        private_nh.param("dx_err", dx_err_, 2.0);
        private_nh.param("dy_err", dy_err_, 4.0);
        private_nh.param("factor", los_factor_, 3.5);

        ROS_INFO_STREAM("Follow velocity: " << vel_);
        ROS_INFO_STREAM("Transform tolerance: " << stop_tolerance_);
        ROS_INFO_STREAM("global_frame: " << global_frame_);
        ROS_INFO_STREAM("base_frame: " << base_frame_);
        ROS_INFO_STREAM("control_frequency: " << control_frequency_);
        ROS_INFO_STREAM("PID parameters: " << "kp: " << kp_ << " kd: " << kd_ << " ki: " << ki_);

        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); // publish velocity command
        goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&RosLosNav::goalCb, this, _1));
        mission_type_sub_ = nh.subscribe<los_nav::Mission>("mission_type", 1, boost::bind(&RosLosNav::missionTypeCb, this, _1));
        action_goal_pub_ = action_nh.advertise<los_nav_msgs::LosNavActionGoal>("goal", 1);
        current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 1);

        vis_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

        los_nav_as_ = new LosNavActionServer(ros::NodeHandle(), "los_nav", boost::bind(&RosLosNav::executeCb, this, _1), false);

        ROS_INFO("Create control performer");
        performer_ = new LosNav(kp_, kd_, ki_, dx_err_, dy_err_, los_factor_);
        performer_->initialize(stop_tolerance_);

        ROS_INFO("Start actionlib server"); 
        los_nav_as_->start();

        ROS_INFO("Initialization done");
        /* Preserved for dynamic configuration initialization */
    }

    RosLosNav::~RosLosNav(){
        if(los_nav_as_){
            delete los_nav_as_;
        }

        if(performer_){
            delete performer_;
        }
    }

    void RosLosNav::executeCb(const los_nav_msgs::LosNavGoalConstPtr& los_nav_goal){
        ROS_INFO("Received goal");
        if(!isQuaternionValid(los_nav_goal->target_pose.pose.orientation)){
            los_nav_as_->setAborted(los_nav_msgs::LosNavResult(), 
                "Aborting on goal cus it was sent with an invalid quaternion!");
            return;
        }
        geometry_msgs::PoseStamped goal = goalToGlobalFrame(los_nav_goal->target_pose);

        current_goal_pub_.publish(goal);
        // visualization_msgs::Marker m = generateVisPoint(goal);
        // vis_pub_.publish(m); 
        // ROS_WARN("Marker publish done");

        ros::Rate r(control_frequency_);
        
        // performer_ = new LosNav(kp_, kd_, ki_);

        ros::NodeHandle n;

        while(n.ok()){
            ROS_INFO("In controlling loop");
            if(los_nav_as_->isPreemptRequested()){
                if(los_nav_as_->isNewGoalAvailable()){
                    ROS_INFO("Interrupt with new goal");
                    los_nav_msgs::LosNavGoal new_goal = *los_nav_as_->acceptNewGoal();
                    if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
                        los_nav_as_->setAborted(los_nav_msgs::LosNavResult(), 
                            "Aborting on goal cus it was sent with an invalid quaternion!");
                        return;
                    }
                    goal = goalToGlobalFrame(new_goal.target_pose);
                    current_goal_pub_.publish(goal);
                }
                else{
                    // resetState();
                    los_nav_as_->setPreempted();
                    return;
                }
            }

            if(goal.header.frame_id != global_frame_){
                goal = goalToGlobalFrame(goal);
                current_goal_pub_.publish(goal);
            }
       
            ROS_INFO("Start control");
            bool done = controlCycle(goal);
            if(done){
                return;
            }
            
            r.sleep();
        };
        los_nav_as_->setAborted(los_nav_msgs::LosNavResult(), 
            "Aborting on the goal cus the node has been killed!");

        return;
    }

    bool RosLosNav::controlCycle(const geometry_msgs::PoseStamped& goal){
        geometry_msgs::Twist cmd_vel;
        
        tf::Stamped<tf::Pose> global_pose;
        getRobotPose(global_pose);

        geometry_msgs::PoseStamped current_position;
        tf::poseStampedTFToMsg(global_pose, current_position);

        los_nav_msgs::LosNavFeedback feedback;
        feedback.base_position = current_position;
        los_nav_as_->publishFeedback(feedback);

        pthread_rwlock_rdlock(&flock_);
        if(performer_){
            if(performer_->isControllerAvailable()){
                ROS_INFO("Compute controlling quantity");
                std::pair<double, int> ctrl_fb = performer_->computeCtrlQuantity(
                    current_position.pose.position.x, current_position.pose.position.y,
                    goal.pose.position.x, goal.pose.position.y,
                    tf::getYaw(current_position.pose.orientation));
                pthread_rwlock_unlock(&flock_);
                
                if(ctrl_fb.second == 0){
                    cmd_vel.linear.x = vel_;
                    cmd_vel.angular.z = -ctrl_fb.first;
                    cmd_vel_pub_.publish(cmd_vel);
                    return false;
                }
                else if(ctrl_fb.second == 1){
                    ROS_INFO("Controlling done");
                    publishZeroVelocity();
                    los_nav_as_->setSucceeded(los_nav_msgs::LosNavResult(), "Goal reached!");
                    return true;
                }
                else{
                    publishZeroVelocity();
                    return false;
                }
            }
            else{
                ROS_INFO("Wait for the initialization of controller");
                return false;
            }
        }
        resetState(); 
        return false;
    }

    geometry_msgs::PoseStamped RosLosNav::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
        tf::Stamped<tf::Pose> goal_pose, global_pose;
        poseStampedMsgToTF(goal_pose_msg, goal_pose);

        goal_pose.stamp_ = ros::Time();
        
        try{
            tf_.transformPose(global_frame_, goal_pose, global_pose);
        }
        catch(tf::TransformException& ex){
            ROS_INFO("los_nav: Failed to transform the goal pose from %s into the %s frame: %s", 
                goal_pose.frame_id_.c_str(), global_frame_.c_str(), ex.what());
            return goal_pose_msg;
        }

        geometry_msgs::PoseStamped global_pose_msg;
        tf::poseStampedTFToMsg(global_pose, global_pose_msg);
        return global_pose_msg;
    }

    bool RosLosNav::isQuaternionValid(const geometry_msgs::Quaternion& q)const{
        if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
            ROS_ERROR("los_nav: Quaternion has nans or infs... discarding as a target goal!");
            return false;
        }
        tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
        if(tf_q.length2() < 1e-6){
            ROS_ERROR("los_nav: QUaternion has length close to zero... discarding as target goal!");
            return false;
        }
        tf_q.normalize();
        tf::Vector3 up(0, 0, 1);
        double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));
        if(fabs(dot - 1) > 1e-3){
            ROS_ERROR("los_nav: QUaternion is invalid... the z-axis of the quaternion must be closed to vertical!");
            return false;
        }
        return true;
    }

    void RosLosNav::goalCb(const geometry_msgs::PoseStamped::ConstPtr& goal){
        ROS_INFO("Transform received goal");
        los_nav_msgs::LosNavActionGoal action_goal;
        action_goal.header.stamp = ros::Time::now();
        action_goal.goal.target_pose = *goal;
        action_goal_pub_.publish(action_goal);
    }

    bool RosLosNav::getRobotPose(tf::Stamped<tf::Pose>& global_pose)const{
        global_pose.setIdentity();
        tf::Stamped<tf::Pose> robot_pose;
        robot_pose.setIdentity();
        robot_pose.frame_id_ = base_frame_;
        robot_pose.stamp_ = ros::Time();
        ros::Time current_time = ros::Time::now(); 

        try{
            tf_.transformPose(global_frame_, robot_pose, global_pose);
        } 
        catch(tf::LookupException& ex){
            ROS_ERROR_THROTTLE(1.0, "No transform availabale, error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch(tf::ConnectivityException& ex){
            ROS_ERROR_THROTTLE(1.0, "Connectivity error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch(tf::ExtrapolationException& ex){
            ROS_ERROR_THROTTLE(1.0, "Extrapolation error looking up robot pose: %s\n", ex.what());
            return false;
        }
        if(current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance_){
            ROS_WARN_THROTTLE(1.0, "LosNav transform timeout");
        }
    }

    void RosLosNav::resetState(){
        publishZeroVelocity();

        // reset controller
        pthread_rwlock_wrlock(&flock_);
        if(performer_){
            delete performer_;
        }
        performer_ = new LosNav(kp_, kd_, ki_, dx_err_, dy_err_, los_factor_);
        pthread_rwlock_unlock(&flock_);
    }

    void RosLosNav::publishZeroVelocity(){
        geometry_msgs::Twist vel;
        vel.linear.x = 0.0;
        vel.linear.y = 0.0;
        vel.linear.z = 0.0;
        vel.angular.x = 0.0;
        vel.angular.y = 0.0;
        vel.angular.z = 0.0;
        cmd_vel_pub_.publish(vel);
    }

     void RosLosNav::missionTypeCb(const los_nav::Mission::ConstPtr& type){
        pthread_rwlock_wrlock(&flock_);
        if(performer_){
            if(performer_->isControllerAvailable()){
                switch (type->mission_type)
                {
                case 0:
                    if(performer_->getMissionType() != MissionType::POINT){
                        performer_->initialize(stop_tolerance_);
                    }
                    break;
                case 1:
                    if(performer_->getMissionType() != MissionType::C_LINE){
                        CLine line{type->lines[0].start_x, type->lines[0].start_y,
                            type->lines[0].end_x, type->lines[0].end_y,
                            type->lines[0].k, type->lines[0].b, type->lines[0].is_reverse};
                        
                        performer_->initialize(line, los_factor_, stop_tolerance_);
                    }
                    break;
                case 2:
                    if(performer_->getMissionType() != MissionType::CIRCLE){
                    /* Preserved for implementation of controller initialization */
                    }
                default:
                    break;
                }
            }
        }
        else{
            resetState();
        }
        pthread_rwlock_unlock(&flock_);
     }
 
     visualization_msgs::Marker RosLosNav::generateVisPoint(const geometry_msgs::PoseStamped& goal){
        ROS_WARN("Create visualization marker");
        visualization_msgs::Marker vis_point;
        ROS_WARN("Base property initialization");
        vis_point.header.frame_id = goal.header.frame_id;
        vis_point.header.stamp = ros::Time::now();
        // vis_point.ns = "/los_nav";
        vis_point.action = visualization_msgs::Marker::ADD;
        vis_point.type = visualization_msgs::Marker::SPHERE;
        vis_point.pose.position = goal.pose.position;
        vis_point.pose.orientation = goal.pose.orientation;
        vis_point.id = 0;
        ROS_WARN("Set scale");
        // default size is 0.2
        vis_point.scale.x = 0.2;
        vis_point.scale.y = 0.2;
        vis_point.scale.z = 0.2;
        ROS_WARN("Set color");
        // default color is green
        vis_point.color.a = 1.0;
        vis_point.color.g = 1.0;
        vis_point.color.r = 0.0; 
        vis_point.color.b = 0.0;

        ROS_WARN("Marker setting done");
     }

}; // end of ns