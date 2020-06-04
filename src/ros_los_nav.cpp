/*
 * @Author: Zhao Wang
 * @Date: 2020-05-11 
 * @LastEditTime: 2020-06-04 17:03:25
 * @LastEditors: Please set LastEditors
 * @Description: Implementation of interface of RosLosNav class
 * @FilePath: /los_nav/src/clf_los_controller.cpp
 */
#include <los_nav/ros_los_nav.h>
#include <vector>
#include <exception>

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
        private_nh.param("velocity", vel_, 0.4);
        private_nh.param("stop_tolerance", stop_tolerance_, 1.5);
        private_nh.param("transform_tolerance", transform_tolerance_, 0.2);
        private_nh.param("global_frame", global_frame_, std::string("wamv/odom"));
        private_nh.param("base_frame", base_frame_, std::string("wamv/base_link"));
        private_nh.param("control_frequency", control_frequency_, 20.0); // hz
        private_nh.param("kp", kp_, 0.5);
        private_nh.param("kd", kd_, 5.0);
        private_nh.param("ki", ki_, 0.5);
        private_nh.param("dx_err", dx_err_, -2.0);
        private_nh.param("dy_err", dy_err_, 4.0);
        private_nh.param("factor", los_factor_, 3.0);

        ROS_INFO_STREAM("Follow velocity: " << vel_);
        ROS_INFO_STREAM("Transform tolerance: " << stop_tolerance_);
        ROS_INFO_STREAM("global_frame: " << global_frame_);
        ROS_INFO_STREAM("base_frame: " << base_frame_);
        ROS_INFO_STREAM("control_frequency: " << control_frequency_);
        ROS_INFO_STREAM("PID parameters: " << "kp: " << kp_ << " kd: " << kd_ << " ki: " << ki_);

        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); // publish velocity command
        // goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&RosLosNav::goalCb, this, _1));
        // mission_type_sub_ = nh.subscribe<los_nav_msgs::Mission>("mission_type", 1, boost::bind(&RosLosNav::missionTypeCb, this, _1));
        // action_goal_pub_ = action_nh.advertise<los_nav_msgs::LosNavActionGoal>("goal", 1);
        current_goal_pub_ = private_nh.advertise<los_nav_msgs::Mission>("current_goal", 1);

        vis_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

        path_pub_ = nh.advertise<nav_msgs::Path>("trajectory", 1);

        los_nav_as_ = new LosNavActionServer(ros::NodeHandle(), "los_nav", boost::bind(&RosLosNav::executeCb, this, _1), false);

        ROS_INFO("Create control performer");
  
        path_.header.frame_id = global_frame_;
        path_.header.stamp = ros::Time::now();
        traj_pub_thread_ = new boost::thread(boost::bind(&RosLosNav::trajectoryPublish, this));
        // traj_hide_thread_ = new boost::thread(boost::bind(&RosLosNav::trajectoryHidden, this));

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
            los_nav_as_ = nullptr;
        }
        if(performer_){
            delete performer_;
            performer_ = nullptr;
        }
        if(traj_pub_thread_){
            delete traj_pub_thread_;
            traj_pub_thread_ = nullptr;
        }
        if(traj_hide_thread_){
            delete traj_hide_thread_;
            traj_hide_thread_ = nullptr;
        }
    }

    void RosLosNav::executeCb(const los_nav_msgs::LosNavGoalConstPtr& los_nav_goal){
        ROS_INFO("Received goal");
          
        los_nav_msgs::Mission current_mission = los_nav_goal->mission_msgs;

        current_goal_pub_.publish(current_mission);

        ros::Rate r(control_frequency_);
        
        // performer_ = new LosNav(kp_, kd_, ki_);

        ros::NodeHandle n;

        pthread_rwlock_wrlock(&flock_);
        // Initialize los nav performer according to mission type
        if(performer_){
            if(performer_->isControllerAvailable()){
                ROS_INFO("Check if the controller should be changed");
                switchController(current_mission);
            }
        }
        else{
            resetState();
        }
        pthread_rwlock_unlock(&flock_);

        while(n.ok()){
            // publish visualization marker
            vis_pub_.publish(marker_); 

            ROS_INFO("In controlling loop");
            if(los_nav_as_->isPreemptRequested()){
                if(los_nav_as_->isNewGoalAvailable()){
                    ROS_INFO("Interrupt with new mission");
                    // los_nav_msgs::LosNavGoal new_mission = *los_nav_as_->acceptNewGoal();
                    // current_mission = new_mission.mission_msgs;
                    // current_goal_pub_.publish(current_mission);
                    los_nav_as_->setPreempted();
                    return;
                }
                else{
                    // resetState();
                    los_nav_as_->setPreempted();
                    break;
                }
            }

            /*Preserved for frame transformation*/
       
            ROS_INFO("Start control");
            bool done = controlCycle(current_mission);
            if(done){
                return;
            }
            
            r.sleep();
        };
        los_nav_as_->setAborted(los_nav_msgs::LosNavResult(), 
            "Aborting on the mission cus the node has been killed!");

        return;
    }

    bool RosLosNav::controlCycle(const los_nav_msgs::Mission& mission){
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
                    mission.goal.pose.position.x, mission.goal.pose.position.y,
                    tf::getYaw(current_position.pose.orientation));

                if(ctrl_fb.second == 0){
                    cmd_vel.linear.x = vel_;
                    cmd_vel.angular.z = ctrl_fb.first;
                    ROS_WARN_STREAM("cmd_vel: " << "linear velocity x: " << vel_ << " angular velocity z: " << cmd_vel.angular.z);
                    cmd_vel_pub_.publish(cmd_vel);
                    pthread_rwlock_unlock(&flock_);
                    return false;
                }
                else if(ctrl_fb.second == 1){
                    ROS_INFO("Controlling done");
                    publishZeroVelocity();
                    los_nav_as_->setSucceeded(los_nav_msgs::LosNavResult(), "Goal reached!");
                    pthread_rwlock_unlock(&flock_);
                    return true;
                }
                else{
                    publishZeroVelocity();
                    pthread_rwlock_unlock(&flock_);
                    return false;
                }
            }
            else{
                ROS_INFO("Wait for the initialization of controller");
                pthread_rwlock_unlock(&flock_);
                return false;
            }
        }
        
        resetState(); 
        // pthread_rwlock_unlock(&flock_);
        
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
            return false;
        }
        return true;
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

     void RosLosNav::missionTypeCb(const los_nav_msgs::Mission::ConstPtr& type){
        // Transfer received message to action goal message type
        ROS_INFO("Transform received goal");
        los_nav_msgs::LosNavActionGoal action_goal;
        action_goal.header.frame_id = global_frame_;
        action_goal.header.stamp = ros::Time::now();
        action_goal.goal.mission_msgs.header.frame_id = type->header.frame_id;
        action_goal.goal.mission_msgs.header.stamp = type->header.stamp;
        action_goal.goal.mission_msgs.mission_type = type->mission_type;
        action_goal.goal.mission_msgs.lines = type->lines;
        action_goal.goal.mission_msgs.circle = type->circle;
        
        action_goal_pub_.publish(action_goal);
     }
 
     void RosLosNav::generateVisTarget(visualization_msgs::Marker& marker, const geometry_msgs::PoseStamped& goal){
        visualization_msgs::Marker m;
        m.header.frame_id = goal.header.frame_id;
        m.header.stamp = ros::Time::now();
        // m.ns = "/los_nav";
        m.action = visualization_msgs::Marker::ADD;
        m.type = visualization_msgs::Marker::SPHERE;
        m.pose.position = goal.pose.position;
        m.pose.orientation = goal.pose.orientation;
        m.id = 0;
        // default size is 0.2
        m.scale.x = 1.0;
        m.scale.y = 1.0;
        m.scale.z = 1.0;
        // default color is green
        m.color.a = 1.0f;
        m.color.g = 1.0f;
        m.color.r = 0.0f; 
        m.color.b = 0.0f;

        marker = m;
     }

     void RosLosNav::generateVisLineStrip(visualization_msgs::Marker& marker, const std::vector<geometry_msgs::Point>& p_vec, 
        std::string frame_id){
        visualization_msgs::Marker m;    
        m.header.frame_id = frame_id;
        m.header.stamp = ros::Time::now();
        m.action = visualization_msgs::Marker::ADD;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.pose.orientation.w = 1.0;
        m.id = 1;
        m.scale.x = 0.5;
        m.color.b = 1.0;
        m.color.a = 1.0;
        for(auto ele : p_vec){
            m.points.push_back(ele);
        }

        marker = m;
     }

    void RosLosNav::generateVisCircle(visualization_msgs::Marker& marker, double ori_x, double ori_y, double radius, 
        std::string frame_id)
     {
        std::vector<geometry_msgs::Point> p_vec = std::move(circleToPoints(ori_x, ori_y, radius));
        ROS_INFO("circle point num: %d", p_vec.size());
        generateVisLineStrip(marker, p_vec, frame_id);     
     }

    std::vector<geometry_msgs::Point> RosLosNav::circleToPoints(double ori_x, double ori_y, double radius){
        double rad = 0;
        std::vector<geometry_msgs::Point> p_vec;
        for(; rad <= 2 * PI; rad += 5 * PI / 180){
            geometry_msgs::Point p;
            p.x = ori_x + radius * std::cos(rad);
            p.y = ori_y + radius * std::sin(rad);
            p.z = 0.0;
            p_vec.push_back(p);
        }
        ROS_INFO("origin circle point num: %d", p_vec.size());    
        return p_vec;
     }

    void RosLosNav::switchController(const los_nav_msgs::Mission& current_mission){
        ROS_INFO("Received mission type: %d", current_mission.mission_type);
        switch (current_mission.mission_type)
        {
            case 0:
            {
                if(performer_->getMissionType() != MissionType::POINT){        
                    ROS_INFO("Update visualization marker");
                    generateVisTarget(marker_, current_mission.goal);
                    ROS_INFO("Switch to point follow controller");
                    performer_->initialize(stop_tolerance_);
                }
                else{
                    ROS_INFO("Update visualization marker");
                    generateVisTarget(marker_, current_mission.goal);
                }
            }
            break;
            case 1:
            {
                // ROS_WARN_STREAM("lines num: " << current_mission.lines.size());
                // ROS_WARN_STREAM("target: " << "x: " << current_mission.goal.pose.position.x << " y: " << current_mission.goal.pose.position.y);
                // ROS_WARN_STREAM("line: sx, sy " << "x: " << current_mission.lines[0].end_x << "y: " << current_mission.lines[0].start_x);
                double k = static_cast<double>(current_mission.lines[0].start_y - current_mission.lines[0].end_y) / (current_mission.lines[0].start_x - current_mission.lines[0].end_x); 
                double b = current_mission.lines[0].start_y - current_mission.lines[0].start_x * k;
                // ROS_WARN_STREAM("line: k, b " << "k: " << current_mission.lines[0].k << "b: " << current_mission.lines[0].b);
                CLine line{current_mission.lines[0].start_x, current_mission.lines[0].start_y,
                           current_mission.lines[0].end_x, current_mission.lines[0].end_y,
                           current_mission.lines[0].k, current_mission.lines[0].b, current_mission.lines[0].is_reverse};
                geometry_msgs::Point s_p;
                s_p.x = current_mission.lines[0].start_x;
                s_p.y = current_mission.lines[0].start_y;
                s_p.z = 0.0;
                geometry_msgs::Point e_p;
                e_p.x = current_mission.lines[0].end_x;
                e_p.y = current_mission.lines[0].end_y;
                e_p.z = 0.0;

                ROS_WARN_STREAM("Cline: k: " << line.k_ << " b: " << line.b_);

                std::vector<geometry_msgs::Point> p_vec;
                p_vec.push_back(s_p);
                p_vec.push_back(e_p);
                if(performer_->getMissionType() != MissionType::C_LINE){
                    // Regenerate visualization marker
                    ROS_INFO("Update visualization marker");
                    generateVisLineStrip(marker_, p_vec, current_mission.header.frame_id);
                    // Initialize los nav performer
                    ROS_INFO("Switch to common line follow controller");
                    std::cout << "los_factor: " << los_factor_ << std::endl; 
                    performer_->initialize(line, los_factor_, stop_tolerance_);
                }
                else{
                    ROS_INFO("Update visualization marker");
                    generateVisLineStrip(marker_, p_vec, current_mission.header.frame_id);
                }   
            }
            break;
            case 2:
            {
                if(performer_->getMissionType() != MissionType::CIRCLE){
                    /* Preserved for implementation of controller initialization */
                    Circle circle{current_mission.circle.origin_x, current_mission.circle.origin_y, 
                            current_mission.circle.r};
                    ROS_INFO("Update visualization marker");
                    generateVisCircle(marker_, current_mission.circle.origin_x, current_mission.circle.origin_y, current_mission.circle.r, 
                            current_mission.header.frame_id);
                    ROS_INFO_STREAM("Circle information: " << current_mission.circle.origin_x << "," << current_mission.circle.origin_y << "," << current_mission.circle.r);
                    ROS_INFO("Switch to circle following mission");        
                    performer_->initialize(circle, los_factor_, stop_tolerance_);
                }
                else{
                    ROS_INFO("Update visualization marker");
                    generateVisCircle(marker_, current_mission.circle.origin_x, current_mission.circle.origin_y, current_mission.circle.r, 
                            current_mission.header.frame_id);
                }   
            }
            break;
            default:
            {
                ROS_WARN("Invalid mission type");
            }
            break;
        }       
    }    

    void RosLosNav::trajectoryPublish()
    {
        ros::NodeHandle nh;
        while(nh.ok()){
            // get robot pose
            tf::Stamped<tf::Pose> global_pose;
            getRobotPose(global_pose);

            geometry_msgs::PoseStamped current_position;
            tf::poseStampedTFToMsg(global_pose, current_position);

            pthread_rwlock_wrlock(&flock_);
            path_.poses.push_back(current_position);
            path_pub_.publish(path_);
            pthread_rwlock_unlock(&flock_);

            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        }
    }

    void RosLosNav::trajectoryHidden(){
        ros::NodeHandle nh;
        ros::Time start_time = ros::Time::now();
        while(nh.ok()){
            ros::Time current_time = ros::Time::now();
            nav_msgs::Path copy_path;
            // hide half past part of path
            if((current_time - start_time) >= ros::Duration(120)){
                for(int i = path_.poses.size() / 2; i < path_.poses.size(); i++){
                    copy_path.poses.push_back(path_.poses[i]);
                }
                pthread_rwlock_wrlock(&flock_);
                path_.poses = copy_path.poses;
                pthread_rwlock_unlock(&flock_);
            }
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        }
    }

}; // end of ns