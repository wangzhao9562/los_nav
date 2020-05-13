/*
 * @Author: Zhao Wang
 * @Date: 2020-05-07 17:40:29
 * @LastEditTime : 2020-05-13 
 * @LastEditors  : Zhao Wang
 * @Description: Definition of RosLosNav class
 * @FilePath: /los_nav/include/los_nav/ros_los_nav.h
 */
#ifndef ROS_LOS_NAV_H_
#define ROS_LOS_NAV_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <los_nav/los_nav.h>
#include <los_nav/Mission.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <los_nav_msgs/LosNavAction.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>

#include <std_srvs/Empty.h>
#include <string>
#include <boost/thread.hpp>
#include <cmath>
#include <thread>
#include <pthread.h>

namespace los_nav{
typedef actionlib::SimpleActionServer<los_nav_msgs::LosNavAction> LosNavActionServer;

class RosLosNav{
public:
    RosLosNav(tf::TransformListener& tf);
    ~RosLosNav();

private:
    void executeCb(const los_nav_msgs::LosNavGoalConstPtr& los_nav_goal);

    bool controlCycle(const geometry_msgs::PoseStamped& goal);

    geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

    bool isQuaternionValid(const geometry_msgs::Quaternion& q)const;

    void goalCb(const geometry_msgs::PoseStamped::ConstPtr& goal);

    bool getRobotPose(tf::Stamped<tf::Pose>& global_pose)const;

    void resetState();

    void publishZeroVelocity();

    /* Reserved for a callback function to modify the mission type of controller */
    void missionTypeCb(const los_nav::Mission::ConstPtr& type);

    visualization_msgs::Marker generateVisPoint(const geometry_msgs::PoseStamped& goal);

private:
    LosNav* performer_;
    tf::TransformListener& tf_;
    LosNavActionServer* los_nav_as_;
    // unsigned int mission_type_; // discarded

    std::string global_frame_, base_frame_;

    double stop_tolerance_, transform_tolerance_;
    double vel_; // follow velocity
    double control_frequency_;

    double kp_, kd_, ki_; // PID parameters
    double dx_err_, dy_err_;
    double los_factor_;

    tf::Stamped<tf::Pose> global_pose_;
    
    // std::thread type_thread_; // discarded
    pthread_rwlock_t flock_;

    ros::Subscriber goal_sub_;
    ros::Subscriber mission_type_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher action_goal_pub_;
    ros::Publisher current_goal_pub_;

    /* Preserved for visualized parameters publisher */
    ros::Publisher vis_pub_;
}; // end of class
}; // end of ns 

#endif