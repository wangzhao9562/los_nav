/*
 * @Author: Zhao Wang
 * @Date: 2020-05-07 17:40:29
 * @LastEditTime: 2020-05-21 12:13:12
 * @LastEditors: Please set LastEditors
 * @Description: Definition of RosLosNav class
 * @FilePath: /los_nav/include/los_nav/ros_los_nav.h
 */
#ifndef ROS_LOS_NAV_H_
#define ROS_LOS_NAV_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <los_nav/los_nav.h>
// #include <los_nav/Mission.h>
#include <los_nav_msgs/Mission.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <los_nav_msgs/LosNavAction.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>

#include <vector>
#include <string>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <cmath>
#include <pthread.h>
#include <deque>

namespace los_nav{
typedef actionlib::SimpleActionServer<los_nav_msgs::LosNavAction> LosNavActionServer;

class RosLosNav{
public:
    RosLosNav(tf::TransformListener& tf);
    ~RosLosNav();

private:
    void executeCb(const los_nav_msgs::LosNavGoalConstPtr& los_nav_goal);

    // bool controlCycle(const geometry_msgs::PoseStamped& goal);
    bool controlCycle(const los_nav_msgs::Mission& mission);

    geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

    bool isQuaternionValid(const geometry_msgs::Quaternion& q)const;

    // void goalCb(const geometry_msgs::PoseStamped::ConstPtr& goal); // Discarded

    bool getRobotPose(tf::Stamped<tf::Pose>& global_pose)const;

    void resetState();

    void publishZeroVelocity();

    /* Reserved for a callback function to modify the mission type of controller */
    // void missionTypeCb(const los_nav::Mission::ConstPtr& type);
    void missionTypeCb(const los_nav_msgs::Mission::ConstPtr& type);

    void generateVisTarget(visualization_msgs::Marker& m, const geometry_msgs::PoseStamped& goal);

    void generateVisLineStrip(visualization_msgs::Marker& m, const std::vector<geometry_msgs::Point>& p_vec, std::string frame_id);

    void generateVisCircle(visualization_msgs::Marker& m, double ori_x, double ori_y, double radius, std::string frame_id);

    std::vector<geometry_msgs::Point> circleToPoints(double ori_x, double ori_y, double radius);

    void switchController(const los_nav_msgs::Mission& current_mission);

    void trajectoryPublish(); // thread function for USV trajectory publishing
    
    void trajectoryHidden(); 

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

    nav_msgs::Path path_;

    tf::Stamped<tf::Pose> global_pose_;
    
    // std::thread type_thread_; // discarded
    boost::thread* traj_pub_thread_; // trajectory publish thread
    boost::thread* traj_hide_thread_;
    pthread_rwlock_t flock_;

    ros::Subscriber goal_sub_;
    ros::Subscriber mission_type_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher action_goal_pub_;
    ros::Publisher current_goal_pub_;

    /* Preserved for visualized parameters publisher */
    ros::Publisher vis_pub_;
    ros::Subscriber odom_sub_;
    ros::Publisher path_pub_;

    visualization_msgs::Marker marker_;
}; // end of class
}; // end of ns 

#endif