/*
 * @Author: Zhao Wang
 * @Date: 2020-05-15 13:06:13
 * @LastEditTime: 2020-05-17 21:39:44
 * @LastEditors: Please set LastEditors
 * @Description: Test client for los nav
 * @FilePath: /los_nav/test/test_client.cpp
 */
#include <ros/ros.h>
#include <tf/tf.h>
#include <los_nav/los_nav.h>
#include <los_nav_msgs/Mission.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <los_nav_msgs/LosNavAction.h>

#include <cstdlib>
#include <cmath> 
#include <string>
#include <iostream>

typedef actionlib::SimpleActionClient<los_nav_msgs::LosNavAction> LosNavActionClient;

int main(int argc, char** argv){
    ros::init(argc, argv, "test_los_nav");
    ros::NodeHandle nh;
    ros::Publisher mission_pub = nh.advertise<los_nav_msgs::Mission>("mission_type", 1);
    
    unsigned int mission_type = 0;
    const char* mission_type_tag = argv[1];

    ROS_INFO("mission type: %s", mission_type_tag);

    if(std::strcmp(mission_type_tag, "-p") == 0){
        mission_type = 0; // point following command
        ROS_INFO("Mission type is point following");
    }
    else if(std::strcmp(mission_type_tag, "-cl") == 0){
        mission_type = 1; // common line following command
        ROS_INFO("Mission type is common line following");
    }
    else if(std::strcmp(mission_type_tag, "-c") == 0){
        mission_type = 2; // circle following command
        ROS_INFO("Mission type is circle following");
    }

    LosNavActionClient los_nav_ac("los_nav", true);
    ROS_INFO("Los nav client started");

    los_nav_ac.waitForServer();
    ROS_INFO("Action server started, send mission message");

    los_nav_msgs::Mission mission;

    mission.header.frame_id = "wamv/odom";
    mission.header.stamp = ros::Time::now();

    switch (mission_type)
    {
    case 0:
        {
            const char* target_x_c_str = argv[2];
            const char* target_y_c_str = argv[3];
      
            mission.mission_type = 0;;
            mission.goal.header.frame_id = "wamv/odom";
            mission.goal.header.stamp = ros::Time::now();
            mission.goal.pose.position.x = std::atof(target_x_c_str);
            mission.goal.pose.position.y = std::atof(target_y_c_str);
            mission.goal.pose.position.z = 0.0;
            mission.goal.pose.orientation.w = 1.0;
            mission_pub.publish(mission);
        }
        break;
    case 1:
        { 
            if(argc < 5){
                ROS_ERROR("Input argument is not enough!");
                exit(1);
            }

            for(int i = 1; i < argc - 4; i += 4){
                const char* line_start_x = argv[i];
                const char* line_start_y = argv[i + 1];
                const char* line_end_x = argv[i + 2];
                const char* line_end_y = argv[i + 3]; 
                los_nav_msgs::CommonLine c_line;
                c_line.start_x = std::atof(line_start_x);
                c_line.start_y = std::atof(line_start_y);
                c_line.end_x = std::atof(line_end_x);
                c_line.end_y = std::atof(line_end_y);
                double k = std::atan2(c_line.start_y - c_line.end_y, c_line.start_x - c_line.end_x);
                double b = c_line.start_y - c_line.start_x * k;
                c_line.k = k;
                c_line.b = b;
                c_line.is_reverse = false;
                mission.lines.push_back(c_line);
            }

            mission_pub.publish(mission);
        }
        break;
    case 2:
        {
            if(argc != 4){
                ROS_ERROR("Illegal input");
                exit(1);
            }
            const char* circle_origin_x = argv[2];
            const char* circle_origin_y = argv[3];
            const char* circle_r = argv[4];
            mission.circle.origin_x = std::atof(circle_origin_x);
            mission.circle.origin_y = std::atof(circle_origin_y);
            mission.circle.r = std::atof(circle_r);
            mission_pub.publish(mission);
        }
        break;
    default:
        ROS_ERROR("Undefined mission type!");
        break;
    } 

    los_nav_msgs::LosNavGoal goal;
    goal.mission_msgs = mission;
    los_nav_ac.sendGoal(goal);
    ROS_INFO("Los nav action goal has been published!"); 

    return 0;
}