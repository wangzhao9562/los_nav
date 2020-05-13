/*
 * @Author: Zhao Wang
 * @Date: 2020-05-09 20:08:07
 * @LastEditTime: 2020-05-09 20:55:46
 * @LastEditors: Zhao Wang
 * @Description: Driver program of los nav node
 * @FilePath: /los_nav/src/los_nav_node.cpp
 */
#include <los_nav/ros_los_nav.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "los_nav_node");
    tf::TransformListener tf(ros::Duration(10));

    los_nav::RosLosNav los_nav_node(tf);

    ros::spin();

    return 0;
}