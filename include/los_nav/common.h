/*
 * @Author: your name
 * @Date: 2020-05-07 15:08:09
 * @LastEditTime: 2020-05-08 21:26:09
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: /los_nav/include/los_nav/common.h
 */
#ifndef COMMON_H_
#define COMMON_H_

#include <math.h>

namespace los_nav{
    /**
     * @brief Enum class to define mission type;  
     */
    enum class MissionType{
        NONE,
        POINT, 
        C_LINE,
        S_LINE,
        M_LINE,
        CIRCLE
    };

    const double PI = 3.1415926535;

    double distance(double x1, double y1, double x2, double y2);

    double radToAngle(double rad);

    double slopeToAngle(double slope);

    double slopeToAngle(double det_y, double det_x);

}; // end of ns

#endif