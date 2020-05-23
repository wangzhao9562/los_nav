/*
 * @Author: your name
 * @Date: 2020-05-07 15:08:09
 * @LastEditTime: 2020-05-23 21:18:55
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /los_nav/include/los_nav/common.h
 */
#ifndef COMMON_H_
#define COMMON_H_

/*
 * @Author: Zhao Wang
 * @Date: 2020-05-07 
 * @LastEditTime: 2020-05-10
 * @LastEditors: Zhao Wang
 * @Description: Definition of common methods in los_nav package
 * @FilePath: /los_nav/src/pf_los_controller.cpp
 */
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

    struct CLine{
        CLine(){};
        CLine(double s_x, double s_y, double e_x, double e_y, 
            double k, double b, bool is_reverse = false) : start_x_(s_x), start_y_(s_y), end_x_(e_x), end_y_(e_y), 
                k_(k), b_(b), is_reverse_(is_reverse){}
        CLine(const CLine& cline){
            start_x_ = cline.start_x_;
            start_y_ = cline.start_y_;
            end_x_ = cline.end_x_;
            end_y_ = cline.end_y_;
            k_ = cline.k_;
            b_ = cline.b_;
            is_reverse_ = cline.is_reverse_;
        }
    
        double start_x_, start_y_, end_x_, end_y_;
        double k_, b_;
        bool is_reverse_;
    };

    struct Circle{
        Circle(){};
        Circle(double origin_x, double origin_y, double r) : origin_x_(origin_x), origin_y_(origin_y), r_(r){}
        double origin_x_, origin_y_;
        double r_;
    };

    const double PI = 3.1415926535;

    double distance(double x1, double y1, double x2, double y2);

    double radToAngle(double rad);

    double slopeToAngle(double slope);

    double slopeToAngle(double det_y, double det_x);

}; // end of ns

#endif