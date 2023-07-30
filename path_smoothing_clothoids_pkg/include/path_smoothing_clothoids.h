#pragma once
#ifndef PATH_SMOOTHING_CLOTHOIDS_H
#define PATH_SMOOTHING_CLOTHOIDS_H
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include<vector>
#include <tuple>
#include <iostream>
#include <algorithm>
#include <cmath>
//#include <boost/make_shared.hpp>
//#include <boost/range/adaptor/sliced.hpp>
#endif // PATH_SMOOTHING_CLOTHOIDS_H


struct Point 
{
    double x;
    double y;
};




struct Point3D 
{
    double x;
    double y;
    double z;
};


struct STGPoint 
{   
    double x;
    double y;
    double z;
};





    

    

    void globalPlanCallback(const nav_msgs::Path::ConstPtr& global_plan_msg);
    


    
    ros::NodeHandle nh_;
    ros::Subscriber global_plan_sub_;
    ros::Publisher smoothed_path_pub_;

    nav_msgs::Path clothoidInterpolation(const nav_msgs::Path::ConstPtr& global_plan_msg);
    

  
    // Function to calculate x and y coordinates of a basic clothoid, zero initial heading, zero inital curvature, scaling factor = 1, starting at the origin

    std::vector<std::pair<double, double>> Calculate_Basic_Clothoid(double s_length);
    
    


    std::pair<double, double> Reflect_point_across_normal(std::pair<double, double> point, std::pair<double, double> merging_point, double tangent_angle) ;
    
    std::vector<std::pair<double, double>> Merged_symmetric_clothoids(const std::vector<std::pair<double, double>>& first_clothoid_points) ;
    

    double distance(const Point& a, const Point& b) ;
    

    double angle(const Point& a, const Point& b, const Point& c) ;
    


    std::pair<std::vector<Point>, std::vector<int>> rdp_with_indices(const nav_msgs::Path::ConstPtr& global_plan_msg, double epsilon, double max_angle) ;
    

    std::vector<std::pair<double, double>> Clockwise_Merged_Clothoid(const std::vector<std::pair<double, double>>& merged_clothoid_points) ;
    

    

    std::vector<std::pair<double, double>> Subset_Basic_Clothoid(const std::vector<std::pair<double, double>>& Basic_Clothoid, double s_BC) ;
    



    std::pair<double, double> rotatePoint(double x, double y, double theta) ;
    
    std::vector<std::pair<double, double>> TransformSymmetricClothoid(
    const std::vector<std::pair<double, double>>& merged_symmetric_clothoid,
    double theta, const geometry_msgs::Point& start_point) ;
    


    
