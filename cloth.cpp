
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include<vector>
#include<cmath>
#include <tuple>
#include <iostream>
#include <algorithm>
#include <math.h>


class PathSmoothingNode
{
public:
  PathSmoothingNode()
  {
    // Subscribe to the global plan topic
    global_plan_sub_ = nh_.subscribe("/move_base_node/NavfnROS/plan", 1,
                                    &PathSmoothingNode::globalPlanCallback, this);

    // Publish the smoothed path to rviz
    smoothed_path_pub_ = nh_.advertise<nav_msgs::Path>("/rviz", 1);
  }

  void globalPlanCallback(const nav_msgs::Path::ConstPtr& global_plan_msg)
  {
    // Perform clothoid interpolation to generate a smooth path
    nav_msgs::Path smoothed_path = clothoidInterpolation(global_plan_msg);

    // Publish the smoothed path
    ROS_INFO("Received global plan with %zu poses", global_plan_msg->poses.size());
    smoothed_path_pub_.publish(smoothed_path);

  }

  nav_msgs::Path clothoidInterpolation(const nav_msgs::Path::ConstPtr& global_plan_msg)
  {
    
    // Return the smoothed path
    nav_msgs::Path smoothedPath;
    smoothedPath.header = global_plan_msg->header;
    smoothedPath.poses = global_plan_msg->poses;
    return smoothedPath;
  }

  double calculateAngle(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& p3)
    {
    double dx1 = p1.x - p2.x;
    double dy1 = p1.y - p2.y;
    double dx2 = p3.x - p2.x;
    double dy2 = p3.y - p2.y;
    
    double angle1 = std::atan2(dy1, dx1);
    double angle2 = std::atan2(dy2, dx2);
    double angle = std::abs(angle1 - angle2);
    
    // Convert angle to degrees
    angle = angle * 180.0 / M_PI;
    
    // Adjust the angle to be in the range of 0-180 degrees
    if (angle > 180.0)
        angle = 360.0 - angle;
    
    return angle;
    } 



  std::vector<int> identifySharpTurns(const nav_msgs::Path& globalPath, double thresholdAngle)
    {
    std::vector<int> sharpTurnIndices;
    thresholdAngle = 120;
    
    // Iterate through the global path
    for (int i = 1; i < globalPath.poses.size() - 1; ++i)
    {
        const geometry_msgs::Point& p1 = globalPath.poses[i-1].pose.position;
        const geometry_msgs::Point& p2 = globalPath.poses[i].pose.position;
        const geometry_msgs::Point& p3 = globalPath.poses[i+1].pose.position;
        
        double angle = calculateAngle(p1, p2, p3);
        
        // Check if the angle is greater than the threshold
        if (angle > thresholdAngle)
        {
            sharpTurnIndices.push_back(i);
        }
    }
    
    return sharpTurnIndices;
    }


// Function to calculate x and y coordinates of a basic clothoid, zero initial heading, zero inital curvature, scaling factor = 1, starting at the origin

std::vector<std::pair<double, double>> Calculate_Basic_Clothoid(double s_length) {
    std::vector<std::pair<double, double>> clothoid_points;
    double dt = 0.1; // Step size for numerical integration
    double t = 0.0;
    double x = 0.0;
    double y = 0.0;
    clothoid_points.push_back(std::make_pair(x, y));
    // Trapezoid Rule for integration
    while (t <= s_length) {
        x += dt * 0.5 * (std::cos(t * t / 2) + std::cos((t + dt) * (t + dt) / 2));
        y += dt * 0.5 * (std::sin(t * t / 2) + std::sin((t + dt) * (t + dt) / 2));
        clothoid_points.push_back(std::make_pair(x, y));
        t += dt;
    }
    
    return clothoid_points;
}

std::pair<double, double> Reflect_point_across_normal(std::pair<double, double> point, std::pair<double, double> merging_point, double tangent_angle) {
    double x = point.first;
    double y = point.second;
    double x_merge = merging_point.first;
    double y_merge = merging_point.second;
    
    double normal_x = std::cos(M_PI / 2 + tangent_angle);
    double normal_y = std::sin(M_PI / 2 + tangent_angle);

    double dx = x - x_merge;
    double dy = y - y_merge;

    // Calculate the dot product of the vector and the normal
    double dot_product = dx * normal_x + dy * normal_y;

    // Project the vector onto the normal
    double projected_x = dot_product * normal_x;
    double projected_y = dot_product * normal_y;

    double origin_ref_x = projected_x + (projected_x - dx);
    double origin_ref_y = projected_y + (projected_y - dy);

    double reflected_x = origin_ref_x + x_merge;
    double reflected_y = origin_ref_y + y_merge;

    return std::make_pair(reflected_x, reflected_y);


}

std::vector<std::pair<double, double>> Merged_symmetric_clothoids(const std::vector<std::pair<double, double>>& first_clothoid_points, double tangent_angle) {
 

    std::pair<double, double> merging_point = first_clothoid_points.back();
    std::pair<double, double> before_merg = first_clothoid_points[first_clothoid_points.size() - 2];
    double pseudo_tangent_x = before_merg.first - merging_point.first;
    double pseudo_tangent_y = before_merg.second - merging_point.second;
    double tangent_angle = std::atan2(pseudo_tangent_y, pseudo_tangent_x);

    std::vector<std::pair<double, double>> symmetric_clothoid_points;
    for (auto it = first_clothoid_points.rbegin(); it != first_clothoid_points.rend(); ++it) {
        symmetric_clothoid_points.push_back(Reflect_point_across_normal(*it, merging_point, tangent_angle));
    }

    std::vector<std::pair<double, double>> merged_clothoid_points = first_clothoid_points;
    merged_clothoid_points.insert(merged_clothoid_points.end(), symmetric_clothoid_points.begin(), symmetric_clothoid_points.end());

    return merged_clothoid_points;
}

double distance(const Point& a, const Point& b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

double angle(const Point& a, const Point& b, const Point& c) {
    double v1_x = a.x - b.x;
    double v1_y = a.y - b.y;
    double v2_x = c.x - b.x;
    double v2_y = c.y - b.y;

    double dot_product = v1_x * v2_x + v1_y * v2_y;
    double v1_length = sqrt(v1_x * v1_x + v1_y * v1_y);
    double v2_length = sqrt(v2_x * v2_x + v2_y * v2_y);

    return acos(dot_product / (v1_length * v2_length));
}


std::pair<std::vector<Point>, std::vector<int>> rdp_with_indices(const nav_msgs::Path::ConstPtr& global_plan_msg, double epsilon, double max_angle) {
    std::vector<Point> points;
    std::vector<int> indices;
    for (size_t i = 0; i < global_plan_msg->poses.size(); ++i) {
        const auto& pose = global_plan_msg->poses[i].pose;
        points.push_back({pose.position.x, pose.position.y});
        indices.push_back(i);
    }

    double dmax = 0.0;
    int index = 0;

    for (size_t i = 1; i < points.size() - 1; ++i) {
        double a = angle(points[i - 1], points[i], points[i + 1]);
        if (a < max_angle) {
            double d = distance(points[i], points[0]) + distance(points[i], points.back());
            if (d > dmax) {
                index = static_cast<int>(i);
                dmax = d;
            }
        }
    }

    std::pair<std::vector<Point>, std::vector<int>> result;
    if (dmax >= epsilon) {
        // Recursively simplify the points and indices
        auto simplified_left_indices = std::vector<int>(indices.begin(), indices.begin() + index + 1);
        auto simplified_left = rdp_with_indices(global_plan_msg, epsilon, max_angle);

        auto simplified_right_indices = std::vector<int>(indices.begin() + index, indices.end());
        auto simplified_right = rdp_with_indices(global_plan_msg, epsilon, max_angle);

        // Combine the results, excluding the last point from the left segment to avoid duplication
        result.first = std::vector<Point>(simplified_left.first.begin(), simplified_left.first.end() - 1);
        result.second = std::vector<int>(simplified_left_indices.begin(), simplified_left_indices.end() - 1);
        result.first.insert(result.first.end(), simplified_right.first.begin(), simplified_right.first.end());
        result.second.insert(result.second.end(), simplified_right_indices.begin(), simplified_right_indices.end());
    } else {
        // If the maximum perpendicular distance is less than epsilon, keep only the first and last points
        result.first = {points.front(), points.back()};
        result.second = {indices.front(), indices.back()};
    }

    return result;
}

std::vector<std::pair<double, double>> Clockwise_Merged_Clothoid(const std::vector<std::pair<double, double>>& merged_clothoid_points) {
    std::vector<std::pair<double, double>> clockwise_merged_clothoid;
    for (const auto& point : merged_clothoid_points) {
        double point_x = point.first;
        double point_y = -point.second;
        clockwise_merged_clothoid.push_back(std::make_pair(point_x, point_y));
    }

    return clockwise_merged_clothoid;
}

std::vector<std::pair<double, double>> Subset_Basic_Clothoid(const std::vector<std::pair<double, double>>& Basic_Clothoid, double s_BC) {
    std::vector<std::pair<double, double>> subset_clothoid_points;

    // Ensure s_BC is within the range of Basic_Clothoid
    s_BC = std::min(s_BC, static_cast<double>((Basic_Clothoid.size() - 1) * 0.1));

    // Number of points to be extracted from the lookup table
    int num_points = static_cast<int>(s_BC / 0.1);

    // Extract the desired number of points
    for (int i = 0; i <= num_points; ++i) {
        subset_clothoid_points.push_back(Basic_Clothoid[i]);
    }

    return subset_clothoid_points;
}

struct Point {
    double x;
    double y;
};

std::pair<double, double> RotatePoint(const std::pair<double, double>& point, double angle) {
    double cos_theta = std::cos(angle);
    double sin_theta = std::sin(angle);

    double new_x = point.first * cos_theta - point.second * sin_theta;
    double new_y = point.first * sin_theta + point.second * cos_theta;

    return std::make_pair(new_x, new_y);
}

std::pair<double, double> TranslatePoint(const std::pair<double, double>& point, const std::pair<double, double>& translation) {
    return std::make_pair(point.first + translation.first, point.second + translation.second);
}

std::vector<std::pair<double, double>> TransformSymmetricClothoid(const std::vector<std::pair<double, double>>& merged_symmetric_clothoid, const std::pair<double, double>& start_point) {
    std::vector<std::pair<double, double>> transformed_points;

    // Step 1: Calculate the angle between the first point of the curve and the start_point
    double angle = CalculateAngle(merged_symmetric_clothoid[0], start_point);

    // Step 2: Rotate each point around the origin (first point) by the calculated angle
    for (const auto& point : merged_symmetric_clothoid) {
        std::pair<double, double> rotated_point = RotatePoint(point, angle);
        transformed_points.push_back(rotated_point);
    }

    // Step 3: Translate all the points so that the first point (origin) moves to start_point
    std::pair<double, double> translation = std::make_pair(start_point.first - merged_symmetric_clothoid[0].first, start_point.second - merged_symmetric_clothoid[0].second);
    for (auto& point : transformed_points) {
        point = TranslatePoint(point, translation);
    }

    return transformed_points;
}





void Clothoid_Interpolation(const nav_msgs::Path::ConstPtr& global_plan_msg, double s_length, double epsilon,double max_angle){


epsilon = 0.1; //The maximum distance between the original path and the simplified path
max_angle = 120; //The maximum angle between the two consecutive points of the original path
s_lenth = 5.0; //The length of the clothoid in the lookup table









//The lookup table of a basic Clthoid, so that the Fresnal integrals are only solved once here
std::vector<std::pair<double, double>> Basic_Clothoid = Calculate_Basic_Clothoid(double s_length);

const auto& simplified_result = rdp_with_indices(global_plan_msg, epsilon, max_angle);

    
    const std::vector<Point>& simplified_points = simplified_result.first;
    const std::vector<int>& turning_point_indices = simplified_result.second;




    // Iterate through the turning points (excluding the first and last points as they are already present in the global plan)
    for (size_t i = 1; i < turning_point_indices.size() - 1; ++i) {
        int turning_point_index = turning_point_indices[i];
        const Point& turning_point = simplified_points[turning_point_index];

        int start_index = turning_point_indices[i - 2];
        const Point& start_point = global_plan_msg->poses[start_index].pose.position;
        
        // Identify goal_point (one index after the turning_point)
        int goal_index = turning_point_indices[i + 2];
        const Point& goal_point = global_plan_msg->poses[goal_index].pose.position;


        //Transform the points so that Starting point is at Orign and the Turning Point is at the positive xAxis
        double translate_x = -start_point.x;
        double translate_y = -start_point.y;

        // Step 2: Calculate the angle between the turning point and the positive x-axis
        double theta = atan2(turning_point.y - start_point.y, turning_point.x - start_point.x);

        // Step 3: Rotate only the three points (start_point, turning_point, and goal_point) by the negative of that angle to align the turning point with the positive x-axis
        double transformed_start_x = cos(-theta) * (start_point.x + translate_x) - sin(-theta) * (start_point.y + translate_y);
        double transformed_start_y = sin(-theta) * (start_point.x + translate_x) + cos(-theta) * (start_point.y + translate_y);

        double transformed_turning_x = cos(-theta) * (turning_point.x + translate_x) - sin(-theta) * (turning_point.y + translate_y);
        double transformed_turning_y = sin(-theta) * (turning_point.x + translate_x) + cos(-theta) * (turning_point.y + translate_y);

        double transformed_goal_x = cos(-theta) * (goal_point.x + translate_x) - sin(-theta) * (goal_point.y + translate_y);
        double transformed_goal_y = sin(-theta) * (goal_point.x + translate_x) + cos(-theta) * (goal_point.y + translate_y);

        double d_max = transformed_turning_x;
        double Phi_C2 = math.atan2(transformed_goal_y, transformed_goal_x) * 0.5;
        double s_BC = math.sqrt(2 * Phi_C2);


        //Now take from Basic Clothoid the part with length s_BC
        std::vector<std::pair<double, double>> subset_basic_clothoid = Subset_Basic_Clothoid(Basic_Clothoid,s_BC);

        //Last point of the subset clothoid
        double last_x = subset_basic_clothoid.back().first;
        double last_y = subset_basic_clothoid.back().second;

        d_BC = last_x + last_y * tan(Phi_C2);
        e_BC = last_y / cos(Phi_C2);

        double Scale_Ratio = d_BC/d_max;  //The scale Ratio 

        double s_final = s_BC/Scale_Ratio; //The final length of the clothoid

        double c_sharpness = Scale_Ratio * Scale_Ratio; //The sharpness of the clothoid

        std::vector<std::pair<double, double>> first_clothoid_points = Subset_Basic_Clothoid(Basic_Clothoid,s_final); //first Clothoid with final length

        for (auto& point : first_clothoid_points) {
        point.first /= c_sharpness;
        point.second /= c_sharpness;
    }

        
        //Now merge the first clothoid with its symmetric clothoid

        //Is the merged clothoid clockwise or counter-clockwise?
        double determinant = (start_point.x * turning_point.y + start_point.y * goal_point.x + turning_point.x * goal_point.y) - (goal_point.x * turning_point.y + goal_point.y * start_point.x + turning_point.x * start_point.y);
        // Determinant of Matrix | x1 y1 1 |
        //                       | x2 y2 1 |
         //                      | x3 y3 1 |

         if determinant>0 {
            std::vector<std::pair<double, double>> merged_symmetric_clothoid = Merged_symmetric_clothoids(first_clothoid_points);
         }
         else {
            std::vector<std::pair<double, double>> merged_symmetric_clothoid = Clockwise_Merged_Clothoid(Merged_symmetric_clothoids(first_clothoid_points));
         }


        std::vector<std::pair<double, double>> transformed_points = TransformSymmetricClothoid(merged_symmetric_clothoid, start_point);
        // Erease the points between the start_point and the goal_point
        global_plan_msg->poses.erase(global_plan_msg->poses.begin() + start_index + 1, global_plan_msg->poses.begin() + goal_index);
        //Insert the transformed_points at the index of the start_point in the global plan
        global_plan_msg->poses.insert(global_plan_msg->poses.begin() + start_index + 1, transformed_points.begin(), transformed_points.end());






        


}






































// Function that given the length of the Basic Clothoid, gives out a few paramters that will be used later to scale the clothoid
std::tuple<double, double, double, double, double, double> parameterBasicClothoid(double s_length) {
    
    double Phi_C2 = s_length * s_length / 2;
    
    
    std::vector<std::pair<double, double>> x_and_y_BCt = calculateBasicClothoid(s_length);
    
    std::pair<double, double> x_and_y_BCt_lastPair = x_and_y_BCt.back();
    double x_BCt = x_and_y_BCt_lastPair.first;
    double y_BCt = x_and_y_BCt_lastPair.second;
    
    
    double d_BC = x_BCt + y_BCt * tan(Phi_C2);
    double e_BC = y_BCt/cos(Phi_C2);
    double x_TP = d_BC;
    double y_TP = 0.0;

    

    // Return the six variables as a tuple
    return std::make_tuple(x_BCt, y_BCt, d_BC, e_BC, x_TP, y_TP);
}




//This function gives out the paramters of the two symmetric clothoids that are used to connect the Start and Goal points
std::vector<double> smoothSharpTurn(double x_start,double y_start, double x_turn, double y_turn,  double x_goal, double y_goal, double e_max, double d_max) {
    
    std::vector<double> result;

    double d_max = std::min(std::fabs(x_start), std::fabs(std::sqrt(x_goal * x_goal + y_goal * y_goal)));
    double Phi_C2 = atan2 (y_goal,x_goal)*0.5;  
    double s_BC = sqrt(2*Phi_C2);
    std::tuple<double, double, double, double, double, double> par_Basic_Clothoid = parameterBasicClothoid(s_BC);
    double x_BCt = std::get<0>(par_Basic_Clothoid);
    double y_BCt = std::get<1>(par_Basic_Clothoid);
    double d_BC = std::get<2>(par_Basic_Clothoid);
    double e_BC = std::get<3>(par_Basic_Clothoid);
    double x_TP = std::get<4>(par_Basic_Clothoid);
    double y_TP = std::get<5>(par_Basic_Clothoid);

    double scale_ratio = std::max(fabs(d_BC/d_max), fabs(e_BC/e_max));
    double s = s_BC/scale_ratio;
    double x_C2 = x_BCt/scale_ratio;
    double y_C2 = y_BCt/scale_ratio;
    double d = d_BC/scale_ratio;
    double x_0_C1 = -d;
    double y_0_C1 = 0.0;
    double Phi_C1 = 0.0;
    double kappa_0_C1 = 0.0;
    double c_C1 = scale_ratio*scale_ratio;
    double s_C1 = s;
    double x_0_C2 = x_C2;
    double y_0_C2 = y_C2;
    double Phi_0_C2 = Phi_C2;
    double kappa_0_C2 = s_C1*c_C1;
    double c_c2 = - scale_ratio*scale_ratio;
    double s_C2 = s;





    
    result.push_back(0);
    

    return result;
}
