
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
  std::vector<std::pair<double, double>> calculateBasicClothoid(double s_length) {
    std::vector<std::pair<double, double>> clothoid_points;
    double t = 0.0;
    double dt = 0.1; // Step size for numerical integration

    while (t <= s_length) {
        double x = t * cos(t * t / 2);
        double y = t * sin(t * t / 2);
        clothoid_points.push_back(std::make_pair(x, y));
        t += dt;
    }

    return clothoid_points;
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





Point reflect_point_across_normal(Point point, Point merging_point) {
    double x = point.x;
    double y = point.y;
    double x_merge = merging_point.x;
    double y_merge = merging_point.y;
    double tangent_angle = M_PI / 2 - atan2(y_merge, -x_merge);
    double normal_x = sin(tangent_angle);
    double normal_y = cos(tangent_angle);

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
    return {reflected_x, reflected_y};
}

std::vector<Point> symmetric_clothoid(const std::vector<Point>& first_clothoid_points) {
    Point merging_point = first_clothoid_points.back();

    std::vector<Point> symmetric_clothoid_points;
    for (auto it = first_clothoid_points.rbegin(); it != first_clothoid_points.rend(); ++it) {
        Point point = reflect_point_across_normal(*it, merging_point);
        symmetric_clothoid_points.push_back(point);
    }

    return symmetric_clothoid_points;
}








/////////////////////////////////////////














/////////////////Turning Points/////////////////////////











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













































































int main() {
    double clothoid_length = 5.0; // Set the desired length of the clothoid
    std::vector<double> x_coordinates;
    std::vector<double> y_coordinates;

    calculateClothoid(clothoid_length, x_coordinates, y_coordinates);

    // Print the calculated coordinates
    for (size_t i = 0; i < x_coordinates.size(); ++i) {
        std::cout << "x[" << i << "] = " << x_coordinates[i] << ", y[" << i << "] = " << y_coordinates[i] << std::endl;
    }

    return 0;
}























private:
  ros::NodeHandle nh_;
  ros::Subscriber global_plan_sub_;
  ros::Publisher smoothed_path_pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_smoothing_node");
  PathSmoothingNode path_smoothing_node;

  ros::spin();

  return 0;
}