//#include "path_smoothing_clothoids_pkg/include/path_smoothing_clothoids.h"
//#include <boost/make_shared.hpp>
//#include <boost/range/adaptor/sliced.hpp>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <tuple>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <utility>
#include <stack>




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
        nav_msgs::Path smoothed_global_path = clothoidInterpolation(global_plan_msg);

        // Publish the smoothed path
        ROS_INFO("Received global plan with %zu poses", global_plan_msg->poses.size());
        for (const auto& poseStamped : smoothed_global_path.poses) {
        const auto& position = poseStamped.pose.position;
        ROS_INFO("X: %f, Y: %f, Z: %f", position.x, position.y, position.z);
        }
        smoothed_path_pub_.publish(smoothed_global_path);
        

    }


    private:
    ros::NodeHandle nh_;
    ros::Subscriber global_plan_sub_;
    ros::Publisher smoothed_path_pub_;

    nav_msgs::Path clothoidInterpolation(const nav_msgs::Path::ConstPtr& global_plan_msg)
    {
    
        double epsilon = 0.1; //The maximum distance between the original path and the simplified path
        double max_angle = 120; //The maximum angle between the two consecutive points of the original path
        double s_length = 5.0; //The length of the clothoid in the lookup table
        nav_msgs::Path smoothed_global_path;
        int connecting_points_start_index = 0;
        int connecting_points_end_index = 0;








        //The lookup table of a basic Clthoid, so that the Fresnal integrals are only solved once here
        std::vector<std::pair<double, double>> Basic_Clothoid = Calculate_Basic_Clothoid(s_length);

        //const auto& simplified_result = rdp_with_indices(global_plan_msg, epsilon, max_angle);

    
        //const std::vector<Point>& simplified_points = simplified_result.first;
        //const std::vector<int>& turning_point_indices = simplified_result.second;

        std::vector<std::pair<Point, int>> turningPoints_idx = findTurningPoints(global_plan_msg);
        std::vector<Point> simplified_points;
        std::vector<int> turning_point_indices;

        for (const auto& tp : turningPoints_idx) {
        simplified_points.push_back(tp.first);
        turning_point_indices.push_back(tp.second);
    }




        // Iterate through the turning points (excluding the first and last points as they are already present in the global plan)
        for (size_t i = 1; i < turning_point_indices.size() - 1; ++i)
        {
            int turning_point_index = turning_point_indices[i];
            const Point& turning_point = simplified_points[turning_point_index];

            int start_index = turning_point_indices[i - 2];
            const geometry_msgs::Point& start_point = global_plan_msg->poses[start_index].pose.position;
        
            // Identify goal_point (one index after the turning_point)
            int goal_index = turning_point_indices[i + 2];
            const geometry_msgs::Point& goal_point = global_plan_msg->poses[goal_index].pose.position;


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
            double Phi_C2 = atan2(transformed_goal_y, transformed_goal_x) * 0.5;
            double s_BC = sqrt(2 * Phi_C2);


            //Now take from Basic Clothoid the part with length s_BC
            std::vector<std::pair<double, double>> subset_basic_clothoid = Subset_Basic_Clothoid(Basic_Clothoid,s_BC);

            //Last point of the subset clothoid
            double last_x = subset_basic_clothoid.back().first;
            double last_y = subset_basic_clothoid.back().second;

            double d_BC = last_x + last_y * tan(Phi_C2);
            double e_BC = last_y / cos(Phi_C2);

            double Scale_Ratio = d_BC/d_max;  //The scale Ratio 

            double s_final = s_BC/Scale_Ratio; //The final length of the clothoid

            double c_sharpness = Scale_Ratio * Scale_Ratio; //The sharpness of the clothoid

            std::vector<std::pair<double, double>> first_clothoid_points = Subset_Basic_Clothoid(Basic_Clothoid,s_final); //first Clothoid with final length

            for (auto& point : first_clothoid_points) 
            {
            point.first *= c_sharpness;
            point.second *= c_sharpness;
            }

        
            //Now merge the first clothoid with its symmetric clothoid

            //Is the merged clothoid clockwise or counter-clockwise?
            double determinant = (start_point.x * turning_point.y + start_point.y * goal_point.x + turning_point.x * goal_point.y) - (goal_point.x * turning_point.y + goal_point.y * start_point.x + turning_point.x * start_point.y);
            // Determinant of Matrix | x1 y1 1 |
            //                       | x2 y2 1 |
            //                       | x3 y3 1 |
            // Alternatively use this tangent angle  Phi_C2 =   
            std::vector<std::pair<double, double>> merged_symmetric_clothoid;
            if (determinant>0)
            {
                std::vector<std::pair<double, double>> merged_symmetric_clothoid = Merged_symmetric_clothoids(first_clothoid_points);
            }
            else
            {
                std::vector<std::pair<double, double>> merged_symmetric_clothoid = Clockwise_Merged_Clothoid(Merged_symmetric_clothoids(first_clothoid_points));
                
            }

            
            std::vector<std::pair<double, double>> transformed_points = TransformSymmetricClothoid(merged_symmetric_clothoid, theta ,  start_point);

            
            connecting_points_end_index = start_index;
            

            for (int i = connecting_points_start_index; i <= connecting_points_end_index; ++i)
            {
                smoothed_global_path.poses.push_back(global_plan_msg->poses[i]);
            }

            for (const auto& point : transformed_points)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = point.first;
                pose.pose.position.y = point.second;
                smoothed_global_path.poses.push_back(pose);
            }

            connecting_points_start_index = goal_index;


        }

        return smoothed_global_path;

    }

  
    struct Point 
    {
        double x, y;
        int index; // To store the index of the point in the original path.

        Point(double x, double y, int index) : x(x), y(y), index(index) {}
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
    
    struct Pose
    {
        Point position;
    
    };

    struct Path
    {
        std::vector<Pose> poses;
    
    };
    





    // Function to calculate x and y coordinates of a basic clothoid, zero initial heading, zero inital curvature, scaling factor = 1, starting at the origin

    std::vector<std::pair<double, double>> Calculate_Basic_Clothoid(double s_length) 
    {
        std::vector<std::pair<double, double>> clothoid_points;
        double dt = 0.1; // Step size for numerical integration
        double t = 0.0;
        double x = 0.0;
        double y = 0.0;
        clothoid_points.push_back(std::make_pair(x, y));
        // Trapezoid Rule for integration
        while (t <= s_length) 
        {
            x += dt * 0.5 * (std::cos(t * t / 2) + std::cos((t + dt) * (t + dt) / 2));
            y += dt * 0.5 * (std::sin(t * t / 2) + std::sin((t + dt) * (t + dt) / 2));
            clothoid_points.push_back(std::make_pair(x, y));
            t += dt;
        }
    
        return clothoid_points;
    }


    std::pair<double, double> Reflect_point_across_normal(std::pair<double, double> point, std::pair<double, double> merging_point, double tangent_angle) 
    {
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

    std::vector<std::pair<double, double>> Merged_symmetric_clothoids(const std::vector<std::pair<double, double>>& first_clothoid_points) 
    {
 

        std::pair<double, double> merging_point = first_clothoid_points.back();
        std::pair<double, double> before_merg = first_clothoid_points[first_clothoid_points.size() - 2];
        double pseudo_tangent_x = before_merg.first - merging_point.first;
        double pseudo_tangent_y = before_merg.second - merging_point.second;
        double tangent_angle = atan2(pseudo_tangent_y, pseudo_tangent_x);

        std::vector<std::pair<double, double>> symmetric_clothoid_points;
        for (auto it = first_clothoid_points.rbegin(); it != first_clothoid_points.rend(); ++it) 
        {
            symmetric_clothoid_points.push_back(Reflect_point_across_normal(*it, merging_point, tangent_angle));
        }

        std::vector<std::pair<double, double>> merged_clothoid_points = first_clothoid_points;
        merged_clothoid_points.insert(merged_clothoid_points.end(), symmetric_clothoid_points.begin(), symmetric_clothoid_points.end());

        return merged_clothoid_points;
    }
















    int orientation(Point p, Point q, Point r) {
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (val == 0) return 0; // Collinear
    return (val > 0) ? 1 : 2; // Clockwise or Counterclockwise
    }

    // Comparator lambda function to sort points based on polar angle with respect to the reference point.
bool polarOrderComparator(const Point& p0, const Point& p1, const Point& p2) {
    int o = orientation(p0, p1, p2);
    if (o == 0) // If collinear, choose the one closest to the reference point.
        return (p1.x - p0.x)*(p1.x - p0.x) + (p1.y - p0.y)*(p1.y - p0.y) <
               (p2.x - p0.x)*(p2.x - p0.x) + (p2.y - p0.y)*(p2.y - p0.y);

    return (o == 2); // Sort in counterclockwise order.
}


std::vector<Point> convexHull(std::vector<Point>& points) {
    int n = points.size();
    if (n <= 3) return points;

    // Find the point with the lowest y-coordinate (and leftmost if there are ties).
    int minY = points[0].y, minIndex = 0;
    for (int i = 1; i < n; i++) {
        int y = points[i].y;
        if ((y < minY) || (y == minY && points[i].x < points[minIndex].x)) {
            minY = y;
            minIndex = i;
        }
    }

    // Place the point with the lowest y-coordinate at the beginning of the vector.
    std::swap(points[0], points[minIndex]);

    // Sort the points based on polar angle with respect to the reference point (points[0]).
    std::sort(points.begin() + 1, points.end(), [&](const Point& p1, const Point& p2) {
        return polarOrderComparator(points[0], p1, p2);
    });

    // Graham's scan algorithm to find the convex hull.
    std::stack<Point> hullStack;
    hullStack.push(points[0]);
    hullStack.push(points[1]);

    for (int i = 2; i < n; i++) {
        while (hullStack.size() >= 2) {
            Point p1 = hullStack.top();
            hullStack.pop();
            Point p2 = hullStack.top();
            if (orientation(p2, p1, points[i]) == 1) {
                hullStack.push(p1);
                break;
            }
        }
        hullStack.push(points[i]);
    }

    // Convert the stack to a vector and return the convex hull.
    std::vector<Point> convexHullPoints;
    while (!hullStack.empty()) {
        convexHullPoints.push_back(hullStack.top());
        hullStack.pop();
    }

    return convexHullPoints;
}

std::vector<std::pair<Point, int>> findTurningPoints(const nav_msgs::Path::ConstPtr& global_plan_msg) {
    std::vector<Point> points;
    int index = 0;

    // Extract points from the global path message and store them in the 'points' vector.
    for (const auto& pose : global_plan_msg->poses) {
        double x = pose.pose.position.x;
        double y = pose.pose.position.y;
        points.emplace_back(x, y, index++);
    }

    // Find the convex hull of the points.
    std::vector<Point> convexHullPoints = convexHull(points);

    // Convert the convex hull points to turning points and their indices in the global path.
    std::vector<std::pair<Point, int>> turningPoints;
    for (const auto& p : convexHullPoints) {
        turningPoints.emplace_back(p, p.index);
    }

    return turningPoints;
}






 /*

    double distance(const Point& a, const Point& b) 
    {
        return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }

    double angle(const Point& a, const Point& b, const Point& c) 
    {
        double v1_x = a.x - b.x;
        double v1_y = a.y - b.y;
        double v2_x = c.x - b.x;
        double v2_y = c.y - b.y;

        double dot_product = v1_x * v2_x + v1_y * v2_y;
        double v1_length = sqrt(v1_x * v1_x + v1_y * v1_y);
        double v2_length = sqrt(v2_x * v2_x + v2_y * v2_y);

        return acos(dot_product / (v1_length * v2_length));
    }



    std::pair<std::vector<Point>, std::vector<int>> rdp_with_indices(const nav_msgs::Path::ConstPtr& global_plan_msg, double epsilon, double max_angle) 
{
    std::vector<Point> points;
    std::vector<int> indices;

    const nav_msgs::Path& global_plan = *global_plan_msg;

    for (size_t i = 0; i < global_plan.poses.size(); ++i) 
    {
        const auto& pose = global_plan.poses[i].pose;
        points.push_back({pose.position.x, pose.position.y});
        indices.push_back(i);
    }

    double dmax = 0.0;
    int index = 0;

    for (size_t i = 1; i < points.size() - 1; ++i) 
    {
        double a = angle(points[i - 1], points[i], points[i + 1]);
        if (a < max_angle) 
        {
            double d = distance(points[i], points[0]) + distance(points[i], points.back());
            if (d > dmax) 
            {
                index = static_cast<int>(i);
                dmax = d;
            }
        }
    }

    std::pair<std::vector<Point>, std::vector<int>> result;

    if (dmax >= epsilon) 
    {
        // Recursively simplify the points and indices for the left segment
        auto left_segment = std::make_shared<nav_msgs::Path>();
        left_segment->poses.insert(left_segment->poses.end(), global_plan.poses.begin(), global_plan.poses.begin() + index + 1);
        auto simplified_left = rdp_with_indices(left_segment, epsilon, max_angle);

        // Recursively simplify the points and indices for the right segment
        auto right_segment = std::make_shared<nav_msgs::Path>();
        right_segment->poses.insert(right_segment->poses.end(), global_plan.poses.begin() + index, global_plan.poses.end());
        auto simplified_right = rdp_with_indices(right_segment, epsilon, max_angle);

        // Combine the results by excluding the last point from the left segment to avoid duplication
        result.first = std::vector<Point>(simplified_left.first.begin(), simplified_left.first.end() - 1);
        result.second = std::vector<int>(simplified_left.second.begin(), simplified_left.second.end() - 1);
        result.first.insert(result.first.end(), simplified_right.first.begin(), simplified_right.first.end());
        result.second.insert(result.second.end(), simplified_right.second.begin(), simplified_right.second.end());
    } 
    else    
    {
        // If the maximum perpendicular distance is less than epsilon, keep only the first and last points
        result.first = {points.front(), points.back()};
        result.second = {indices.front(), indices.back()};
    }

    return result;
}

*/   

    /*
    std::pair<std::vector<Point>, std::vector<int>> rdp_with_indices(const nav_msgs::Path::ConstPtr& global_plan_msg, double epsilon, double max_angle) 
    {
        std::vector<Point> points;
        std::vector<int> indices;
        for (size_t i = 0; i < global_plan_msg->poses.size(); ++i) 
        {
            const auto& pose = global_plan_msg->poses[i].pose;
            points.push_back({pose.position.x, pose.position.y});
            indices.push_back(i);
        }

        double dmax = 0.0;
        int index = 0;

        for (size_t i = 1; i < points.size() - 1; ++i) 
        {
            double a = angle(points[i - 1], points[i], points[i + 1]);
            if (a < max_angle) 
            {
                double d = distance(points[i], points[0]) + distance(points[i], points.back());
                if (d > dmax) 
                {
                    index = static_cast<int>(i);
                    dmax = d;
                }
            }
        }

        std::pair<std::vector<Point>, std::vector<int>> result;

        if (dmax >= epsilon) 
        {
            // Recursively simplify the points and indices for the left segment
            auto simplified_left = rdp_with_indices(boost::make_shared<const nav_msgs::Path>(*global_plan_msg | boost::adaptors::sliced(0, index + 1)), epsilon, max_angle);

            // Recursively simplify the points and indices for the right segment
            auto simplified_right = rdp_with_indices(boost::make_shared<const nav_msgs::Path>(*global_plan_msg | boost::adaptors::sliced(index, global_plan_msg->poses.size())), epsilon, max_angle);

        

            // Combine the results by excluding the last point from the left segment to avoid duplication
            result.first = std::vector<Point>(simplified_left.first.begin(), simplified_left.first.end() - 1);
            result.second = std::vector<int>(simplified_left.second.begin(), simplified_left.second.end() - 1);
            result.first.insert(result.first.end(), simplified_right.first.begin(), simplified_right.first.end());
            result.second.insert(result.second.end(), simplified_right.second.begin(), simplified_right.second.end());
        } 
        else    
        {
            // If the maximum perpendicular distance is less than epsilon, keep only the first and last points
            result.first = {points.front(), points.back()};
            result.second = {indices.front(), indices.back()};
        }

        return result;
    }
    */
    

    std::vector<std::pair<double, double>> Clockwise_Merged_Clothoid(const std::vector<std::pair<double, double>>& merged_clothoid_points) 
    {
        std::vector<std::pair<double, double>> clockwise_merged_clothoid;
        for (const auto& point : merged_clothoid_points) {
        double point_x = point.first;
        double point_y = -point.second;
        clockwise_merged_clothoid.push_back(std::make_pair(point_x, point_y));
    }

    return clockwise_merged_clothoid;
    }

    std::vector<std::pair<double, double>> Subset_Basic_Clothoid(const std::vector<std::pair<double, double>>& Basic_Clothoid, double s_BC) 
    {
        std::vector<std::pair<double, double>> subset_clothoid_points;

        // Ensure s_BC is within the range of Basic_Clothoid
        s_BC = std::min(s_BC, static_cast<double>((Basic_Clothoid.size() - 1) * 0.1));

        // Number of points to be extracted from the lookup table
        int num_points = static_cast<int>(s_BC / 0.1);

        // Extract the desired number of points
        for (int i = 0; i <= num_points; ++i) 
        {
        subset_clothoid_points.push_back(Basic_Clothoid[i]);
        }

        return subset_clothoid_points;
    }



    std::pair<double, double> rotatePoint(double x, double y, double theta) 
    {
        double cosTheta = std::cos(theta);
        double sinTheta = std::sin(theta);
        double xRotated = x * cosTheta - y * sinTheta;
        double yRotated = x * sinTheta + y * cosTheta;
        return std::make_pair(xRotated, yRotated);
    }

    std::vector<std::pair<double, double>> TransformSymmetricClothoid(
    const std::vector<std::pair<double, double>>& merged_symmetric_clothoid,
    double theta, const geometry_msgs::Point& start_point) 
    {

        std::vector<std::pair<double, double>> transformed_clothoid;

        if (merged_symmetric_clothoid.empty())
            return transformed_clothoid;

        double x0 = merged_symmetric_clothoid[0].first;
        double y0 = merged_symmetric_clothoid[0].second;

        double deltaX = start_point.x - x0;
        double deltaY = start_point.y - y0;

        for (const auto& point : merged_symmetric_clothoid) 
        {
            double xRotated, yRotated;
            std::tie(xRotated, yRotated) = rotatePoint(point.first - x0, point.second - y0, -theta);
            double xTransformed = xRotated + start_point.x + deltaX;
            double yTransformed = yRotated + start_point.y + deltaY;
            transformed_clothoid.emplace_back(xTransformed, yTransformed);
        }

        return transformed_clothoid;
    }


    
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_smoothing_node");
  PathSmoothingNode path_smoothing_node;



  ros::spin();

  return 0;
}