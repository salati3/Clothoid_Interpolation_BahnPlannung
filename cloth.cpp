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
#include <stdexcept>
#include <visualization_msgs/Marker.h>



class PathSmoothingNode
{
    public:
    PathSmoothingNode()
    {
        // Subscribe to the global plan topic
        global_plan_sub_ = nh_.subscribe("/move_base_node/NavfnROS/plan", 1,
                                    &PathSmoothingNode::globalPlanCallback, this);

        // Publish the smoothed path to rviz
        smoothed_path_pub_ = nh_.advertise<nav_msgs::Path>("/rviz_smoothed_path_clothoid", 10);

        
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("turning_points_markers", 10);

    }


       


    void globalPlanCallback(const nav_msgs::Path::ConstPtr& global_plan_msg)
    {
        float epsilon = 0.05; //The maximum distance between the original path and the simplified path  -> larger epsilon -> less points -> cruder path
        float max_angle = 35; //The maximum angle between the two consecutive points of the original path
        

        std::vector<std::pair<std::pair<float, float>, int>> global_path_vector = convertPathToVector(global_plan_msg);
        
        std::vector<std::pair<std::pair<float, float>, int>> simplified_path_indeces = rdp(global_path_vector, epsilon);

        std::vector<std::pair<std::pair<float, float>, int>> turning_points_indices = findTurningPoints( simplified_path_indeces,  max_angle);

        nav_msgs::Path smoothed_global_path = clothoidInterpolation( global_path_vector);

        smoothed_global_path.header.frame_id = "map";


        //Publish gloal smoothed path
        smoothed_path_pub_.publish(smoothed_global_path);



        // Publish the smoothed path
        ROS_INFO("Received global plan with %zu poses", global_plan_msg->poses.size());

        
        //Print the global path vector
         for (auto& point : global_path_vector) {
        
        std::cout << "At index " << point.second << " Global Path x (" << point.first.first << " y " << point.first.second << ") \n";
        }
        
        /*

        //Print the simplified points and their indices
        for (const auto& tp : simplified_path_indeces) {
        std::cout << "Simplified point at index " << tp.second << " with coordinates (" << tp.first.first << ", " << tp.first.second << ")\n";
        }

        */

        //Print the turning points and their indices
        for (const auto& tp : turning_points_indices) {
        std::cout << "Turning Point at index " << tp.second << " with coordinates (" << tp.first.first << ", " << tp.first.second << ")\n";
        }


        /*
        //Print the global smoothed path i terminal:
        ROS_INFO("Smoothed Global Path:");

        for (const geometry_msgs::PoseStamped& pose_stamped : smoothed_global_path.poses) {
            const geometry_msgs::Point& position = pose_stamped.pose.position;
            ROS_INFO("X: %f, Y: %f", position.x, position.y);
        }
        */


        //This to visualize the turning points in rviz
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";  // Assuming you want to visualize points in the "map" frame. Change it to your desired frame.
            marker.header.stamp = ros::Time::now();
            marker.ns = "turning_points";
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::POINTS;
            marker.id = 0;
            marker.scale.x = 0.08;  // Adjust the size of the markers as needed
            marker.scale.y = 0.08;
            marker.color.r = 1.0;  // Marker color (red in this case)
            marker.color.a = 1.0;  // Marker alpha (fully opaque)

            // Assuming turning_points_indices is a vector of pairs where each pair contains the x, y coordinates
            for (const auto& point : turning_points_indices)
            {
                geometry_msgs::Point p;
                p.x = point.first.first;
                p.y = point.first.second;
                p.z = 0;  // Assuming you want to place the points in the z=0 plane

                marker.points.push_back(p);
            }

            marker_pub_.publish(marker);

        
        

    }


    private:
    ros::NodeHandle nh_;
    ros::Subscriber global_plan_sub_;
    ros::Publisher smoothed_path_pub_;
    ros::Publisher marker_pub_;


    std::vector<std::pair<std::pair<float, float>, int>> convertPathToVector(const nav_msgs::Path::ConstPtr& global_plan_msg) 
    {
        std::vector<std::pair<std::pair<float, float>, int>> global_path_vector;
        const nav_msgs::Path& globalPath = *global_plan_msg;

        for (size_t i = 0; i < globalPath.poses.size(); ++i) 
        {
            const auto& pose = globalPath.poses[i];
            float x = pose.pose.position.x;
            float y = pose.pose.position.y;
            global_path_vector.emplace_back(std::make_pair(x, y), i);
        }

        return global_path_vector;
    }





    nav_msgs::Path clothoidInterpolation(std::vector<std::pair<std::pair<float, float>, int>> global_path_vector2)
    {
        std::vector<std::pair<std::pair<float, float>, int>> global_path_vector = global_path_vector2;
        float epsilon = 0.1; //The maximum distance between the original path and the simplified path
        float max_angle = 30; //The maximum angle between the two consecutive points of the original path
        float s_length = 5.0; //The length of the clothoid in the lookup table
        float d_max = 0.3; // The maximum distance between ST or TG
        nav_msgs::Path smoothed_global_path;
        nav_msgs::Path smoothed_global_path2;
        int connecting_points_start_index = 0;
        int connecting_points_end_index = 0;
        std::vector<std::pair<float, float>> turning_points;
        std::vector<int> turning_point_indices;
        std::vector<std::pair<float, float>> merged_symmetric_clothoid;
        std::vector<std::pair<float, float>> transformed_points;








        //The lookup table of a basic Clthoid, so that the Fresnal integrals are only solved once here, Test passed
        std::vector<std::pair<float, float>> Basic_Clothoid = Calculate_Basic_Clothoid(s_length);


        


        // RDP to simplify the path and findTurningPoints to find the turning points, rdp function test passed  and findTurningPoints test passed
        std::vector<std::pair<std::pair<float, float>, int>> simplified_path_indeces = rdp(global_path_vector, epsilon);

        std::vector<std::pair<std::pair<float, float>, int>> turning_points_and_indices = findTurningPoints( simplified_path_indeces,  max_angle);



 

        // Create a vector of the turning points and another the indeces, test passed
        for (const auto& tp : turning_points_and_indices) {
        turning_points.push_back(tp.first);
        turning_point_indices.push_back(tp.second);
        }


        

        

        
        // Iterate through the turning points,  test in progress
        for (size_t i = 0; i < turning_point_indices.size(); ++i)
        {


            ///////Find connecting points and their indeces, test bestanden
            int turning_point_index = turning_point_indices[i];
            std::pair<float, float> turning_point = global_path_vector[turning_point_index].first;

            int start_index = turning_point_indices[i]-3;
            std::pair<float, float> start_point = global_path_vector[start_index].first;
        
            
            int goal_index = turning_point_indices[i] +3;
            std::pair<float, float> goal_point = global_path_vector[goal_index].first;
        
            std::cout << "Iteration " << i << "\n";
            std::cout << "TESTT1 : Turning point x:" << turning_point.first << " y:" << turning_point.second << " with index  " << turning_point_index << ")\n";
            std::cout << "TESTT1 : Start point x:" << start_point.first << " y:" << start_point.second << " with index  " << start_index<< ")\n";
            std::cout << "TESTT1 : Goal point x:" << goal_point.first << " y:" << goal_point.second << " with index  " << goal_index << ")\n";



        

        


            //Transform the points so that Starting point is at Orign and the Turning Point is at the positive xAxis , test bestanden
            float translate_x = -start_point.first;
            float translate_y = -start_point.second;

            



            // Step 2: Calculate the angle between the turning point and the positive x-axis, test bestanden
            float theta_rad = atan2(turning_point.second - start_point.second, turning_point.first - start_point.first);
            float theta_deg = theta_rad * 180 / M_PI;


            




            // Step 3: Rotate only the three points (start_point, turning_point, and goal_point) by the negative of that angle to align the turning point with the positive x-axis, test bestanden
            float transformed_start_x = cos(-theta_rad) * (start_point.first + translate_x) - sin(-theta_rad) * (start_point.second + translate_y);
            float transformed_start_y = sin(-theta_rad) * (start_point.first + translate_x) + cos(-theta_rad) * (start_point.second + translate_y);

            float transformed_turning_x = cos(-theta_rad) * (turning_point.first + translate_x) - sin(-theta_rad) * (turning_point.second + translate_y);
            float transformed_turning_y = sin(-theta_rad) * (turning_point.first + translate_x) + cos(-theta_rad) * (turning_point.second + translate_y);

            float transformed_goal_x = cos(-theta_rad) * (goal_point.first + translate_x) - sin(-theta_rad) * (goal_point.second + translate_y);
            float transformed_goal_y = sin(-theta_rad) * (goal_point.first + translate_x) + cos(-theta_rad) * (goal_point.second + translate_y);

            



            /////Step 4: Find the length of the basic clothoid for the given point configuration, test bestanden
            float distanceStartToTurn = std::sqrt(std::pow(transformed_turning_x - transformed_start_x, 2) + std::pow(transformed_turning_y - transformed_start_y, 2));
            float distanceTurnToGoal = std::sqrt(std::pow(transformed_goal_x - transformed_turning_x, 2) + std::pow(transformed_goal_y - transformed_turning_y, 2));
            float d = std::min({distanceStartToTurn, distanceTurnToGoal, d_max});            
            float Phi_C2 = atan2(transformed_goal_y, transformed_goal_x - transformed_turning_x) * 0.5;
            float s_BC = sqrt(2 * std::abs(Phi_C2));

            std::cout << "Transformed start point: (" << transformed_start_x<< ", " << transformed_start_y << ")" << std::endl;
            std::cout << "Transformed turn point: (" << transformed_turning_x << ", " << transformed_turning_y << ")" << std::endl;
            std::cout << "Transformed goal point: (" << transformed_goal_x << ", " << transformed_goal_y << ")" << std::endl;
            std::cout << "d=" << d << "    PHI_C2= " << Phi_C2 << "      s_BC=" << s_BC << std::endl;
        

        
        

            //Now take from Basic Clothoid the part with length s_BC , test bestanden
            std::vector<std::pair<float, float>> subset_basic_clothoid = Subset_Basic_Clothoid(Basic_Clothoid,s_BC);





            //Last point of the subset clothoid
            float last_x = subset_basic_clothoid.back().first;
            float last_y = subset_basic_clothoid.back().second;

            float d_BC = last_x + last_y * tan(Phi_C2);
            float e_BC = last_y / cos(Phi_C2);

            float Scale_Ratio = d_BC/d;  //The scale Ratio 

            float s_final = s_BC/Scale_Ratio; //The final length of the clothoid

            float c_sharpness = Scale_Ratio * Scale_Ratio; //The sharpness of the clothoid

            std::vector<std::pair<float, float>> first_clothoid_points = Subset_Basic_Clothoid(Basic_Clothoid,s_BC); //first Clothoid with final length

            
            
            
            
            
            //Scale the first clothoid
            
            for (auto& point : first_clothoid_points) 
            {
            point.first /= Scale_Ratio;
            point.second /= Scale_Ratio;
            }
        
        
        
        
            

        
            //Now merge the first clothoid with its symmetric clothoid,test bestanden

            //Is the merged clothoid clockwise or counter-clockwise?
            float determinant = (start_point.first * turning_point.second + start_point.second * goal_point.first + turning_point.first * goal_point.second) - (goal_point.first * turning_point.second + goal_point.second * start_point.first + turning_point.first * start_point.second);
            // Determinant of Matrix | x1 y1 1 |
            //                       | x2 y2 1 |
            //                       | x3 y3 1 |
            // Alternatively use this tangent angle  Phi_C2 =   
            
            if (determinant>0)
            {
            merged_symmetric_clothoid = Merged_symmetric_clothoids(first_clothoid_points);
            }
            else
            {
            merged_symmetric_clothoid = Clockwise_Merged_Clothoid(Merged_symmetric_clothoids(first_clothoid_points));
            }


            for (auto const& point : merged_symmetric_clothoid) 
            { std::cout << "Merged Symmetric Clothoid x: " << point.first << "     and y: " << point.second << std::endl; }
            



            //Final transformed Clothoid Test teilweise bestanden, works needs adjustment
            transformed_points = TransformSymmetricClothoid(merged_symmetric_clothoid, theta_rad ,  start_point);


            for (auto const& point : transformed_points) 
            {

            std::cout << "Final Transformed  x: " << point.first << "     and y: " << point.second << std::endl;

            }

            
            connecting_points_end_index = start_index;
            

            for (int i = connecting_points_start_index; i <= connecting_points_end_index; ++i)
            {
            
            std::pair<std::pair<float, float>, int> data = global_path_vector[i];
            std::pair<float, float> float_pair = data.first;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose.position.x = float_pair.first;
            pose_stamped.pose.position.y = float_pair.second;
            pose_stamped.pose.position.z = 0.0; // Assuming z-coordinate is 0
            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0;
            pose_stamped.header.frame_id = "map";
            

            smoothed_global_path2.poses.push_back(pose_stamped);

            


            }

            for (const auto& point : transformed_points)
            {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose.position.x = point.first;
            pose_stamped.pose.position.y = point.second;
            pose_stamped.pose.position.z = 0.0; // Assuming z-coordinate is 0
            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0;
            smoothed_global_path2.poses.push_back(pose_stamped);
            pose_stamped.header.frame_id = "map";
            
            
            }

            connecting_points_start_index = goal_index;

        }


        for (int i = connecting_points_start_index; i <= global_path_vector.size()-1; ++i)
            {
            
            std::pair<std::pair<float, float>, int> data = global_path_vector[i];
            std::pair<float, float> float_pair = data.first;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose.position.x = float_pair.first;
            pose_stamped.pose.position.y = float_pair.second;
            pose_stamped.pose.position.z = 0.0; // Assuming z-coordinate is 0
            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0;
            pose_stamped.header.frame_id = "map";
            

            smoothed_global_path2.poses.push_back(pose_stamped);

            


            }


    
        return smoothed_global_path2;
        smoothed_global_path2.header.frame_id = "map";


    } 
    



















    // Function to calculate x and y coordinates of a basic clothoid, zero initial heading, zero inital curvature, scaling factor = 1, starting at the origin

    std::vector<std::pair<float, float>> Calculate_Basic_Clothoid(double s_length) 
    {
        std::vector<std::pair<float, float>> clothoid_points;
        float dt = 0.01; // Step size for numerical integration
        float t = 0.0;
        float x = 0.0;
        float y = 0.0;
        clothoid_points.push_back(std::make_pair(x, y));
        // Trapezoid Rule for integration
        while (t <= s_length) 
        {
            x += dt * 0.5 * (cos(t * t / 2) + cos((t + dt) * (t + dt) / 2));
            y += dt * 0.5 * (sin(t * t / 2) + sin((t + dt) * (t + dt) / 2));
            clothoid_points.push_back(std::make_pair(x, y));
            t += dt;
        }
    
        return clothoid_points;
    }


    std::pair<float, float> Reflect_point_across_normal(std::pair<float, float> point, std::pair<float, float> merging_point, float tangent_angle) 
    {
        float x = point.first;
        float y = point.second;
        float x_merge = merging_point.first;
        float y_merge = merging_point.second;
    
        float normal_x = std::cos(M_PI / 2 + tangent_angle);
        float normal_y = std::sin(M_PI / 2 + tangent_angle);

        float dx = x - x_merge;
        float dy = y - y_merge;

        // Calculate the dot product of the vector and the normal
        float dot_product = dx * normal_x + dy * normal_y;

        // Project the vector onto the normal
        float projected_x = dot_product * normal_x;
        float projected_y = dot_product * normal_y;

        float origin_ref_x = projected_x + (projected_x - dx);
        float origin_ref_y = projected_y + (projected_y - dy);

        float reflected_x = origin_ref_x + x_merge;
        float reflected_y = origin_ref_y + y_merge;

        return std::make_pair(reflected_x, reflected_y);


    }






    std::vector<std::pair<float, float>> Merged_symmetric_clothoids(const std::vector<std::pair<float, float>>& first_clothoid_points) 
    {
 

        std::pair<float, float> merging_point = first_clothoid_points.back();
        std::pair<float, float> before_merg = first_clothoid_points[first_clothoid_points.size() - 2];
        float pseudo_tangent_x = before_merg.first - merging_point.first;
        float pseudo_tangent_y = before_merg.second - merging_point.second;
        float tangent_angle = atan2(pseudo_tangent_y, pseudo_tangent_x);

        std::vector<std::pair<float, float>> symmetric_clothoid_points;
        for (auto it = first_clothoid_points.rbegin(); it != first_clothoid_points.rend(); ++it) 
        {
            symmetric_clothoid_points.push_back(Reflect_point_across_normal(*it, merging_point, tangent_angle));
        }

        std::vector<std::pair<float, float>> merged_clothoid_points = first_clothoid_points;
        merged_clothoid_points.insert(merged_clothoid_points.end(), symmetric_clothoid_points.begin(), symmetric_clothoid_points.end());

        return merged_clothoid_points;
    }


 






    

    ///////////////////////// RDP /////////////////////////
 
    float distance(std::pair<float, float> a, std::pair<float, float> b) 
    {
            return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
    }

        
        
    float point_line_distance(std::pair<float, float> point, std::pair<float, float> start, std::pair<float, float> end) 
    {
            if (start == end) {
            return distance(point, start);
            } else {
            float n = abs((end.first - start.first) * (start.second - point.second) -
                      (start.first - point.first) * (end.second - start.second));
            float d = sqrt(pow(end.first - start.first, 2) + pow(end.second - start.second, 2));
            return n / d;
            }
    }

    std::vector<std::pair<std::pair<float, float>, int>> rdp(const std::vector<std::pair<std::pair<float, float>, int>>& points, float epsilon) 
    {
        std::vector<std::pair<std::pair<float, float>, int>> results;
        float dmax = 0.0;
        int index = 0;

        for (int i = 1; i < points.size() - 1; i++) 
        {
            float d = point_line_distance(points[i].first, points[0].first, points[points.size() - 1].first);
            if (d > dmax) {
            index = i;
            dmax = d;
            }
        }

        if (dmax >= epsilon) {
        std::vector<std::pair<std::pair<float, float>, int>> first_half = rdp(std::vector<std::pair<std::pair<float, float>, int>>(points.begin(), points.begin() + index + 1), epsilon);
        std::vector<std::pair<std::pair<float, float>, int>> second_half = rdp(std::vector<std::pair<std::pair<float, float>, int>>(points.begin() + index, points.end()), epsilon);

        results.insert(results.end(), first_half.begin(), first_half.end() - 1);
        results.insert(results.end(), second_half.begin(), second_half.end());
        } 
        else {
        results.push_back(points[0]);
        results.push_back(points[points.size() - 1]);
        }

        return results;
    }










    ///////////////////////// FIND TURNING POINTS /////////////////////////

    // Function to calculate the direction vector between two points
    std::pair<float, float> calculateDirectionVector(const std::pair<float, float>& p1, const std::pair<float, float>& p2)  
    {
        float dx = p2.first - p1.first;
        float dy = p2.second - p1.second;
        return std::make_pair(dx, dy);
    }

    // Function to calculate the angle between two direction vectors in degrees
    float calculateAngle(const std::pair<float, float>& v1, const std::pair<float, float>& v2) 
    {
        float dotProduct = v1.first * v2.first + v1.second * v2.second;
        float magnitudeV1 = std::sqrt(v1.first * v1.first + v1.second * v1.second);
        float magnitudeV2 = std::sqrt(v2.first * v2.first + v2.second * v2.second);
        float cosineTheta = dotProduct / (magnitudeV1 * magnitudeV2);
        return std::acos(cosineTheta) * 180.0f / M_PI;
    }

    // Function to find turning points based on angle threshold
    std::vector<std::pair<std::pair<float, float>, int>> findTurningPoints(const std::vector<std::pair<std::pair<float, float>, int>>& simplified_path_indices, float angleThreshold) 
    {
        std::vector<std::pair<std::pair<float, float>, int>> turningPoints;

        if (simplified_path_indices.size() < 3) {
        return turningPoints; // Not enough points to calculate angles
        }

        for (size_t i = 1; i < simplified_path_indices.size() - 1; ++i) 
        {
            const auto& prevPoint = simplified_path_indices[i - 1].first;
            const auto& currPoint = simplified_path_indices[i].first;
            const auto& nextPoint = simplified_path_indices[i + 1].first;

            // Calculate direction vectors
            auto directionVector1 = calculateDirectionVector(prevPoint, currPoint);
            auto directionVector2 = calculateDirectionVector(currPoint, nextPoint);

            // Calculate angles between the direction vectors
            float angle = calculateAngle(directionVector1, directionVector2);

            // Check if the change in direction exceeds the threshold
            if (angle >= angleThreshold) {
            turningPoints.push_back(simplified_path_indices[i]);
            }
        }

        return turningPoints;
    }































    ///////////////////////// CLOTHOID  INTERPOLATION FUNCTIONS/////////////////////////
 



    std::vector<std::pair<float, float>> Clockwise_Merged_Clothoid(const std::vector<std::pair<float, float>>& merged_clothoid_points) 
    {
        std::vector<std::pair<float, float>> clockwise_merged_clothoid;
        for (const auto& point : merged_clothoid_points) {
        float point_x = point.first;
        float point_y = -point.second;
        clockwise_merged_clothoid.push_back(std::make_pair(point_x, point_y));
        }

        return clockwise_merged_clothoid;
    }







    std::vector<std::pair<float, float>> Subset_Basic_Clothoid(const std::vector<std::pair<float, float>>& Basic_Clothoid, float s_BC) 
    {
        std::vector<std::pair<float, float>> subset_clothoid_points;

        // Ensure s_BC is within the range of Basic_Clothoid
        s_BC = std::min(s_BC, static_cast<float>((Basic_Clothoid.size() - 1) * 0.01));

        // Number of points to be extracted from the lookup table
        int num_points = static_cast<int>(s_BC / 0.01);  //0.1 is the integration step in BasicClothoid

        // Extract the desired number of points
        for (int i = 0; i <= num_points; ++i) 
        {
        subset_clothoid_points.push_back(Basic_Clothoid[i]);
        }

        return subset_clothoid_points;
    }



    std::pair<float, float> rotatePoint(float x, float y, float theta) 
    {
        float cosTheta = std::cos(theta);
        float sinTheta = std::sin(theta);
        float xRotated = x * cosTheta - y * sinTheta;
        float yRotated = x * sinTheta + y * cosTheta;
        return std::make_pair(xRotated, yRotated);
    }

    std::vector<std::pair<float, float>> TransformSymmetricClothoid(
    const std::vector<std::pair<float, float>>& merged_symmetric_clothoid,
    float theta, const std::pair<float,float>& start_point) 
    {

        std::vector<std::pair<float, float>> transformed_clothoid;
        

        if (merged_symmetric_clothoid.empty())
            return transformed_clothoid;

        float x0 = merged_symmetric_clothoid[0].first;
        float y0 = merged_symmetric_clothoid[0].second;

        float deltaX = start_point.first - x0;
        float deltaY = start_point.second - y0;

        for (const auto& point : merged_symmetric_clothoid) 
        {
            float xRotated, yRotated;
            std::pair<float,float> rotated_points = rotatePoint(point.first - x0, point.second - y0, theta);
            float xTransformed = rotated_points.first  + deltaX;
            float yTransformed = rotated_points.second  + deltaY;
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
