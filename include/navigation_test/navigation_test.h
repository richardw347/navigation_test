#ifndef NAVIGATION_TEST_H
#define NAVIGATION_TEST_H

#include <random>
#include <unordered_set>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetPlan.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NavigationTest{
public:
    NavigationTest();
    ~NavigationTest();
    void runTests();
    bool costMapInit();
    void mapToWorld(uint32_t map_x, uint32_t map_y, double& world_x, double& world_y);
    void targetMarker(double x, double y);
    void currentPositionCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& location);
    void generateTarget();
    bool validateGoal();
    bool executeGoal();
    bool check_map_occupancy(int x, int y);
private:
    ros::NodeHandle nh_;
    ros::Subscriber robot_pose_sub_;
    MoveBaseClient mb_action_client_;
    std::vector<int8_t> map_data_;
    uint32_t map_size_x_;
    uint32_t map_size_y_;
    float map_resolution_;
    double map_origin_x_;
    double map_origin_y_;
    geometry_msgs::PoseWithCovarianceStamped current_position_;
    geometry_msgs::PoseStamped current_goal_;
    double distance_threshold_;
    double robot_radius_;
    int heading_bins_;
    int num_tests_;
};

#endif  // NAVIGATION_TEST_H