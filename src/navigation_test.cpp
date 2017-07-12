#include "navigation_test.h"

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

NavigationTest::NavigationTest() : nh_("~"), mb_action_client_("move_base", true) {
    if (costMapInit()) {
        std::cout << "Cost map properly initialized." << std::endl;
    } else {
        std::cout << "Issues initializing cost map." << std::endl;
        ros::shutdown();
    }
    robot_pose_sub_ = nh_.subscribe("/amcl_pose", 10, &NavigationTest::currentPositionCB, this);

    nh_.param("distance_threshold", distance_threshold_, 8.0);
    nh_.param("heading_bins_", heading_bins_, 20);
    nh_.param("number_of_tests", num_tests_, 4);
    nh_.param("robot_radius", robot_radius_, 0.6);
};

NavigationTest::~NavigationTest(){

};

bool NavigationTest::costMapInit() {
    while (!ros::service::waitForService("static_map", ros::Duration(2.0))) {
        ROS_INFO("Waiting for static_map");
    }

    nav_msgs::GetMap srv_map;
    if (ros::service::call("static_map", srv_map)) {
        ROS_INFO("Got costmap");
        map_origin_x_ = srv_map.response.map.info.origin.position.x;
        map_origin_y_ = srv_map.response.map.info.origin.position.y;
        map_resolution_ = srv_map.response.map.info.resolution;
        map_size_x_ = srv_map.response.map.info.width;
        map_size_y_ = srv_map.response.map.info.height;
        map_data_ = srv_map.response.map.data;
        return true;
    }
    ROS_INFO("Service call failed");
    return false;
}

void NavigationTest::currentPositionCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &location) {
    current_position_ = *location;
}

void NavigationTest::mapToWorld(uint32_t map_x, uint32_t map_y, double &world_x, double &world_y) {
    world_x = map_origin_x_ + (map_x + 0.5) * map_resolution_;
    world_y = map_origin_y_ + (map_y + 0.5) * map_resolution_;
}

void NavigationTest::runTests(){
    ros::Rate loop_rate(10);
    int current_test = 1;
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        if (current_test <=  num_tests_){
            ros::Time time_start = ros::Time::now();
            bool goal_found = false;
            ROS_INFO("Starting test %d of %d", current_test, num_tests_);
            while (ros::ok()){
                this->generateTarget();
                if (this->validateGoal()){
                    goal_found = true;
                    break;
                }
                if ((ros::Time::now() - time_start) > ros::Duration(5.0)){
                    break;
                }
            }
            if (goal_found){
                ROS_INFO_STREAM("Goal: " << current_goal_);
                this->executeGoal();
                current_test++;
            }

        } else {
            ROS_INFO("All tests completed!");
            break;
        }
    }
}

bool NavigationTest::validateGoal(){
    nav_msgs::GetPlan plan_req;
    while (!ros::service::waitForService("/move_base_node/GlobalPlanner/make_plan", ros::Duration(2.0))) {
        ROS_INFO("Waiting for static_map");
    }
    geometry_msgs::PoseStamped start_p;
    plan_req.request.start.header.stamp = ros::Time::now();
    plan_req.request.start.header.frame_id = "map";
    plan_req.request.start.pose = current_position_.pose.pose;
    plan_req.request.goal = current_goal_;

    if (ros::service::call("/move_base_node/GlobalPlanner/make_plan", plan_req)) {
        if (plan_req.response.plan.poses.size() > 0){
            ROS_INFO("Valid plan found.");
            return true;
        } else {
            ROS_WARN("Couldn't find a valid plan to goal.");
        }
    }
    return false;
}

bool NavigationTest::check_map_occupancy(int map_x, int map_y){
    std::vector<int> points_to_check;
    int radius_cells = (int) (robot_radius_ / map_resolution_);
    for (int i=map_x-radius_cells; i<(map_x+radius_cells); i++){
        for (int j=map_y-radius_cells; j<(map_y + radius_cells); j++){
            int idx = i + j * map_size_x_;
            if (0 < idx < map_data_.size()){
                if (map_data_[idx] != 0){
                    return false;
                }
            }

        }
    }
    return true;
}

void NavigationTest::generateTarget() {
    std::random_device rd;
    std::mt19937 gen(rd());

    // Use 5 as buffer for map boundaries, as want to avoid edge cases.
    std::uniform_int_distribution<uint32_t> grid_x(5, map_size_x_ - 5);
    std::uniform_int_distribution<uint32_t> grid_y(5, map_size_y_ - 5);

    double world_x, world_y;
    bool thresh;
    auto checkThresh = [&](double x, double y, double wx, double wy) {
        return sqrt(pow(wx - x, 2) + pow(wy - y, 2)) > distance_threshold_;
    };

    while (true) {
        uint32_t map_x = grid_x(gen);
        uint32_t map_y = grid_y(gen);

        mapToWorld(map_x, map_y, world_x, world_y);

        thresh = checkThresh(current_position_.pose.pose.position.x,
                             current_position_.pose.pose.position.y,
                             world_x,
                             world_y);

        if ((thresh == 1) && check_map_occupancy(map_x, map_y)){
            break;
        }

        if (!ros::ok()){
            ros::shutdown();
        }

    }

    std::uniform_int_distribution<int> heading(1, heading_bins_);
    int rnd_heading_bin = heading(gen);
    double tgt_angle = ((M_PI*2) / heading_bins_) * rnd_heading_bin;

    if (tgt_angle >= M_PI) {
        tgt_angle -= (M_PI * 2.0);
    } else if (tgt_angle < -M_PI) {
        tgt_angle += (M_PI * 2.0);
    }

    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(tgt_angle);
    geometry_msgs::Quaternion q_msg;
    tf::quaternionTFToMsg(quaternion, q_msg);

    current_goal_.header.stamp = ros::Time::now();
    current_goal_.header.frame_id = "map";
    current_goal_.pose.position.x = world_x;
    current_goal_.pose.position.y = world_y;
    current_goal_.pose.orientation = q_msg;
};

bool NavigationTest::executeGoal(){
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = current_goal_;
    ROS_INFO("Executing goal");

    mb_action_client_.waitForServer();
    mb_action_client_.sendGoal(goal);

    mb_action_client_.waitForResult();

    if (mb_action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("You have reached the goal!");
        return true;
    } else{
        ROS_INFO("The base failed for some reason");
        return false;
    }
};
