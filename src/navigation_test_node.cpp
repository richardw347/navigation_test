#include <ros/ros.h>
#include <navigation_test.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation_test");
    NavigationTest nav_test;
    ROS_INFO("Navigation test is running!");
    nav_test.runTests();
    return 0;
}
