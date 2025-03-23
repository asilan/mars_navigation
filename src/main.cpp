#include <ros/ros.h>
#include <mars_navigation/mars_navigation_node.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "mars_navigation");
    ros::NodeHandle nh;
    
    // 创建异步spinner，使用4个线程
    ros::AsyncSpinner spinner(4);
    
    try {
        MarsNavigationNode navigation(nh);
        
        // 启动异步spinner
        spinner.start();
        
        // 等待关闭
        ros::waitForShutdown();
    } catch (const std::exception& e) {
        ROS_ERROR("Mars navigation node threw an exception: %s", e.what());
        return 1;
    } catch (...) {
        ROS_ERROR("Mars navigation node threw an unknown exception");
        return 1;
    }
    
    return 0;
} 