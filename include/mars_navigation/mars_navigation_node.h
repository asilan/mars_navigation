#ifndef MARS_NAVIGATION_NODE_H
#define MARS_NAVIGATION_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/BatteryState.h>
#include <mars_canopen/RobotMode.h>
#include <actionlib/server/simple_action_server.h>
#include <mars_navigation/ExecuteMissionAction.h>
#include <mars_navigation/NavigateToTargetAction.h>
#include <mars_navigation/UploadMission.h>
#include <mars_navigation/EmergencyStop.h>
#include <mars_navigation/Waypoint.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <angles/angles.h>
#include <vector>
#include <mutex>
#include <algorithm>

class MarsNavigationNode {
public:
    explicit MarsNavigationNode(ros::NodeHandle& nh);
    ~MarsNavigationNode();

private:
    // ROS相关
    ros::NodeHandle& nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Action服务器
    actionlib::SimpleActionServer<mars_navigation::ExecuteMissionAction> execute_mission_server_;
    actionlib::SimpleActionServer<mars_navigation::NavigateToTargetAction> navigate_to_target_server_;

    // 服务服务器
    ros::ServiceServer upload_mission_server_;
    ros::ServiceServer emergency_stop_server_;
    ros::ServiceServer stop_navigation_server_;
    ros::ServiceServer pause_navigation_server_;
    ros::ServiceServer resume_navigation_server_;

    // 订阅者和发布者
    ros::Subscriber odom_subscriber_;
    ros::Subscriber battery_state_subscriber_;
    ros::Subscriber robot_mode_subscriber_;
    ros::Publisher cmd_vel_publisher_;

    // 导航状态枚举
    enum class NavigationState {
        IDLE,
        RUNNING,
        PAUSED
    };

    enum class NavigationStage {
        HEADING_ADJUST,
        PATH_FOLLOWING,
    };

    struct WaypointOdom {
        geometry_msgs::Point position;  // 在odom坐标系中的位置
        double heading;                 // 目标朝向
        double velocity;               // 目标速度
        double xy_goal_tolerance;      // 平面位置容差
        double yaw_goal_tolerance;     // 航向角容差
    };

    NavigationStage navigation_stage_;
    

    // 配置参数
    double max_acceleration_;      // 最大加速度 (m/s^2)
    double max_linear_speed_;     // 最大线速度 (m/s)

    double control_frequency_;    // 控制频率 (Hz)
    double look_ahead_distance_;  // 前视距离 (m)
    double k2_;                   // 横向误差增益
    double wheel_base_;          // 轮距 (m)
    double kp_angular_;          // 角速度P控制器增益
    double max_angular_velocity_; // 最大角速度 (rad/s)
    double battery_threshold_;     // 电池电量阈值
    double odom_timeout_;         // 里程计超时时间
    double heading_gain_;         // 航向误差增益
    std::string odom_frame_;      // 里程计坐标系
    std::string base_frame_;      // 基座坐标系
    std::string utm_frame_;       // UTM坐标系
    std::string utm_zone_;        // UTM区域

    // 状态变量
    std::vector<mars_navigation::Waypoint> mission_waypoints_;
    std::vector<WaypointOdom> mission_waypoints_odom_;
    nav_msgs::Odometry current_odom_;
    geometry_msgs::Point path_start_odom_;
    geometry_msgs::Point path_end_odom_;
    ros::Time last_odom_time_;
    double battery_level_;
    double current_velocity_;
    bool is_ready_;
    bool is_autonomous_mode_;
    bool emergency_stop_;
    bool is_mission_active_;
    bool has_valid_odom_;
    NavigationState nav_state_;
    size_t current_waypoint_index_;

    // 互斥锁
    mutable std::mutex mission_mutex_;
    mutable std::mutex odom_mutex_;
    mutable std::mutex battery_mutex_;
    mutable std::mutex mode_mutex_;
    mutable std::mutex nav_state_mutex_;

    // 保存当前跟踪的路径段的起点和终点
    mars_navigation::Waypoint path_start_;
    mars_navigation::Waypoint path_end_;



    bool localToLatLon(double x, double y, double z,
                      double& latitude, double& longitude, double& altitude);
    bool latLonToLocal(double latitude, double longitude, double altitude,
                      double& x, double& y, double& z);
    bool latLonToUtm(double latitude, double longitude,
                    double& easting, double& northing);
    bool utmToLatLon(double easting, double northing,
                    double& latitude, double& longitude);
    bool transformPoseToUtm(const geometry_msgs::Pose& pose,
                          double& easting, double& northing);
    bool transformUtmToOdom(double easting, double northing,
                          geometry_msgs::Pose& pose);

    // 服务回调函数
    bool handleUploadMission(mars_navigation::UploadMission::Request& req,
                           mars_navigation::UploadMission::Response& res);
    bool handleEmergencyStop(mars_navigation::EmergencyStop::Request& req,
                           mars_navigation::EmergencyStop::Response& res);
    bool handlePauseNavigation(std_srvs::Trigger::Request& req,
                             std_srvs::Trigger::Response& res);
    bool handleResumeNavigation(std_srvs::Trigger::Request& req,
                              std_srvs::Trigger::Response& res);
    bool handleStopNavigation(std_srvs::Trigger::Request& req,
                              std_srvs::Trigger::Response& res);
    // Action回调函数
    void executeMissionCallback(const mars_navigation::ExecuteMissionGoalConstPtr& goal);
    void navigateToTargetCallback(const mars_navigation::NavigateToTargetGoalConstPtr& goal);

    // 话题回调函数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
    void robotModeCallback(const mars_canopen::RobotMode::ConstPtr& msg);

    // 导航辅助函数
    bool isOdomValid() const;
    bool isAutonomousMode() const;
    bool isBatteryLow() const;
    bool isEmergencyStop() const;
    bool isWaypointValid(const mars_navigation::Waypoint& wp) const;
    bool isNavigationPaused() const;
    void setNavigationState(NavigationState state);
    NavigationState getNavigationState() const;
    void publishVelocityCommand(double linear_x, double angular_z);
    
    double getCurrentVelocity() const;
    double calculateTargetVelocity(double distance, double max_velocity) const;
    void calculateVelocityCommand(
        const geometry_msgs::Point& path_start,
        const geometry_msgs::Point& path_end,
        const geometry_msgs::Point& current_location,
        double target_velocity,
        geometry_msgs::Twist& cmd_vel);
    double calculateDistance2D(const geometry_msgs::Point& wp1,
                           const geometry_msgs::Point& wp2);
    double calculateHeadingError(
        const geometry_msgs::Point& path_start,
        const geometry_msgs::Point& path_end,
        const geometry_msgs::Quaternion& current_orientation) const;
    
    void stopRobot();

    // 辅助函数
    inline double clamp(double value, double min_value, double max_value) const {
        return std::min(std::max(value, min_value), max_value);
    }

};

#endif // MARS_NAVIGATION_NODE_H 