#include "mars_navigation/mars_navigation_node.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>
#include "mars_navigation/conversions.h"
#include "mars_navigation/Waypoint.h"
#include <boost/bind.hpp>
#include <tf2/utils.h>

MarsNavigationNode::MarsNavigationNode(ros::NodeHandle& nh) 
    : nh_(nh),
      execute_mission_server_(nh, "execute_mission", 
          boost::bind(&MarsNavigationNode::executeMissionCallback, this, _1), false),
      navigate_to_target_server_(nh, "navigate_to_target", 
          boost::bind(&MarsNavigationNode::navigateToTargetCallback, this, _1), false),
      tf_listener_(tf_buffer_),
      nav_state_(NavigationState::IDLE),
      navigation_stage_(NavigationStage::HEADING_ADJUST),
      is_ready_(false) {
    ROS_INFO("Initializing Mars Navigation Node...");
    
    // 初始化成员变量
    current_waypoint_index_ = 0;
    is_mission_active_ = false;
    emergency_stop_ = false;
    current_velocity_ = 0.0;
    battery_level_ = 100.0;
    is_autonomous_mode_ = false;  // 初始化为非自主模式
    
    // 初始化里程计相关变量
    has_valid_odom_ = false;
    last_odom_time_ = ros::Time(0);
    odom_timeout_ = 1.0;  // 1秒超时
    odom_frame_ = "odom";
    base_frame_ = "base_link";
    utm_frame_ = "utm";
    
    // 从参数服务器加载配置
    nh_.param("odom_timeout", odom_timeout_, 1.0);
    nh_.param("odom_frame", odom_frame_, std::string("odom"));
    nh_.param("base_frame", base_frame_, std::string("base_link"));
    nh_.param("utm_frame", utm_frame_, std::string("utm"));
    
    // 从参数服务器加载参数
    if (!nh_.param<double>("max_acceleration", max_acceleration_, 0.5)) {
        ROS_INFO("Using default max_acceleration: %.2f m/s^2", max_acceleration_);
    }
    
    if (!nh_.param<double>("control_frequency", control_frequency_, 10.0)) {
        ROS_INFO("Using default control_frequency: %.1f Hz", control_frequency_);
    }
    
    if (!nh_.param<double>("maximum_linear_speed", max_linear_speed_, 1.0)) {
        ROS_INFO("Using default maximum_linear_speed: %.2f m/s", max_linear_speed_);
    }
    
    if (!nh_.param<double>("look_ahead_distance", look_ahead_distance_, 1.0)) {
        ROS_INFO("Using default look_ahead_distance: %.2f m", look_ahead_distance_);
    }
    
    if (!nh_.param<double>("k2", k2_, 0.5)) {
        ROS_INFO("Using default k2: 0.5");
    }
    
    if (!nh_.param<double>("kp_angular", kp_angular_, 1.0)) {
        ROS_INFO("Using default kp_angular: 1.0");
    }
    
    if (!nh_.param<double>("max_angular_velocity", max_angular_velocity_, 1.0)) {
        ROS_INFO("Using default max_angular_velocity: 1.0 rad/s");
    }

    if (!nh_.param<double>("heading_gain", heading_gain_, 1.0)) {
        ROS_INFO("Using default heading_gain: 1.0");
    }

    
    // 订阅话题
    odom_subscriber_ = nh_.subscribe<nav_msgs::Odometry>("odom", 10, &MarsNavigationNode::odomCallback, this);
    battery_state_subscriber_ = nh_.subscribe<sensor_msgs::BatteryState>("battery_state", 10, &MarsNavigationNode::batteryStateCallback, this);
    robot_mode_subscriber_ = nh_.subscribe<mars_canopen::RobotMode>("robot_mode", 10, &MarsNavigationNode::robotModeCallback, this);
    
    // 发布器
    cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    
    // 等待有效的里程计数据
    ROS_INFO("Waiting for valid odometry data...");
    ros::Rate wait_rate(100);  // 1Hz检查频率
    while (ros::ok() && !isOdomValid()) {
        ros::spinOnce();
        wait_rate.sleep();
    }
    ROS_INFO("Received valid odometry data");
    
    // 初始化服务器
    upload_mission_server_ = nh_.advertiseService("upload_mission", &MarsNavigationNode::handleUploadMission, this);
    emergency_stop_server_ = nh_.advertiseService("emergency_stop", &MarsNavigationNode::handleEmergencyStop, this);
    stop_navigation_server_ = nh_.advertiseService("stop_navigation", &MarsNavigationNode::handleStopNavigation, this);
    pause_navigation_server_ = nh_.advertiseService("pause_navigation", &MarsNavigationNode::handlePauseNavigation, this);
    resume_navigation_server_ = nh_.advertiseService("resume_navigation", &MarsNavigationNode::handleResumeNavigation, this);
    
    // 启动action服务器
    execute_mission_server_.start();
    navigate_to_target_server_.start();
    
    is_ready_ = true;
    ROS_INFO("Mars navigation node is ready");
}

MarsNavigationNode::~MarsNavigationNode() {
    // 清理资源（如果需要）
    ROS_INFO("Mars navigation node is shutting down");
        // 停止所有运动
    publishVelocityCommand(0.0, 0.0);
}

bool MarsNavigationNode::isOdomValid() const {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    // 检查是否收到过里程计数据
    if (last_odom_time_.isZero()) {
        return false;
    }
    
    // 检查里程计数据是否超时
    ros::Time now = ros::Time::now();
    double dt = (now - last_odom_time_).toSec();
    return dt <= odom_timeout_;
}

void MarsNavigationNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_odom_ = *msg;
    current_velocity_ = current_odom_.twist.twist.linear.x;
    last_odom_time_ = ros::Time::now();
        
    ROS_DEBUG("Odometry update - Position: (%.2f, %.2f, %.2f), Velocity: %.2f m/s",
              current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z, current_velocity_);
}

double MarsNavigationNode::getCurrentVelocity() const {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    // 使用里程计中的实际速度
    return std::abs(current_odom_.twist.twist.linear.x);
}

bool MarsNavigationNode::localToLatLon(double x, double y, double z, double& lat, double& lon, double& alt) {
    try {
        // 1. 获取从odom到utm的转换
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            utm_frame_, odom_frame_, ros::Time(0));
        
        // 2. 创建odom坐标系中的点
        geometry_msgs::Pose odom_pose;
        odom_pose.position.x = x;
        odom_pose.position.y = y;
        odom_pose.position.z = z;
        odom_pose.orientation.w = 1.0;
        
        // 3. 将odom坐标转换到UTM坐标系
        geometry_msgs::Pose utm_pose;
        tf2::doTransform(odom_pose, utm_pose, transform);
        
        // 4. 将UTM坐标转换为经纬度
        if (!utmToLatLon(utm_pose.position.x, utm_pose.position.y, lat, lon)) {
            ROS_ERROR("Failed to convert UTM coordinates to lat/lon");
            return false;
        }
        
        // 5. 高度直接使用z值
        alt = z;
        return true;
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("Failed to transform odom coordinates to UTM: %s", ex.what());
        return false;
    }
}

bool MarsNavigationNode::latLonToLocal(double lat, double lon, double alt, double& x, double& y, double& z) {
    try {
        // 1. 将经纬度转换为UTM坐标
        double utm_x, utm_y;
        if (!latLonToUtm(lat, lon, utm_x, utm_y)) {
            ROS_ERROR("Failed to convert lat/lon to UTM coordinates");
            return false;
        }
        
        // 2. 获取从utm到odom的转换
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            odom_frame_, utm_frame_, ros::Time(0));
        
        // 3. 创建UTM坐标系中的点
        geometry_msgs::Pose utm_pose;
        utm_pose.position.x = utm_x;
        utm_pose.position.y = utm_y;
        utm_pose.position.z = alt;
        utm_pose.orientation.w = 1.0;
        
        // 4. 将UTM坐标转换到odom坐标系
        geometry_msgs::Pose odom_pose;
        tf2::doTransform(utm_pose, odom_pose, transform);
        
        // 5. 直接使用odom坐标系中的坐标
        x = odom_pose.position.x;
        y = odom_pose.position.y;
        z = alt;
        return true;
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("Failed to transform UTM coordinates to odom: %s", ex.what());
        return false;
    }
}

bool MarsNavigationNode::latLonToUtm(double lat, double lon, double& utm_x, double& utm_y) {
    // 使用conversions.h中的LLtoUTM函数
    gps_tools::LLtoUTM(lat, lon, utm_y, utm_x, utm_zone_);
    
    return true;
}

bool MarsNavigationNode::utmToLatLon(double utm_x, double utm_y, double& lat, double& lon) {
    if (utm_zone_.empty()) {
        ROS_ERROR("UTM zone is not set");
        return false;
    }
    // 使用conversions.h中的UTMtoLL函数
    gps_tools::UTMtoLL(utm_y, utm_x, utm_zone_.c_str(), lat, lon);
    return true;
}

bool MarsNavigationNode::transformPoseToUtm(const geometry_msgs::Pose& pose, double& utm_x, double& utm_y) {
    try {
        // 获取从odom到utm的转换
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            utm_frame_, odom_frame_, ros::Time(0));
        
        // 转换位置
        geometry_msgs::Pose utm_pose;
        tf2::doTransform(pose, utm_pose, transform);
        
        utm_x = utm_pose.position.x;
        utm_y = utm_pose.position.y;
        
        return true;
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("Failed to transform pose to UTM: %s", ex.what());
        return false;
    }
}

bool MarsNavigationNode::transformUtmToOdom(double utm_x, double utm_y, geometry_msgs::Pose& pose) {
    try {
        // 获取从utm到odom的转换
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            odom_frame_, utm_frame_, ros::Time(0));
        
        // 创建UTM坐标系中的点
        geometry_msgs::Pose utm_pose;
        utm_pose.position.x = utm_x;
        utm_pose.position.y = utm_y;
        utm_pose.position.z = 0.0;
        utm_pose.orientation.w = 1.0;
        
        // 转换到odom坐标系
        tf2::doTransform(utm_pose, pose, transform);
        
        return true;
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("Failed to transform UTM to odom: %s", ex.what());
        return false;
    }
}

bool MarsNavigationNode::handleUploadMission(
    mars_navigation::UploadMission::Request& req,
    mars_navigation::UploadMission::Response& res) {
    
    ROS_INFO("Received mission upload request with %zu waypoints", req.waypoints.size());
    
    if (req.waypoints.empty()) {
        ROS_WARN("Received empty waypoint list");
        res.success = false;
        res.message = "Empty waypoint list";
        return true;
    }
    
    
    for (size_t i = 0; i < req.waypoints.size(); ++i) {
        const auto& wp = req.waypoints[i];
        
        // 检查航点参数
        if (wp.velocity <= 0 || wp.velocity > max_linear_speed_) {
            ROS_ERROR("Invalid velocity (%.2f) for waypoint %zu", wp.velocity, i);
            res.success = false;
            res.message = "Invalid velocity parameter";
            return true;
        }
        
        if (wp.xy_goal_tolerance <= 0) {
            ROS_ERROR("Invalid xy_goal_tolerance (%.2f) for waypoint %zu", 
                     wp.xy_goal_tolerance, i);
            res.success = false;
            res.message = "Invalid xy_goal_tolerance parameter";
            return true;
        }
        
        if (wp.yaw_goal_tolerance <= 0 || wp.yaw_goal_tolerance > M_PI) {
            ROS_ERROR("Invalid yaw_goal_tolerance (%.2f) for waypoint %zu", 
                     wp.yaw_goal_tolerance, i);
            res.success = false;
            res.message = "Invalid yaw_goal_tolerance parameter";
            return true;
        }
    }
    // 将互斥锁限制在较小的作用域内
    {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        ROS_INFO("Mission mutex acquired");
        
        try {
            // 只在实际需要保护的代码块中使用互斥锁
            mission_waypoints_ = req.waypoints;
            current_waypoint_index_ = 0;
            
            ROS_INFO("Mission waypoints updated successfully");
        }
        catch (const std::exception& e) {
            ROS_ERROR("Error updating mission waypoints: %s", e.what());
            res.success = false;
            res.message = "Internal error while updating waypoints";
            return true;
        }
    } // 互斥锁在这里自动释放
    
    ROS_INFO("Mission uploaded successfully with %zu waypoints", mission_waypoints_.size());
    res.success = true;
    res.message = "Mission uploaded successfully";
    return true;
}

void MarsNavigationNode::navigateToTargetCallback(
    const mars_navigation::NavigateToTargetGoalConstPtr& goal) {
    std::lock_guard<std::mutex> lock(mission_mutex_);
    
    // 设置初始导航状态
    {
        std::lock_guard<std::mutex> state_lock(nav_state_mutex_);
        nav_state_ = NavigationState::RUNNING;
    }
    
    ros::Rate rate(control_frequency_);  // 使用参数配置的控制频率
    
    // 将目标点转换到odom坐标系
    double odom_x, odom_y, odom_z;
    if (!latLonToLocal(goal->target.latitude, goal->target.longitude, 
                      goal->target.altitude, odom_x, odom_y, odom_z)) {
        mars_navigation::NavigateToTargetResult result;
        result.success = false;
        result.message = "Failed to convert target to odom coordinates";
        navigate_to_target_server_.setAborted(result);
        return;
    }
    
    // 设置路径段（从当前位置到目标点）
    path_start_odom_ = current_odom_.pose.pose.position;
    path_end_odom_.x = odom_x;
    path_end_odom_.y = odom_y;
    path_end_odom_.z = odom_z;
    auto previous_time = ros::Time::now();
    
    while (ros::ok()) {
        // 检查是否有抢占请求
        if (navigate_to_target_server_.isPreemptRequested()) {
            // 发送零速度命令
            publishVelocityCommand(0.0, 0.0);
            
            // 设置状态为IDLE
            {
                std::lock_guard<std::mutex> state_lock(nav_state_mutex_);
                nav_state_ = NavigationState::IDLE;
            }
            
            navigate_to_target_server_.setPreempted();
            return;
        }
        
        // 检查里程计是否有效
        if (!isOdomValid()) {
            publishVelocityCommand(0.0, 0.0);
            
            mars_navigation::NavigateToTargetResult result;
            result.success = false;
            result.message = "Invalid odometry data";
            result.final_distance = 0.0;
            result.final_heading_error = 0.0;
            navigate_to_target_server_.setAborted(result);
            return;
        }
        
        // 检查是否处于自主模式
        if (!isAutonomousMode()) {
            publishVelocityCommand(0.0, 0.0);
            
            mars_navigation::NavigateToTargetResult result;
            result.success = false;
            result.message = "Robot is not in autonomous mode";
            result.final_distance = 0.0;
            result.final_heading_error = 0.0;
            navigate_to_target_server_.setAborted(result);
            return;
        }
        
        // 检查导航状态
        {
            std::lock_guard<std::mutex> state_lock(nav_state_mutex_);
            if (nav_state_ == NavigationState::IDLE) {
                // 导航被停止
                publishVelocityCommand(0.0, 0.0);
                
                mars_navigation::NavigateToTargetResult result;
                result.success = false;
                result.message = "Navigation stopped";
                result.final_distance = 0.0;
                result.final_heading_error = 0.0;
                navigate_to_target_server_.setAborted(result);
                return;
            } else if (nav_state_ == NavigationState::PAUSED) {
                // 导航暂停时发布零速度并等待
                publishVelocityCommand(0.0, 0.0);
                rate.sleep();
                continue;
            }
        }
        
        // 计算当前位置到目标的距离和朝向误差
        double distance = calculateDistance2D(current_odom_.pose.pose.position, path_end_odom_);
        double heading_error = calculateHeadingError(path_start_odom_, path_end_odom_, current_odom_.pose.pose.orientation);
        
        // 检查是否到达目标
        if (distance <= goal->target.xy_goal_tolerance && 
            std::abs(heading_error) <= goal->target.yaw_goal_tolerance) {
            publishVelocityCommand(0.0, 0.0);
            
            mars_navigation::NavigateToTargetResult result;
            result.success = true;
            result.message = "Target reached";
            result.final_distance = distance;
            result.final_heading_error = heading_error;
            navigate_to_target_server_.setSucceeded(result);
            return;
        }
        
        // 获取当前实际速度
        double current_velocity = getCurrentVelocity();
        
        geometry_msgs::Twist cmd_vel;
        // 计算目标速度命令
        calculateVelocityCommand(path_start_odom_, path_end_odom_, current_odom_.pose.pose.position, goal->target.velocity, cmd_vel);
        
        // 限制加速度（基于实际速度和目标速度）
        double target_velocity = std::abs(cmd_vel.linear.x);
        double velocity_change = target_velocity - current_velocity;
        double max_velocity_change = max_acceleration_ * (ros::Time::now() - previous_time).toSec();
        
        if (std::abs(velocity_change) > max_velocity_change) {
            // 如果速度变化超过限制，则按最大加速度调整
            double scale = max_velocity_change / std::abs(velocity_change);
            cmd_vel.linear.x *= scale;
        }
        
        cmd_vel_publisher_.publish(cmd_vel);
        

        // 发布反馈（使用实际速度）
        mars_navigation::NavigateToTargetFeedback feedback;
        feedback.current_distance = distance;
        feedback.current_heading_error = heading_error;
        feedback.current_velocity = current_velocity;  // 使用实际速度
        navigate_to_target_server_.publishFeedback(feedback);
        
        rate.sleep();
    }
}

bool MarsNavigationNode::handleEmergencyStop(mars_navigation::EmergencyStop::Request& req,
                                           mars_navigation::EmergencyStop::Response& res) {
    std::lock_guard<std::mutex> lock(mission_mutex_);
    
    emergency_stop_ = req.emergency_stop;
    
    if (emergency_stop_) {
        is_mission_active_ = false;
        publishVelocityCommand(0.0, 0.0);
    }
    
    res.success = true;
    res.message = emergency_stop_ ? "Emergency stop activated" : "Emergency stop deactivated";
    res.stop_distance = calculateDistance2D(path_start_odom_, path_end_odom_);
    return true;
}

void MarsNavigationNode::executeMissionCallback(
    const mars_navigation::ExecuteMissionGoalConstPtr& goal) {
    // 转换所有航点到odom坐标系
    std::vector<WaypointOdom> waypoints_odom;
    ROS_INFO("Starting mission execution");
    {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        
        if (mission_waypoints_.empty()) {
            mars_navigation::ExecuteMissionResult result;
            result.success = false;
            result.message = "No waypoints available";
            execute_mission_server_.setAborted(result);
            return;
        }
    
    
        
        for (size_t i = 0; i < mission_waypoints_.size(); ++i) {
            const auto& wp = mission_waypoints_[i];
            double odom_x, odom_y, odom_z;
            
            if (!latLonToLocal(wp.latitude, wp.longitude, wp.altitude, 
                            odom_x, odom_y, odom_z)) {
                ROS_ERROR("Failed to convert waypoint %zu to odom coordinates", i);
                mars_navigation::ExecuteMissionResult result;
                result.success = false;
                result.message = "Coordinate conversion failed";
                execute_mission_server_.setAborted(result);
                return;
            }
            
            WaypointOdom wp_odom;
            wp_odom.position.x = odom_x;
            wp_odom.position.y = odom_y;
            wp_odom.position.z = odom_z;
            wp_odom.heading = wp.heading;
            wp_odom.velocity = wp.velocity;
            wp_odom.xy_goal_tolerance = wp.xy_goal_tolerance;
            wp_odom.yaw_goal_tolerance = wp.yaw_goal_tolerance;
            
            ROS_DEBUG("Converted waypoint %zu to odom: (%.2f, %.2f, %.2f), heading: %.2f rad",
                    i, odom_x, odom_y, odom_z, wp.heading);
            
            waypoints_odom.push_back(wp_odom);
        }
        
    }

    // 设置状态为RUNNING
    {
        std::lock_guard<std::mutex> state_lock(nav_state_mutex_);
        nav_state_ = NavigationState::RUNNING;
    }
    current_waypoint_index_ = 0;
    ros::Rate rate(control_frequency_);

    auto previous_time = ros::Time::now();
    ROS_INFO("Starting mission with %zu waypoints", mission_waypoints_.size());
    while (ros::ok()) {

        if (execute_mission_server_.isPreemptRequested()) {
            // 发送零速度命令
            stopRobot();
            
            // 设置状态为IDLE
            {
                std::lock_guard<std::mutex> state_lock(nav_state_mutex_);
                nav_state_ = NavigationState::IDLE;
            }
            
            execute_mission_server_.setPreempted();
            return;
        }

        // check if the robot is in autonomous mode
        if (!isAutonomousMode()) {
            stopRobot();
            ROS_ERROR("Cannot execute mission: Robot is not in autonomous mode");
            mars_navigation::ExecuteMissionResult result;
            result.success = false;
            result.message = "Robot is not in autonomous mode";
            execute_mission_server_.setAborted(result);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mission_mutex_);
            if (emergency_stop_) {
                ROS_INFO("Mission stopped");
                stopRobot();
                ROS_ERROR("Mission aborted due to emergency stop");
                mars_navigation::ExecuteMissionResult result;
                result.success = false;
                result.message = "Mission aborted due to emergency stop";
                execute_mission_server_.setAborted(result);
                return;
            }
        }

        auto nav_state = NavigationState::IDLE;
        {
            std::lock_guard<std::mutex> lock(nav_state_mutex_);
            nav_state = nav_state_;
        }


        if (nav_state == NavigationState::PAUSED) {
            ROS_INFO("Mission paused");
            stopRobot();
            rate.sleep();
            continue;
        }
        
        // check if the odometry data is valid
        if (!isOdomValid()) {
            ROS_WARN("Waiting for valid odometry data...");
            // pause the robot 
            publishVelocityCommand(0.0, 0.0);
            rate.sleep();
            continue;
        }

        // 更新当前路径段
        if (current_waypoint_index_ == 0) {
            path_start_odom_ = current_odom_.pose.pose.position;
        } else {
            path_start_odom_ = waypoints_odom[current_waypoint_index_ - 1].position;
        }
        path_end_odom_ = waypoints_odom[current_waypoint_index_].position;
        
        // 计算到当前目标点的距离和航向误差
        double distance = calculateDistance2D(
            current_odom_.pose.pose.position,
            path_end_odom_);
        
        double heading_error = calculateHeadingError(
            path_start_odom_,
            path_end_odom_,
            current_odom_.pose.pose.orientation);
        
        ROS_DEBUG("Navigation status - Waypoint: %zu, Distance: %.2f m, Heading error: %.2f rad",
                 current_waypoint_index_, distance, heading_error);
        
        geometry_msgs::Twist cmd_vel;
        // 获取当前航点
        const auto& current_waypoint = waypoints_odom[current_waypoint_index_];

        if (distance <= current_waypoint.xy_goal_tolerance) {
            // 已到达目标点位置，停止移动
            
            cmd_vel.linear.x = 0.0;
            double heading_error = angles::shortest_angular_distance(tf2::getYaw(current_odom_.pose.pose.orientation), waypoints_odom[current_waypoint_index_].heading);
            // 如果朝向误差大于容差，进行原地旋转
            if (std::abs(heading_error) > current_waypoint.yaw_goal_tolerance) {
                ROS_DEBUG("Adjusting heading at waypoint %d, error: %.2f", 
                    current_waypoint_index_, heading_error);
                
                // 设置角速度（使用比例控制）
                double angular_velocity = clamp(
                    heading_error * heading_gain_,  // 比例控制
                    -max_angular_velocity_,         // 最小角速度
                    max_angular_velocity_           // 最大角速度
                );
                cmd_vel.angular.z = angular_velocity;
            } else {
                // 位置和朝向都满足要求
                ROS_INFO("Reached waypoint %d ------------------------------", current_waypoint_index_);
                cmd_vel.angular.z = 0.0;
                
                // 更新到下一个航点
                current_waypoint_index_++;

                if (current_waypoint_index_ >= waypoints_odom.size()) {
                    ROS_INFO("Mission completed successfully");
                    stopRobot();
                    mars_navigation::ExecuteMissionResult result;
                    result.success = true;
                    result.message = "Mission completed successfully";
                    result.final_distance = distance;
                    result.final_heading_error = heading_error;
                    execute_mission_server_.setSucceeded(result);
                    return;
                }
            }
        } else {
            // 还未到达目标点，继续使用pure pursuit控制
            calculateVelocityCommand(path_start_odom_, path_end_odom_, current_odom_.pose.pose.position, current_waypoint.velocity, cmd_vel);
            
        }
        
        // 获取当前实际速度
        double current_velocity = getCurrentVelocity();
        // 限制加速度（基于实际速度和目标速度）
        double target_velocity = std::abs(cmd_vel.linear.x);
        double velocity_change = target_velocity - current_velocity;
        double max_velocity_change = max_acceleration_ * (ros::Time::now() - previous_time).toSec();
        
        if (std::abs(velocity_change) > max_velocity_change) {  
            double scale = max_velocity_change / std::abs(velocity_change);
            cmd_vel.linear.x *= scale;
        }

        ROS_DEBUG("Command velocity - Linear: %.2f m/s, Angular: %.2f rad/s",
                 cmd_vel.linear.x, cmd_vel.angular.z);
        
        cmd_vel_publisher_.publish(cmd_vel);
        
        // 发布反馈
        mars_navigation::ExecuteMissionFeedback feedback;
        feedback.current_waypoint = current_waypoint_index_;
        feedback.distance_to_waypoint = distance;
        feedback.heading_error = angles::shortest_angular_distance(tf2::getYaw(current_odom_.pose.pose.orientation), waypoints_odom[current_waypoint_index_].heading);;
        feedback.current_velocity = current_velocity;
        execute_mission_server_.publishFeedback(feedback);
        
        rate.sleep();
    }
    
    stopRobot();
    mars_navigation::ExecuteMissionResult result;
    result.success = false;
    result.message = "Mission aborted";
    execute_mission_server_.setAborted(result);
}

double MarsNavigationNode::calculateDistance2D(const geometry_msgs::Point& wp1, const geometry_msgs::Point& wp2) {
    double dx = wp2.x - wp1.x;
    double dy = wp2.y - wp1.y;
    return std::sqrt(dx*dx + dy*dy);
}

double MarsNavigationNode::calculateHeadingError(
    const geometry_msgs::Point& path_start,
    const geometry_msgs::Point& path_end,
    const geometry_msgs::Quaternion& current_orientation) const {
    
    // 将geometry_msgs::Quaternion转换为tf2::Quaternion
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(current_orientation, tf2_quat);
    
    // 获取当前机器人朝向
    double current_yaw = tf2::getYaw(tf2_quat);
    
    // 计算路径方向向量
    double path_x = path_end.x - path_start.x;
    double path_y = path_end.y - path_start.y;
    
    // 计算路径的航向角
    double path_yaw = std::atan2(path_y, path_x);
    
    // 计算航向误差（使用angles::shortest_angular_distance确保得到最小角度差）
    double heading_error = angles::shortest_angular_distance(current_yaw, path_yaw);
    
    return heading_error;
}

void MarsNavigationNode::publishVelocityCommand(double linear_x, double angular_z) {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_x;
    cmd_vel.angular.z = angular_z;
    cmd_vel_publisher_.publish(cmd_vel);
}


void MarsNavigationNode::batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(battery_mutex_);
    
    // 更新电池电量（百分比）
    if (msg->percentage > 0) {
        battery_level_ = msg->percentage * 100.0;
    }
    
    // 检查电池状态
    if (msg->power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING) {
        // 如果电池电量低于20%，停止任务
        if (battery_level_ < 10.0) {
            std::lock_guard<std::mutex> mission_lock(mission_mutex_);
            is_mission_active_ = false;
            publishVelocityCommand(0.0, 0.0);
            ROS_WARN("Battery level low (%.1f%%), stopping mission", battery_level_);
        }
    }
    
    // 检查是否处于紧急状态
    if (msg->power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING ||
        msg->power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL) {
        // 电池状态正常
        is_ready_ = true;
    } else {
        // 电池状态异常
        is_ready_ = false;
        ROS_WARN("Battery status abnormal: %d", msg->power_supply_status);
    }
}

void MarsNavigationNode::robotModeCallback(const mars_canopen::RobotMode::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    is_autonomous_mode_ = (msg->mode == 1);
    if (!is_autonomous_mode_) {
        ROS_WARN_THROTTLE(1.0, "Robot is not in autonomous mode (current mode: %d)", msg->mode);
    }
}

bool MarsNavigationNode::isAutonomousMode() const {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    return is_autonomous_mode_;
}


bool MarsNavigationNode::handlePauseNavigation(std_srvs::Trigger::Request& req, 
                                             std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lock(nav_state_mutex_);
    ROS_INFO("Pause navigation");
    if (nav_state_ == NavigationState::RUNNING) {
        nav_state_ = NavigationState::PAUSED;
        // 发送零速度命令暂停机器人
        stopRobot();
        
        res.success = true;
        res.message = "Navigation paused";
    } else {
        res.success = false;
        res.message = "Navigation is not running";
    }
    return true;
}

bool MarsNavigationNode::handleResumeNavigation(std_srvs::Trigger::Request& req, 
                                              std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lock(nav_state_mutex_);
    ROS_INFO("Resume navigation");
    if (nav_state_ == NavigationState::PAUSED) {
        nav_state_ = NavigationState::RUNNING;
        res.success = true;
        res.message = "Navigation resumed";
    } else {
        res.success = false;
        res.message = "Navigation is not paused";
    }
    return true;
}

bool MarsNavigationNode::handleStopNavigation(std_srvs::Trigger::Request& req, 
                                              std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lock(nav_state_mutex_);
    if (nav_state_ == NavigationState::RUNNING) {
        ROS_INFO("Stop navigation");
        stopRobot();
        nav_state_ = NavigationState::IDLE;
        res.success = true;
        res.message = "Navigation stopped";
    } else {
        res.success = false;
        res.message = "Navigation is not running";
    }
    return true;
}
double MarsNavigationNode::calculateTargetVelocity(double distance, double max_velocity) const {
    // 使用减速公式：v = sqrt(2 * a * d)
    double decel_velocity = std::sqrt(2.0 * max_acceleration_ * distance);
    return std::min(decel_velocity, max_velocity);
}

void MarsNavigationNode::calculateVelocityCommand(
    const geometry_msgs::Point& path_start,
    const geometry_msgs::Point& path_end,
    const geometry_msgs::Point& current_location,
    double target_velocity,
    geometry_msgs::Twist& cmd_vel) {
    
    // 计算路径向量
    double path_x = path_end.x - path_start.x;
    double path_y = path_end.y - path_start.y;
    double path_length = std::sqrt(path_x * path_x + path_y * path_y);
    
    ROS_DEBUG("Path length: %.2f m", path_length);
    
    // 将路径向量单位化
    if (path_length > 0) {
        path_x /= path_length;
        path_y /= path_length;
    }
    else{
        ROS_ERROR("Path length is 0");
        path_x = 0.0;
        path_y = 0.0;
    }
    
    // 计算机器人到路径起点的向量
    double robot_to_start_x = current_location.x - path_start.x;
    double robot_to_start_y = current_location.y - path_start.y;

    // distance to end of path
    double distance_to_end = calculateDistance2D(current_location, path_end);
    
    // 计算机器人在路径上的投影点
    double projection = robot_to_start_x * path_x + robot_to_start_y * path_y;
    // projection = clamp(projection, 0.0, path_length);
    
    // 计算横向误差（使用点到直线的距离公式）
    double lateral_error = std::abs(robot_to_start_x * path_y - robot_to_start_y * path_x);
    
    ROS_DEBUG("Projection: %.2f m, Lateral error: %.2f m", projection, lateral_error);
    
    // 计算考虑减速的目标速度，并限制在最大速度范围内
    double limited_target_velocity = std::min(target_velocity, max_linear_speed_);
    double approach_velocity = calculateTargetVelocity(distance_to_end, limited_target_velocity);
    // ROS_INFO("approach_velocity: %.2f", approach_velocity);
    
    // 计算动态前视距离
    // lookahead = 1/(2*max_acceleration)*speed^2 + k2*lateral_error + initial_lookahead
    double dynamic_look_ahead = (1.0 / (2.0 * max_acceleration_)) * 
                               approach_velocity * approach_velocity +
                               k2_ * lateral_error +
                               look_ahead_distance_;
    
    // 限制前视距离在合理范围内
    dynamic_look_ahead = clamp(dynamic_look_ahead, 
                              look_ahead_distance_ * 0.5,  // 最小前视距离
                              look_ahead_distance_ * 2.0); // 最大前视距离
        
    // 计算前视点位置
    double look_ahead_projection = projection + dynamic_look_ahead;
    look_ahead_projection = clamp(look_ahead_projection, 0.0, path_length);

    
    // 计算前视点的实际位置
    geometry_msgs::Point look_ahead_point;
    look_ahead_point.x = path_start.x + path_x * look_ahead_projection;
    look_ahead_point.y = path_start.y + path_y * look_ahead_projection;
    look_ahead_point.z = path_start.z;
    
    // 计算机器人到前视点的向量
    double distance_to_goal = calculateDistance2D(current_location, look_ahead_point);
    
    double robot_heading = tf2::getYaw(current_odom_.pose.pose.orientation);
    // 计算航向误差
    double heading_error = std::atan2(look_ahead_point.y - current_location.y, look_ahead_point.x - current_location.x) - robot_heading;
    
    ROS_DEBUG("Distance to pursuit point: %.2f m, Heading error: %.2f rad", 
              distance_to_goal, heading_error);
    // 计算曲率
    double curvature = 2.0 * std::sin(heading_error) / distance_to_goal;
    
    // 计算角速度
    double angular_velocity = approach_velocity * curvature;
    
    // 限制角速度
    angular_velocity = clamp(angular_velocity, -max_angular_velocity_, max_angular_velocity_);
    
    // 设置速度命令
    cmd_vel.linear.x = approach_velocity;
    cmd_vel.linear.y = 0.0;  // 差速转向机器人不能横向移动
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = angular_velocity;
    
    ROS_DEBUG("Command velocity - Linear: %.2f m/s, Angular: %.2f rad/s",
              cmd_vel.linear.x, cmd_vel.angular.z);
}

bool MarsNavigationNode::isWaypointValid(const mars_navigation::Waypoint& wp) const {
    // 检查速度是否超过最大限制
    if (wp.velocity > max_linear_speed_) {
        ROS_ERROR("Waypoint velocity (%.2f m/s) exceeds maximum allowed speed (%.2f m/s)",
                  wp.velocity, max_linear_speed_);
        return false;
    }
    
    return true;
}

void MarsNavigationNode::stopRobot() {
    publishVelocityCommand(0.0, 0.0);
}

