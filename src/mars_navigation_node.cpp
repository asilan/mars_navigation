#include "mars_navigation/mars_navigation_node.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>
#include "conversions.h"

MarsNavigationNode::MarsNavigationNode(ros::NodeHandle& nh) 
    : nh_(nh),
      execute_mission_server_(nh, "execute_mission", false),
      navigate_to_target_server_(nh, "navigate_to_target", false),
      tf_listener_(tf_buffer_),
      nav_state_(NavigationState::IDLE),
      is_ready_(false) {
    // 初始化成员变量
    current_waypoint_index_ = 0;
    is_mission_active_ = false;
    is_emergency_stop_ = false;
    current_velocity_ = 0.0;
    battery_level_ = 100.0;
    is_autonomous_mode_ = false;  // 初始化为非自主模式
    
    // 初始化里程计相关变量
    has_valid_odom_ = false;
    last_odom_time_ = ros::Time::now();
    odom_timeout_ = 1.0;  // 1秒超时
    odom_frame_id_ = "odom";
    base_frame_id_ = "base_link";
    utm_frame_id_ = "utm";
    
    // 从参数服务器加载配置
    nh_.param("odom_timeout", odom_timeout_, 1.0);
    nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
    nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
    nh_.param("utm_frame_id", utm_frame_id_, std::string("utm"));
    
    // 从参数服务器加载参数
    if (!nh_.param<double>("max_acceleration", max_acceleration_, 0.5)) {
        ROS_WARN("Using default max_acceleration: 0.5 m/s^2");
    }
    
    if (!nh_.param<double>("control_frequency", control_frequency_, 10.0)) {
        ROS_WARN("Using default control_frequency: 10.0 Hz");
    }
    
    if (!nh_.param<double>("maximum_linear_speed", max_linear_speed_, 1.0)) {
        ROS_WARN("Using default maximum_linear_speed: 1.0 m/s");
    }
    
    if (!nh_.param<double>("look_ahead_distance", look_ahead_distance_, 1.0)) {
        ROS_WARN("Using default look_ahead_distance: 1.0 m");
    }
    
    if (!nh_.param<double>("k2", k2_, 0.5)) {
        ROS_WARN("Using default k2: 0.5");
    }
    
    if (!nh_.param<double>("wheel_base", wheel_base_, 0.5)) {
        ROS_WARN("Using default wheel_base: 0.5 m");
    }
    
    if (!nh_.param<double>("kp_angular", kp_angular_, 1.0)) {
        ROS_WARN("Using default kp_angular: 1.0");
    }
    
    if (!nh_.param<double>("max_angular_velocity", max_angular_velocity_, 1.0)) {
        ROS_WARN("Using default max_angular_velocity: 1.0 rad/s");
    }
    
    // 订阅话题
    odom_subscriber_ = nh_.subscribe<nav_msgs::Odometry>("odom", 10, &MarsNavigationNode::odomCallback, this);
    battery_state_subscriber_ = nh_.subscribe<sensor_msgs::BatteryState>("battery_state", 10, &MarsNavigationNode::batteryStateCallback, this);
    robot_mode_subscriber_ = nh_.subscribe<mars_canopen::RobotMode>("robot_mode", 10, &MarsNavigationNode::robotModeCallback, this);
    
    // 发布器
    cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    
    // 等待有效的里程计数据
    ROS_INFO("Waiting for valid odometry data...");
    ros::Rate wait_rate(1);  // 1Hz检查频率
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
    last_odom_time_ = ros::Time::now();
    updateCurrentPose();
}

double MarsNavigationNode::getCurrentVelocity() const {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    // 使用里程计中的实际速度
    return std::abs(current_odom_.twist.twist.linear.x);
}

void MarsNavigationNode::localToLatLon(double x, double y, double z, double& lat, double& lon, double& alt) {
    try {
        // 1. 获取从odom到utm的转换
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            utm_frame_id_, odom_frame_id_, ros::Time(0));
        
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
            return;
        }
        
        // 5. 高度直接使用z值
        alt = z;
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("Failed to transform odom coordinates to UTM: %s", ex.what());
    }
}

void MarsNavigationNode::latLonToLocal(double lat, double lon, double alt, double& x, double& y, double& z) {
    try {
        // 1. 将经纬度转换为UTM坐标
        double utm_x, utm_y;
        if (!latLonToUtm(lat, lon, utm_x, utm_y)) {
            ROS_ERROR("Failed to convert lat/lon to UTM coordinates");
            return;
        }
        
        // 2. 获取从utm到odom的转换
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            odom_frame_id_, utm_frame_id_, ros::Time(0));
        
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
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("Failed to transform UTM coordinates to odom: %s", ex.what());
    }
}

bool MarsNavigationNode::latLonToUtm(double lat, double lon, double& utm_x, double& utm_y) {
    // 使用conversions.h中的LLtoUTM函数
    char utm_zone[13];
    gps_tools::LLtoUTM(lat, lon, utm_y, utm_x, utm_zone);
    
    // 保存UTM区域信息
    utm_zone_ = utm_zone;
    utm_north_ = (utm_zone[1] == 'N');
    
    return true;
}

bool MarsNavigationNode::utmToLatLon(double utm_x, double utm_y, double& lat, double& lon) {
    // 使用conversions.h中的UTMtoLL函数
    gps_tools::UTMtoLL(utm_y, utm_x, utm_zone_.c_str(), lat, lon);
    return true;
}

bool MarsNavigationNode::transformPoseToUtm(const geometry_msgs::Pose& pose, double& utm_x, double& utm_y) {
    try {
        // 获取从odom到utm的转换
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            utm_frame_id_, odom_frame_id_, ros::Time(0));
        
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
            odom_frame_id_, utm_frame_id_, ros::Time(0));
        
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

bool MarsNavigationNode::handleUploadMission(mars_navigation::UploadMission::Request& req,
                                           mars_navigation::UploadMission::Response& res) {
    std::lock_guard<std::mutex> lock(mission_mutex_);
    
    // 检查里程计是否有效
    if (!isOdomValid()) {
        res.success = false;
        res.message = "No valid odometry data available";
        return true;
    }
    
    // 检查航点列表是否为空
    if (req.waypoints.empty()) {
        res.success = false;
        res.message = "Waypoint list is empty";
        return true;
    }
    
    // 检查每个航点的有效性
    for (size_t i = 0; i < req.waypoints.size(); ++i) {
        const Waypoint& wp = req.waypoints[i];
        
        // 检查经纬度是否有效
        if (wp.latitude < -90.0 || wp.latitude > 90.0) {
            res.success = false;
            res.message = "Invalid latitude in waypoint " + std::to_string(i);
            return true;
        }
        if (wp.longitude < -180.0 || wp.longitude > 180.0) {
            res.success = false;
            res.message = "Invalid longitude in waypoint " + std::to_string(i);
            return true;
        }
        
        // 检查速度是否有效（不超过2 m/s）
        if (wp.velocity <= 0.0 || wp.velocity > max_linear_speed_) {
            res.success = false;
            res.message = "Invalid velocity in waypoint " + std::to_string(i) + 
                         " (must be between 0 and "+ std::to_string(max_linear_speed_) + " m/s)";
            return true;
        }
        
        // 检查朝向是否有效（-π到π之间）
        if (wp.heading < -M_PI || wp.heading > M_PI) {
            res.success = false;
            res.message = "Invalid heading in waypoint " + std::to_string(i) + 
                         " (must be between -π and π)";
            return true;
        }
        
        // 检查航向角容差是否有效（0到π之间）
        if (wp.yaw_goal_tolerance < 0.0 || wp.yaw_goal_tolerance > M_PI) {
            res.success = false;
            res.message = "Invalid yaw_goal_tolerance in waypoint " + std::to_string(i) + 
                         " (must be between 0 and π)";
            return true;
        }
        
        // 检查平面位置容差是否有效（大于0且不超过10米）
        if (wp.xy_goal_tolerance <= 0.0 || wp.xy_goal_tolerance > 10.0) {
            res.success = false;
            res.message = "Invalid xy_goal_tolerance in waypoint " + std::to_string(i) + 
                         " (must be between 0 and 10 meters)";
            return true;
        }
        
        // 将航点转换到odom坐标系
        double odom_x, odom_y, odom_z;
        if (!latLonToLocal(wp.latitude, wp.longitude, wp.altitude, 
                          odom_x, odom_y, odom_z)) {
            res.success = false;
            res.message = "Failed to convert waypoint " + std::to_string(i) + 
                         " to odom coordinates";
            return true;
        }
        
        // 保存转换后的航点
        WaypointOdom wp_odom;
        wp_odom.position.x = odom_x;
        wp_odom.position.y = odom_y;
        wp_odom.position.z = odom_z;
        wp_odom.heading = wp.heading;
        wp_odom.velocity = wp.velocity;
        wp_odom.xy_goal_tolerance = wp.xy_goal_tolerance;
        wp_odom.yaw_goal_tolerance = wp.yaw_goal_tolerance;
        
        mission_waypoints_.push_back(wp);  // 保存原始航点
        mission_waypoints_odom_.push_back(wp_odom);  // 保存odom坐标系中的航点
    }
    
    current_waypoint_index_ = 0;
    
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
    
    while (ros::ok()) {
        // 检查是否有抢占请求
        if (navigate_to_target_server_.isPreemptRequested()) {
            // 发送零速度命令
            geometry_msgs::Twist cmd_vel;
            cmd_vel_publisher_.publish(cmd_vel);
            
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
            geometry_msgs::Twist cmd_vel;
            cmd_vel_publisher_.publish(cmd_vel);  // 发送零速度命令
            
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
            geometry_msgs::Twist cmd_vel;
            cmd_vel_publisher_.publish(cmd_vel);  // 发送零速度命令
            
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
                geometry_msgs::Twist cmd_vel;
                cmd_vel_publisher_.publish(cmd_vel);  // 发送零速度命令
                
                mars_navigation::NavigateToTargetResult result;
                result.success = false;
                result.message = "Navigation stopped";
                result.final_distance = 0.0;
                result.final_heading_error = 0.0;
                navigate_to_target_server_.setAborted(result);
                return;
            } else if (nav_state_ == NavigationState::PAUSED) {
                // 导航暂停时发布零速度并等待
                geometry_msgs::Twist cmd_vel;
                cmd_vel_publisher_.publish(cmd_vel);
                rate.sleep();
                continue;
            }
        }
        
        // 计算当前位置到目标的距离和朝向误差
        double distance = calculateDistance(path_start_odom_, path_end_odom_);
        double heading_error = calculateHeadingError(path_end_odom_);
        
        // 检查是否到达目标
        if (distance <= goal->target.xy_goal_tolerance && 
            std::abs(heading_error) <= goal->target.yaw_goal_tolerance) {
            geometry_msgs::Twist cmd_vel;
            cmd_vel_publisher_.publish(cmd_vel);  // 发送零速度命令
            
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
        previous_time = ros::Time::now();
        
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
    
    if (req.emergency_stop) {
        is_emergency_stop_ = true;
        is_mission_active_ = false;
        publishVelocityCommand(0.0, 0.0);
        
        res.success = true;
        res.message = "Emergency stop activated: " + req.reason;
        res.stop_distance = calculateDistance(path_start_odom_, path_end_odom_);
        return true;
    }
    
    res.success = false;
    res.message = "Invalid emergency stop request";
    return true;
}

void MarsNavigationNode::executeMissionCallback(const mars_navigation::ExecuteMissionGoalConstPtr& goal) {
    std::lock_guard<std::mutex> lock(mission_mutex_);
    
    // 检查是否处于自主模式
    if (!isAutonomousMode()) {
        mars_navigation::ExecuteMissionResult result;
        result.success = false;
        result.message = "Robot is not in autonomous mode";
        result.final_distance = 0.0;
        result.final_heading_error = 0.0;
        execute_mission_server_.setAborted(result);
        return;
    }
    
    // 检查里程计是否有效
    if (!isOdomValid()) {
        mars_navigation::ExecuteMissionResult result;
        result.success = false;
        result.message = "No valid odometry data available";
        result.final_distance = 0.0;
        result.final_heading_error = 0.0;
        execute_mission_server_.setAborted(result);
        return;
    }
    
    // 检查是否有有效的航点列表
    if (mission_waypoints_.empty()) {
        mars_navigation::ExecuteMissionResult result;
        result.success = false;
        result.message = "No valid mission waypoints available. Please upload mission first.";
        result.final_distance = 0.0;
        result.final_heading_error = 0.0;
        execute_mission_server_.setAborted(result);
        return;
    }
    
    // 重置航点索引
    current_waypoint_index_ = 0;
    is_mission_active_ = true;
    
    ros::Rate rate(control_frequency_);  // 使用参数配置的控制频率
    
    while (ros::ok() && is_mission_active_ && !is_emergency_stop_) {
        // 检查是否请求取消
        if (execute_mission_server_.isPreemptRequested()) {
            is_mission_active_ = false;
            mars_navigation::ExecuteMissionResult result;
            result.success = false;
            result.message = "Mission preempted";
            result.final_distance = calculateDistance(path_start_odom_, path_end_odom_);
            result.final_heading_error = calculateHeadingError(path_end_odom_);
            execute_mission_server_.setPreempted(result);
            return;
        }
        
        // 检查里程计是否有效
        if (!isOdomValid()) {
            is_mission_active_ = false;
            mars_navigation::ExecuteMissionResult result;
            result.success = false;
            result.message = "Lost odometry data";
            result.final_distance = calculateDistance(path_start_odom_, path_end_odom_);
            result.final_heading_error = calculateHeadingError(path_end_odom_);
            execute_mission_server_.setAborted(result);
            return;
        }
        
        // 检查是否完成所有航点
        if (current_waypoint_index_ >= mission_waypoints_.size()) {
            is_mission_active_ = false;
            mars_navigation::ExecuteMissionResult result;
            result.success = true;
            result.message = "Mission completed successfully";
            result.final_distance = 0.0;
            result.final_heading_error = 0.0;
            execute_mission_server_.setSucceeded(result);
            return;
        }
        
        // 更新当前路径段
        if (current_waypoint_index_ == 0) {
            // 如果是第一个航点，使用当前位置作为路径起点
            path_start_odom_ = current_odom_.pose.pose.position;
        } else {
            path_start_odom_ = mission_waypoints_odom_[current_waypoint_index_ - 1].position;
        }
        path_end_odom_ = mission_waypoints_odom_[current_waypoint_index_].position;
        
        // 计算到目标的距离和朝向误差
        double distance = calculateDistance(path_start_odom_, path_end_odom_);
        double heading_error = calculateHeadingError(path_end_odom_);
        
        // 发布反馈
        mars_navigation::ExecuteMissionFeedback feedback;
        feedback.current_waypoint_index = current_waypoint_index_;
        feedback.distance_to_target = distance;
        feedback.heading_error = heading_error;
        feedback.current_velocity = current_velocity_;
        feedback.battery_level = battery_level_;
        execute_mission_server_.publishFeedback(feedback);
        
        // 如果到达当前航点，移动到下一个航点
        if (distance < mission_waypoints_[current_waypoint_index_].xy_goal_tolerance && std::abs(heading_error) < mission_waypoints_[current_waypoint_index_].yaw_goal_tolerance) {
            current_waypoint_index_++;
            continue;
        }
        
        // 发布速度命令
        geometry_msgs::Twist cmd_vel;
        calculateVelocityCommand(
            path_start_odom_,
            path_end_odom_,
            current_odom_.pose.pose.position,
            mission_waypoints_odom_[current_waypoint_index_].velocity,
            cmd_vel
        );
        
        rate.sleep();
    }
    
    // 如果任务被中断
    if (!is_mission_active_) {
        mars_navigation::ExecuteMissionResult result;
        result.success = false;
        result.message = "Mission interrupted";
        result.final_distance = calculateDistance(path_start_odom_, path_end_odom_);
        result.final_heading_error = calculateHeadingError(path_end_odom_);
        execute_mission_server_.setAborted(result);
    }
}

double MarsNavigationNode::calculateDistance(const geometry_msgs::Point& wp1, const geometry_msgs::Point& wp2) {
    // 简化的距离计算（实际应用中应该使用更准确的地理距离计算方法）
    double dx = wp2.x - wp1.x;
    double dy = wp2.y - wp1.y;
    double dz = wp2.z - wp1.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

double MarsNavigationNode::calculateHeadingError(const geometry_msgs::Point& wp) {
    // 从四元数中提取偏航角
    double current_yaw = 2.0 * std::atan2(current_odom_.pose.pose.orientation.z, current_odom_.pose.pose.orientation.w);
    return wp.x - current_yaw;
}

void MarsNavigationNode::publishVelocityCommand(double linear_x, double angular_z) {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_x;
    cmd_vel.angular.z = angular_z;
    cmd_vel_publisher_.publish(cmd_vel);
}

void MarsNavigationNode::updateRobotStatus() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    
    // 更新机器人状态
    is_ready_ = !is_emergency_stop_ && battery_level_ > 20.0 && isOdomValid();
    
    // 如果电池电量过低或里程计无效，停止任务
    if (battery_level_ < 20.0 || !isOdomValid()) {
        is_mission_active_ = false;
        publishVelocityCommand(0.0, 0.0);
    }
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
        if (battery_level_ < 20.0) {
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

bool MarsNavigationNode::handleStopNavigation(std_srvs::Trigger::Request& req, 
                                            std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lock(nav_state_mutex_);
    if (nav_state_ != NavigationState::IDLE) {
        nav_state_ = NavigationState::IDLE;
        // 发送零速度命令停止机器人
        geometry_msgs::Twist cmd_vel;
        cmd_vel_publisher_.publish(cmd_vel);
        
        res.success = true;
        res.message = "Navigation stopped";
    } else {
        res.success = false;
        res.message = "No navigation in progress";
    }
    return true;
}

bool MarsNavigationNode::handlePauseNavigation(std_srvs::Trigger::Request& req, 
                                             std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lock(nav_state_mutex_);
    if (nav_state_ == NavigationState::RUNNING) {
        nav_state_ = NavigationState::PAUSED;
        // 发送零速度命令暂停机器人
        geometry_msgs::Twist cmd_vel;
        cmd_vel_publisher_.publish(cmd_vel);
        
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
    
    // 将路径向量单位化
    if (path_length > 0) {
        path_x /= path_length;
        path_y /= path_length;
    }
    
    // 计算机器人到路径起点的向量
    double robot_to_start_x = current_location.x - path_start.x;
    double robot_to_start_y = current_location.y - path_start.y;
    
    // 计算机器人在路径上的投影点
    double projection = robot_to_start_x * path_x + robot_to_start_y * path_y;
    
    // 计算横向误差（使用点到直线的距离公式）
    double lateral_error = std::abs(robot_to_start_x * path_y - robot_to_start_y * path_x);
    
    // 计算考虑减速的目标速度，并限制在最大速度范围内
    double limited_target_velocity = std::min(target_velocity, max_linear_speed_);
    double approach_velocity = calculateTargetVelocity(path_length - projection, limited_target_velocity);
    
    // 计算动态前视距离
    // lookahead = 1/(2*max_acceleration)*speed^2 + k2*lateral_error + initial_lookahead
    double dynamic_look_ahead = (1.0 / (2.0 * max_acceleration_)) * 
                               approach_velocity * approach_velocity +
                               k2_ * lateral_error +
                               look_ahead_distance_;
    
    // 限制前视距离在合理范围内
    dynamic_look_ahead = std::clamp(dynamic_look_ahead, 
                                  look_ahead_distance_ * 0.5,  // 最小前视距离
                                  look_ahead_distance_ * 2.0); // 最大前视距离
    
    // 计算前视点位置
    double look_ahead_projection = projection + dynamic_look_ahead;
    
    // 确保前视点不会超过终点
    if (look_ahead_projection > path_length) {
        look_ahead_projection = path_length;
    }
    
    // 计算前视点的实际位置
    geometry_msgs::Point look_ahead_point;
    look_ahead_point.x = path_start.x + path_x * look_ahead_projection;
    look_ahead_point.y = path_start.y + path_y * look_ahead_projection;
    look_ahead_point.z = path_start.z;  // 保持z坐标不变
    
    // 计算机器人到前视点的向量
    double robot_to_goal_x = look_ahead_point.x - current_location.x;
    double robot_to_goal_y = look_ahead_point.y - current_location.y;
    double distance_to_goal = std::sqrt(robot_to_goal_x * robot_to_goal_x + 
                                      robot_to_goal_y * robot_to_goal_y);
    
    // 获取机器人当前朝向
    double robot_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);
    
    // 计算目标点相对于机器人的角度
    double goal_angle = std::atan2(robot_to_goal_y, robot_to_goal_x);
    
    // 计算转向角误差
    double heading_error = angles::shortest_angular_distance(robot_yaw, goal_angle);
    
    // 计算曲率
    double curvature = 2.0 * std::sin(heading_error) / distance_to_goal;
    
    // 计算角速度
    double angular_velocity = approach_velocity * curvature;
    
    // 限制角速度
    angular_velocity = std::clamp(angular_velocity, 
                                -max_angular_velocity_, 
                                max_angular_velocity_);
    
    // 设置速度命令
    cmd_vel.linear.x = approach_velocity;
    cmd_vel.linear.y = 0.0;  // 差速转向机器人不能横向移动
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = angular_velocity;
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

int main(int argc, char** argv) {
    ros::init(argc, argv, "mars_navigation_node");
    ros::NodeHandle nh;
    
    MarsNavigationNode node(nh);
    
    ros::Rate rate(10);  // 10Hz
    
    while (ros::ok()) {
        node.executeMissionCallback();
        node.updateRobotStatus();
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
} 