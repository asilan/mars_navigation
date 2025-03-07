#!/usr/bin/env python3

import rospy
from mars_navigation.srv import UploadMission, NavigateToTarget, GetRobotStatus, EmergencyStop
from mars_navigation.msg import Waypoint
import math

def test_upload_mission():
    """测试上传任务服务"""
    rospy.wait_for_service('/mars_navigation/upload_mission')
    try:
        upload_mission = rospy.ServiceProxy('/mars_navigation/upload_mission', UploadMission)
        
        # 创建测试航点列表
        waypoints = []
        
        # 添加一些测试航点
        waypoints.append(Waypoint(
            latitude=0.0,
            longitude=0.0,
            altitude=0.0,
            velocity=1.0,
            heading=0.0
        ))
        
        waypoints.append(Waypoint(
            latitude=1.0,
            longitude=1.0,
            altitude=10.0,
            velocity=2.0,
            heading=math.pi/2
        ))
        
        waypoints.append(Waypoint(
            latitude=2.0,
            longitude=2.0,
            altitude=20.0,
            velocity=1.5,
            heading=math.pi
        ))
        
        # 发送请求
        response = upload_mission(waypoints)
        rospy.loginfo(f"Upload Mission Response: Success={response.success}, Message={response.message}")
        return response.success
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def test_navigate_to_target():
    """测试导航到目标服务"""
    rospy.wait_for_service('/mars_navigation/navigate_to_target')
    try:
        navigate = rospy.ServiceProxy('/mars_navigation/navigate_to_target', NavigateToTarget)
        
        # 发送导航请求
        response = navigate(
            latitude=1.0,
            longitude=1.0,
            altitude=10.0,
            max_velocity=2.0,
            max_acceleration=1.0,
            heading=math.pi/2
        )
        
        rospy.loginfo(f"Navigate Response: Success={response.success}, Message={response.message}")
        rospy.loginfo(f"Final Distance: {response.final_distance}m, Heading Error: {response.final_heading_error}rad")
        return response.success
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def test_get_robot_status():
    """测试获取机器人状态服务"""
    rospy.wait_for_service('/mars_navigation/get_robot_status')
    try:
        get_status = rospy.ServiceProxy('/mars_navigation/get_robot_status', GetRobotStatus)
        
        # 发送状态请求
        response = get_status(True)
        
        rospy.loginfo(f"Robot Status: Ready={response.is_ready}, Battery={response.battery_level}%")
        rospy.loginfo(f"Current Velocity: {response.current_velocity}m/s")
        rospy.loginfo(f"Status Message: {response.status_message}")
        return True
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def test_emergency_stop():
    """测试紧急停止服务"""
    rospy.wait_for_service('/mars_navigation/emergency_stop')
    try:
        emergency_stop = rospy.ServiceProxy('/mars_navigation/emergency_stop', EmergencyStop)
        
        # 发送紧急停止请求
        response = emergency_stop(True, "Test emergency stop")
        
        rospy.loginfo(f"Emergency Stop Response: Success={response.success}, Message={response.message}")
        rospy.loginfo(f"Stop Distance: {response.stop_distance}m")
        return response.success
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def main():
    """主测试函数"""
    rospy.init_node('mars_navigation_test')
    rospy.loginfo("Starting Mars Navigation Test")
    
    # 测试所有服务
    tests = [
        ("Upload Mission", test_upload_mission),
        ("Navigate to Target", test_navigate_to_target),
        ("Get Robot Status", test_get_robot_status),
        ("Emergency Stop", test_emergency_stop)
    ]
    
    for test_name, test_func in tests:
        rospy.loginfo(f"\nTesting {test_name}...")
        success = test_func()
        rospy.loginfo(f"{test_name} test {'passed' if success else 'failed'}")
    
    rospy.loginfo("\nAll tests completed")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 