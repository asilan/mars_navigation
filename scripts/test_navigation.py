#!/usr/bin/env python3

import rospy
import actionlib
import math
import numpy as np
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist, Quaternion, Vector3
from mars_navigation.msg import (
    Waypoint,
    ExecuteMissionAction,
    ExecuteMissionGoal,
    NavigateToTargetAction,
    NavigateToTargetGoal
)
from mars_navigation.srv import UploadMission, UploadMissionRequest
from std_srvs.srv import Trigger
from mars_canopen.msg import RobotMode
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class NavigationTester:
    def __init__(self):
        rospy.init_node('navigation_tester')
        
        # 初始化模拟机器人状态
        self.robot_pose = Pose()
        self.robot_twist = Twist()
        self.last_cmd_vel = Twist()
        self.last_cmd_time = rospy.Time.now()
        
        # 初始化TF广播器
        self.tf_broadcaster = TransformBroadcaster()
        
        # 设置UTM原点对应的经纬度（大约在北京附近，50N带）
        self.base_latitude = 29.641968    # 对应约4000000北向坐标
        self.base_longitude = 277.678306  # 对应约500000东向坐标
        
        self.utm_to_odom = TransformStamped()
        self.utm_to_odom.header.frame_id = "utm"
        self.utm_to_odom.child_frame_id = "odom"
        self.utm_to_odom.transform.translation.x = 372064.74  # UTM东向坐标
        self.utm_to_odom.transform.translation.y = 3279843.47  # UTM北向坐标
        self.utm_to_odom.transform.translation.z = 0.0
        # 设置旋转（假设没有旋转）
        q = quaternion_from_euler(0, 0, 0)
        self.utm_to_odom.transform.rotation.x = q[0]
        self.utm_to_odom.transform.rotation.y = q[1]
        self.utm_to_odom.transform.rotation.z = q[2]
        self.utm_to_odom.transform.rotation.w = q[3]
        
        # 发布器和订阅器
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.mode_pub = rospy.Publisher('robot_mode', RobotMode, queue_size=1)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        # 模拟更新定时器 (50Hz)
        self.update_timer = rospy.Timer(rospy.Duration(0.02), self.update_robot_state)
        
        # 等待导航服务和action服务器
        rospy.loginfo("Waiting for navigation services...")
        rospy.wait_for_service('upload_mission')
        # rospy.wait_for_service('stop_navigation')
        rospy.wait_for_service('pause_navigation')
        rospy.wait_for_service('resume_navigation')
        
        # 创建服务客户端
        self.upload_mission = rospy.ServiceProxy('upload_mission', UploadMission)
        self.stop_navigation = rospy.ServiceProxy('stop_navigation', Trigger)
        self.pause_navigation = rospy.ServiceProxy('pause_navigation', Trigger)
        self.resume_navigation = rospy.ServiceProxy('resume_navigation', Trigger)
        
        # 创建action客户端
        self.execute_mission_client = actionlib.SimpleActionClient(
            'execute_mission', ExecuteMissionAction)
        self.navigate_to_target_client = actionlib.SimpleActionClient(
            'navigate_to_target', NavigateToTargetAction)
            
        rospy.loginfo("Waiting for action servers...")
        self.execute_mission_client.wait_for_server()
        self.navigate_to_target_client.wait_for_server()
        
        rospy.loginfo("Navigation tester is ready")
        
    def cmd_vel_callback(self, msg):
        """处理接收到的速度命令"""
        self.last_cmd_vel = msg
        self.last_cmd_time = rospy.Time.now()
        
    def update_robot_state(self, event):
        """更新模拟的机器人状态"""
        now = rospy.Time.now()
        
        # 更新并发布UTM到odom的变换
        self.utm_to_odom.header.stamp = now
        self.tf_broadcaster.sendTransform(self.utm_to_odom)
        
        # 更新机器人状态
        dt = (now - self.last_cmd_time).to_sec()
        
        # 如果1秒内没有收到新的速度命令，认为速度为0
        if dt > 1.0:
            self.last_cmd_vel = Twist()
        
        # 更新位置和方向
        current_yaw = self.get_yaw_from_quaternion(self.robot_pose.orientation)
        
        # 计算位置变化
        dx = self.last_cmd_vel.linear.x * math.cos(current_yaw) * 0.02
        dy = self.last_cmd_vel.linear.x * math.sin(current_yaw) * 0.02
        dyaw = self.last_cmd_vel.angular.z * 0.02
        
        # 更新位置
        self.robot_pose.position.x += dx
        self.robot_pose.position.y += dy
        
        # 更新方向
        new_yaw = current_yaw + dyaw
        q = quaternion_from_euler(0, 0, new_yaw)
        self.robot_pose.orientation = Quaternion(*q)
        
        # 创建并发布里程计消息
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose = self.robot_pose
        odom.twist.twist = self.last_cmd_vel
        
        self.odom_pub.publish(odom)
        
    def get_yaw_from_quaternion(self, q):
        """从四元数中提取yaw角"""
        # 使用arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
    
    def wait_for_position(self, target_x, target_y, tolerance=0.1, timeout=30.0):
        """等待机器人到达指定位置"""
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            dx = self.robot_pose.position.x - target_x
            dy = self.robot_pose.position.y - target_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance <= tolerance:
                return True
                
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("Timeout waiting for position")
                return False
                
            rate.sleep()
    
    def set_robot_mode(self, mode):
        """设置机器人模式"""
        msg = RobotMode()
        msg.mode = mode
        self.mode_pub.publish(msg)
        rospy.sleep(0.5)  # 等待模式切换
        
    def create_test_waypoints(self):
        """创建测试航点"""
        waypoints = []
        
        # 使用与UTM原点匹配的基准经纬度
        base_lat = self.base_latitude
        base_lon = self.base_longitude
        delta = 0.0001  # 约10米的距离
        
        # 创建一个正方形路径的点
        points = [
            (base_lat, base_lon, 0),                    # 起点
            (base_lat, base_lon + delta, math.pi/2),            # 向东
            (base_lat + delta, base_lon + delta, -math.pi),    # 向北
            (base_lat + delta, base_lon, -math.pi/2),            # 向西
            (base_lat, base_lon, 0)                     # 返回起点
        ]
        
        for lat, lon, heading in points:
            wp = Waypoint()
            wp.latitude = lat
            wp.longitude = lon
            wp.altitude = 0.0
            wp.velocity = 0.5
            wp.heading = heading
            wp.xy_goal_tolerance = 0.1
            wp.yaw_goal_tolerance = 0.1
            waypoints.append(wp)
            
            rospy.loginfo(f"添加航点: 纬度={lat:.6f}, 经度={lon:.6f}")
        
        return waypoints
        
    def upload_mission(self, waypoints):
        """上传任务航点"""
        try:
            rospy.loginfo("等待上传任务服务...")
            rospy.wait_for_service('upload_mission', timeout=5.0)
            upload_mission = rospy.ServiceProxy('upload_mission', UploadMission)
            
            # 创建请求
            req = UploadMissionRequest()
            req.waypoints = waypoints
            
            rospy.loginfo(f"开始上传 {len(waypoints)} 个航点...")
            # 设置服务调用超时
            response = upload_mission(req)
            
            if response.success:
                rospy.loginfo("任务上传成功")
                return True
            else:
                rospy.logerr(f"任务上传失败: {response.message}")
                return False
            
        except rospy.ROSException as e:
            rospy.logerr(f"等待服务超时: {str(e)}")
            return False
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {str(e)}")
            return False

    def execute_mission_feedback_cb(self, feedback):
        """任务执行反馈回调"""
        rospy.loginfo("Mission feedback - Waypoint: %d, Distance: %.2f, Heading error: %.2f, Velocity: %.2f",
                     feedback.current_waypoint,
                     feedback.distance_to_waypoint,
                     feedback.heading_error,
                     feedback.current_velocity)
                     
    def test_execute_mission(self):
        """测试任务执行"""
        # 设置为自主模式
        self.set_robot_mode(1)
        
        # 重置机器人位置
        self.robot_pose = Pose()
        q = quaternion_from_euler(0, 0, 0)
        self.robot_pose.orientation = Quaternion(*q)
        
        # 创建并发送目标
        goal = ExecuteMissionGoal()
        
        rospy.loginfo("Starting mission execution...")
        self.execute_mission_client.send_goal(
            goal,
            feedback_cb=self.execute_mission_feedback_cb
        )
        
        # # 等待任务完成
        # finished = self.execute_mission_client.wait_for_result(rospy.Duration(60.0))
        
        # if finished:
        #     state = self.execute_mission_client.get_state()
        #     result = self.execute_mission_client.get_result()
        #     rospy.loginfo("Mission completed with state: %d", state)
        #     rospy.loginfo("Final position: (%.2f, %.2f)", 
        #                  self.robot_pose.position.x,
        #                  self.robot_pose.position.y)
        #     return result.success
        # else:
        #     rospy.logwarn("Mission did not complete within timeout")
        #     return False
            
    def test_navigation_controls(self):
        """测试导航控制（暂停/恢复/停止）"""
        # 开始任务执行
        goal = ExecuteMissionGoal()
        self.execute_mission_client.send_goal(
            goal,
            feedback_cb=self.execute_mission_feedback_cb
        )
        rospy.sleep(2.0)
        
        # 记录暂停时的位置
        pause_position = Point(
            self.robot_pose.position.x,
            self.robot_pose.position.y,
            self.robot_pose.position.z
        )
        
        # 测试暂停
        rospy.loginfo("Testing pause...")
        response = self.pause_navigation()
        rospy.loginfo("Pause result: %s", response.message)
        rospy.sleep(2.0)
        
        # 验证机器人是否真的停止
        if self.last_cmd_vel.linear.x != 0 or self.last_cmd_vel.angular.z != 0:
            rospy.logerr("Robot did not stop after pause command")
        
        # 测试恢复
        rospy.loginfo("Testing resume...")
        response = self.resume_navigation()
        rospy.loginfo("Resume result: %s", response.message)
        rospy.sleep(2.0)
        
        # 测试停止
        rospy.loginfo("Testing stop...")
        response = self.stop_navigation()
        rospy.loginfo("Stop result: %s", response.message)
        
        # 验证机器人是否完全停止
        if self.last_cmd_vel.linear.x != 0 or self.last_cmd_vel.angular.z != 0:
            rospy.logerr("Robot did not stop after stop command")

    def run_tests(self):
        """运行测试序列"""
        rospy.loginfo("开始导航测试...")
        
        # 设置机器人为自动模式
        self.set_robot_mode(1)  # 1 表示自动模式
        rospy.sleep(1.0)  # 等待模式切换生效
        
        # 创建并上传测试航点
        waypoints = self.create_test_waypoints()
        rospy.loginfo(f"生成了 {len(waypoints)} 个测试航点")
        
        # 尝试上传任务，如果失败则退出
        if not self.upload_mission(waypoints):
            rospy.logerr("任务上传失败，测试终止")
            return
        
        rospy.loginfo("等待2秒后开始执行任务...")
        rospy.sleep(2.0)
        
        # 继续其他测试...
        self.test_execute_mission()
        rospy.sleep(10.0)
        self.test_navigation_controls()


if __name__ == '__main__':
    try:
        tester = NavigationTester()
        tester.run_tests()
    except rospy.ROSInterruptException:
        pass 