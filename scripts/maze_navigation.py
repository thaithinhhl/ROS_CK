#!/usr/bin/env python3

import rospy
import math
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class MazeNavigator:
    def __init__(self):
        # Khởi tạo node
        rospy.init_node('maze_navigator', anonymous=True)
        
        # Tạo các biến trạng thái
        self.laser_data = None
        self.robot_position = [0, 0, 0]  # x, y, theta
        self.goal_reached = True
        self.min_front_distance = 0.6  # Tăng ngưỡng khoảng cách tối thiểu phía trước cho tường cây bụi
        self.min_side_distance = 0.5  # Khoảng cách tối thiểu ở bên cạnh
        self.current_goal = None
        
        # Đăng ký các subscribers
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Tạo publisher để điều khiển robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Tạo client để gửi mục tiêu
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        # Khởi tạo tf listener
        self.tf_listener = tf.TransformListener()
        
        rospy.loginfo("Maze Navigator đã được khởi tạo!")
        rospy.sleep(1)  # Chờ subscribers kết nối
        
    def laser_callback(self, data):
        """Hàm callback nhận dữ liệu từ laser scan"""
        self.laser_data = data
        
    def odom_callback(self, data):
        """Hàm callback nhận dữ liệu từ odometry"""
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        
        # Chuyển quaternion thành Euler angles
        (_, _, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        
        self.robot_position = [position.x, position.y, yaw]
        
    def get_front_distance(self):
        """Trả về khoảng cách ngắn nhất phía trước robot"""
        if self.laser_data is None:
            return float('inf')
        
        # Lấy dữ liệu laser ở phía trước robot (±15 độ) - thu hẹp góc cho chính xác hơn
        front_range_size = len(self.laser_data.ranges)
        center_idx = front_range_size // 2
        angle_range = front_range_size // 12  # Khoảng ±15 độ
        
        front_indices = range(center_idx - angle_range, center_idx + angle_range)
        front_ranges = [self.laser_data.ranges[i % front_range_size] for i in front_indices]
        
        # Lọc các giá trị vô hạn hoặc NaN
        valid_ranges = [r for r in front_ranges if r < self.laser_data.range_max and not math.isnan(r)]
        
        if len(valid_ranges) > 0:
            return min(valid_ranges)
        else:
            return float('inf')
            
    def get_left_distance(self):
        """Trả về khoảng cách bên trái robot"""
        if self.laser_data is None:
            return float('inf')
        
        front_range_size = len(self.laser_data.ranges)
        left_center = (front_range_size * 3) // 4  # 270 độ
        angle_range = front_range_size // 12  # Khoảng ±15 độ
        
        left_indices = range(left_center - angle_range, left_center + angle_range)
        left_ranges = [self.laser_data.ranges[i % front_range_size] for i in left_indices]
        valid_ranges = [r for r in left_ranges if r < self.laser_data.range_max and not math.isnan(r)]
        
        if len(valid_ranges) > 0:
            return min(valid_ranges)
        else:
            return float('inf')
            
    def get_right_distance(self):
        """Trả về khoảng cách bên phải robot"""
        if self.laser_data is None:
            return float('inf')
        
        front_range_size = len(self.laser_data.ranges)
        right_center = front_range_size // 4  # 90 độ
        angle_range = front_range_size // 12  # Khoảng ±15 độ
        
        right_indices = range(right_center - angle_range, right_center + angle_range)
        right_ranges = [self.laser_data.ranges[i % front_range_size] for i in right_indices]
        valid_ranges = [r for r in right_ranges if r < self.laser_data.range_max and not math.isnan(r)]
        
        if len(valid_ranges) > 0:
            return min(valid_ranges)
        else:
            return float('inf')
    
    def send_goal(self, x, y):
        """Gửi mục tiêu đến move_base"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        
        rospy.loginfo("Gửi mục tiêu: (%s, %s)", x, y)
        self.move_base_client.send_goal(goal)
        self.current_goal = [x, y]
        self.goal_reached = False
    
    def wall_following(self):
        """
        Thuật toán wall following cải tiến cho mê cung cây bụi
        Robot sẽ đi theo tường bên phải nhưng cẩn thận hơn
        """
        front_distance = self.get_front_distance()
        right_distance = self.get_right_distance()
        left_distance = self.get_left_distance()
        
        twist = Twist()
        
        # In ra thông tin khoảng cách để debug
        rospy.loginfo("Khoảng cách - Trước: %.2f, Phải: %.2f, Trái: %.2f", 
                      front_distance, right_distance, left_distance)
        
        if front_distance < self.min_front_distance:
            # Nếu phía trước có vật cản, quay trái với tốc độ chậm hơn
            rospy.loginfo("Vật cản phía trước, quay trái")
            twist.angular.z = 0.3  # Giảm tốc độ quay
            twist.linear.x = 0.0   # Dừng di chuyển khi quay
        elif right_distance > 1.0:
            # Nếu bên phải không có tường, rẽ phải
            rospy.loginfo("Không có tường bên phải, rẽ phải")
            twist.linear.x = 0.08  # Giảm tốc độ
            twist.angular.z = -0.3 # Giảm tốc độ quay
        elif right_distance < self.min_side_distance:
            # Nếu quá gần tường bên phải, rẽ trái
            rospy.loginfo("Quá gần tường bên phải, rẽ trái")
            twist.linear.x = 0.08  # Giảm tốc độ
            twist.angular.z = 0.3  # Giảm tốc độ quay
        else:
            # Đi thẳng dọc theo tường với tốc độ chậm hơn
            rospy.loginfo("Đi thẳng dọc theo tường")
            twist.linear.x = 0.15  # Giảm tốc độ
            
            # Điều chỉnh nhẹ để giữ khoảng cách với tường
            error = right_distance - 0.7  # Khoảng cách mục tiêu đến tường
            twist.angular.z = -0.1 * error  # Điều chỉnh P controller
            
        self.cmd_vel_pub.publish(twist)
    
    def run(self):
        """Hàm chính để chạy thuật toán navigation"""
        rate = rospy.Rate(10)  # 10Hz
        
        rospy.loginfo("Bắt đầu thuật toán wall following cho mê cung cây bụi")
        
        while not rospy.is_shutdown():
            # Sử dụng thuật toán wall following để điều hướng trong mê cung
            self.wall_following()
            
            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = MazeNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass 