#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus

class GoalNavigator:
    def __init__(self):
        rospy.init_node('goal_navigator', anonymous=True)
        
        # Khởi tạo action client để giao tiếp với move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Đợi kết nối với move_base server...")
        self.client.wait_for_server()
        rospy.loginfo("Đã kết nối với move_base server!")
        
        # Subscribe để nhận mục tiêu từ RViz
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
    def goal_callback(self, data):
        """Callback khi nhận được mục tiêu mới từ RViz"""
        goal = MoveBaseGoal()
        goal.target_pose = data
        
        rospy.loginfo("Nhận mục tiêu mới:")
        rospy.loginfo("Position: x=%.2f, y=%.2f", 
                     data.pose.position.x, 
                     data.pose.position.y)
        
        # Gửi mục tiêu đến move_base
        self.client.send_goal(goal, 
                            done_cb=self.done_callback,
                            feedback_cb=self.feedback_callback)
        
    def done_callback(self, status, result):
        """Callback khi hoàn thành mục tiêu"""
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Đã đến mục tiêu!")
        else:
            rospy.loginfo("Không thể đến mục tiêu: %d", status)
            
    def feedback_callback(self, feedback):
        """Callback để nhận phản hồi về tiến trình"""
        # Có thể thêm xử lý phản hồi ở đây nếu cần
        pass
        
    def run(self):
        """Chạy node"""
        rospy.loginfo("Goal Navigator đã sẵn sàng!")
        rospy.loginfo("Sử dụng nút '2D Nav Goal' trong RViz để đặt mục tiêu")
        rospy.spin()

if __name__ == '__main__':
    try:
        navigator = GoalNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass 