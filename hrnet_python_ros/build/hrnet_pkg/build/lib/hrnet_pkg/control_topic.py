import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Int32MultiArray
import math
from collections import deque

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription_arm = self.create_subscription(Int32MultiArray, 'chatter', self.get_target, 10)
        self.subscription_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        self.current_pose = None
        self.target_theta = None
        self.linear_speed = None # 线速度，可以根据需要调整
        self.angle_diff_history = deque(maxlen=5)
  
    def pose_callback(self, msg):
        self.current_pose = msg
        print(f'pose: {msg}')
        if self.target_theta and self.linear_speed is not None:
            self.control_turtle_orientation_and_move()

    def get_target(self, msg):
        #self.get_logger().info(f'msg.data {msg.data}')
        self.target_theta = int(msg.data[0])
        self.target_theta = math.radians(self.target_theta)
        self.right_theta = int(msg.data[1])
        if -110 < self.right_theta < -80:
            self.linear_speed = 0
        elif -360 < self.right_theta < -110:
            self.linear_speed = 3

        elif -80 < self.right_theta < -0:
            self.linear_speed = 1
        else :
            self.linear_speed = self.linear_speed
        print(f'get_msg: theta:{self.target_theta} v:{self.linear_speed}')
        

    def control_turtle_orientation_and_move(self):

        # 计算角度差
        angle_diff = self.target_theta - self.current_pose.theta
        print(f'angle_diff : {angle_diff}')
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # 将角度差规范化为[-pi, pi]
        # 设置速度
        angle_diff = -angle_diff 
        vel_msg = Twist()

        if abs(angle_diff) < 0.1:
            vel_msg.angular.z = vel_msg.angular.z  # 角速度与角度差成比例

        else:
            vel_msg.angular.z = 1.0 * angle_diff  # 角速度与角度差成比例
        
        vel_msg.linear.x = float(self.linear_speed) 
        # 发布消息
        self.publisher.publish(vel_msg)
            

    def set_target_orientation(self, theta):
        self.target_theta = theta
        if self.current_pose is not None:
            self.control_turtle_orientation_and_move()


def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    # 设定海龟的目标朝向（例如，朝向90度，即π/2弧度）
    target_theta = math.pi / 2
    # 90度
    turtle_controller.set_target_orientation(target_theta)
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()