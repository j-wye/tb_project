import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class TurtleBot4USAR(Node):
    def __init__(self):
        super().__init__('drive')
        self.twist = Twist()
        self.qos_policy = QoSProfile(reliability = ReliabilityPolicy.RELIABLE,history = HistoryPolicy.KEEP_LAST, depth=1)

        # LaserScan ? ?½? ???? κ΅¬λ ??±
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_cb, self.qos_policy)

        # cmd_vel ? ?½? ???? λ°ν? ??±
        self.publisher = self.create_publisher(Twist, '/cmd_vel', self.qos_policy)
        self.odom_sub = self.create_subscription(PoseWithCovarianceStamped, '/odom', self.odom_cb, self.qos_policy)
        self.initial_pose_origin_x = None
        self.initial_pose_origin_y = None
        self.current_pose = None

    def odom_cb(self, data):
        self.current_pose = data.pose.pose
        if not (self.initial_pose_origin_x and self.initial_pose_origin_y):
            self.initial_pose_origin_x = data.pose.pose.position.x
            self.initial_pose_origin_y = data.pose.pose.position.y

    def lidar_cb(self, msg):
        # LaserScan?? κ°λ ? λ³? μΆμΆ
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        
        # λ‘λ΄? ? λ©΄μ ???? ?Έ?±?€ κ³μ°
        index_front = round((-1.5708 - angle_min) / angle_increment)  # -1.5708 rad? -90?
        index_start = max(0, index_front - 37)
        index_end = min(len(msg.ranges) - 1, index_front + 37)
        front_ranges = msg.ranges[index_start:index_end+1]

        # ? ?¨? κ°μ΄ ??μ§? ??Έ?κ³?, ??Όλ©? κΈ°λ³Έκ°? ?€? 
        valid_front_ranges = [x for x in front_ranges if x != float('inf')]
        if valid_front_ranges:
            front_distance = min(valid_front_ranges)
        else:
            front_distance = float('inf')  # ? ?¨? κ°μ΄ ??Όλ©? λ¬΄ν???λ‘? ?€? 

        # ?€λ₯Έμͺ½ λ°©ν₯? ???? ?Έ?±?€ κ³μ°
        index_right = round((1.5708 - angle_min) / angle_increment)  # 1.5708 rad? 90?
        index_right_start = max(0, index_right - 37)
        index_right_end = min(len(msg.ranges) - 1, index_right + 37)
        right_ranges = msg.ranges[index_right_start:index_right_end+1]

        # ? ?¨? κ°μ΄ ??μ§? ??Έ?κ³?, ??Όλ©? κΈ°λ³Έκ°? ?€? 
        valid_right_ranges = [x for x in right_ranges if x != float('inf')]
        if valid_right_ranges:
            right_distance = min(valid_right_ranges)
        else:
            right_distance = float('inf') 

        angle_right_forward = -0.785398  # -45? (-??/4 rad)
        index_right_forward = round((angle_right_forward - angle_min) / angle_increment)
        right_forward_distance = msg.ranges[index_right_forward] if msg.ranges[index_right_forward] != float('inf') else float('inf')

        # ?? ?₯? λ¬Όμ΄ ?? κ²½μ° ?? 
        if front_distance < 0.6:
            self.twist.linear.x = 0.0
            self.twist.angular.z = math.pi / 2  # κ°ν ?? ? ??΄ κ°? μ¦κ??
        
        elif right_distance > 0.3:
            if right_forward_distance < 1.0:
                # ?€λ₯Έμͺ½ λ²½κ³Ό? κ±°λ¦¬κ°? λ©? κ²½μ°, ?€λ₯Έμͺ½?Όλ‘? κ°νκ²? κΈ°μΈκΈ?
                self.twist.linear.x = 0.2
                self.twist.angular.z = -0.1
                print("Case 1")
            else:
                self.twist.linear.x = 0.3
                self.twist.angular.z = -1.0  # κ°ν ?€λ₯Έμͺ½ κΈ°μΈκΈ°λ?? ??΄ κ°? μ¦κ??
                print("Case 2")
        
        elif right_distance < 0.5:
            # ?€λ₯Έμͺ½ λ²½κ³Ό? κ±°λ¦¬κ°? κ°?κΉμΈ κ²½μ°, ?Όμͺ½μΌλ‘? κ°νκ²? κΈ°μΈκΈ?
            self.twist.linear.x = 0.5
            self.twist.angular.z = 0.3  # κ°ν ?Όμͺ? κΈ°μΈκΈ°λ?? ??΄ κ°? μ¦κ??
            print("Case 3")
        
        else:
            # ?€λ₯Έμͺ½ λ²½μ ?°?Ό μ§μ§
            self.twist.linear.x = 0.65
            self.twist.angular.z = 0.0
            print("Case 4")

        # ???μ§μ λͺλ Ή λ°ν
        self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    usar_turtlebot = TurtleBot4USAR()
    try:
        while rclpy.ok():
            rclpy.spin(usar_turtlebot)
    except KeyboardInterrupt:
        pass
    finally:
        usar_turtlebot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()