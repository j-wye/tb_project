import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import cv2, math, tf_transformations
import pyzbar.pyzbar as pyzbar
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

def quaternion_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))

class QRCodeDetector(Node):
    def __init__(self):
        super().__init__('qr_code_detector')
        self.bridge = CvBridge()
        self.twist = Twist()
        self.qos_policy = QoSProfile(reliability = ReliabilityPolicy.RELIABLE,history = HistoryPolicy.KEEP_LAST, depth=1)
        self.qr_count_pub = self.create_publisher(Int32, '/qr_code_count', self.qos_policy)
        self.marker_pub = self.create_publisher(Marker, 'qr_code_visualization_maker', self.qos_policy)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', self.qos_policy)

        self.odom_sub = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.odom_cb, self.qos_policy)
        self.img_sub = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.img_cb, self.qos_policy)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_cb, self.qos_policy)
        self.detected_qr_codes = []
        self.current_lidar_ranges = []
        self.initial_pose_set = False
        self.current_pose = None
    
    def odom_cb(self, data):
        self.current_pose = data.pose.pose
        if not self.initial_pose_set:
            self.initial_pose = data.pose.pose
            self.initial_pose_set = True

    def lidar_cb(self, data):
        self.current_lidar_ranges = data.ranges
    
    def distance(self):
        return math.sqrt((self.current_pose.position.x - self.initial_pose.position.x) ** 2 + (self.current_pose.position.y - self.initial_pose.position.y) ** 2)

    def img_cb(self, data):
        # ros msg�� ���Ź޾Ƽ� cv2 msg�� �ؼ��� �� �ֵ��� ����
        try:
            img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return
        
        # pyzbar�� ���ؼ� qr code Ž��
        qr_codes = pyzbar.decode(img)
        for qr in qr_codes:
            qr_data = qr.data.decode('utf-8')
            # ���� ���� ���� QR Code�� ������ �о��� ���� �ƴ϶�� (��, ��ġ�� �ʴ´ٸ�)
            if qr_data not in self.detected_qr_codes:
                # ��ġ�� ���� ��쿡�� append
                self.detected_qr_codes.append(qr_data)
                # ������� ���� qr code�� ������ qr_count_pub�� ���ؼ� publish
                self.qr_count_pub.publish(Int32(data=len(self.detected_qr_codes)))
                print(len(self.detected_qr_codes))
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.vel_pub.publish(self.twist)
                # ��� ���߰� ���� �Լ��� ���ؼ� qr code������ �����Ÿ� ���
                distance = self.calculate_distance_to_qr()
                # qr code�� ������ ��ġ�� ���
                qr_position = self.calculate_qr_position(distance)
                if qr_position:
                    # ���� qr_position�� ���� �ش� ��ġ�� marker�� ��� �۾�
                    self.add_marker_at_qr_code(qr_position)
                else:
                    self.get_logger().warn("QR position is None, cannot add marker")
    
    def calculate_distance_to_qr(self):
        # �����Ÿ� ��꿡 ���� ����� ������ lidar���� Ȱ��
        if not self.current_lidar_ranges:
            return None
        mid_index = len(self.current_lidar_ranges) // 4
        return self.current_lidar_ranges[mid_index]
    

    def calculate_qr_position(self, distance):
        qx, qy, qz, qw = self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w
        yaw = quaternion_to_yaw(qx, qy, qz, qw)
        qr_x = self.current_pose.position.x + distance * math.cos(yaw)
        qr_y = self.current_pose.position.y + distance * math.sin(yaw)
        return [qr_x, qr_y]
    
    def add_marker_at_qr_code(self, qr_position):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "qr_code"
        marker.id = len(self.detected_qr_codes)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = qr_position[0]
        marker.pose.position.y = qr_position[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.5 + 0.1 * marker.id
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    qr_code_detector = QRCodeDetector()
    rclpy.spin(qr_code_detector)
    qr_code_detector.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()