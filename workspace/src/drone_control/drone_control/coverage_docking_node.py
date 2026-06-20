import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
import numpy as np


from drone_control.aruco_detector import ArucoMarkerDetector 



class DroneMissionNode(Node):
    def __init__(self):
        super().__init__('drone_mission_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)        
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop) 

        
        camera_matrix = [[530.0, 0.0, 320.0], [0.0, 530.0, 240.0], [0.0, 0.0, 1.0]]
        dist_coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.aruco_detector = ArucoMarkerDetector(camera_matrix, dist_coeffs)
        self.bridge = CvBridge()

        self.state = "SEARCHING"

        self.current_pose = None
        self.waypoints = self.generate_zigzag_waypoints()
        self.current_wp_index = 0
        self.tolerance = 0.25 

        # coefficients PD_controller
        self.kp_nav_xy = 0.5   # when drone flys zigzag
        self.kp_nav_z = 0.8
        
        self.kp_align_px = 0.002 # align marker to center using pixel error
        self.center_tolerance_px = 25.0
        self.marker_center_error = None


    def generate_zigzag_waypoints(self):
        waypoints = []
        z_hover = 2.0 
        
        # Không gian 8x8m (-4 đến 4)
        x_min, x_max = -4.0, 4.0
        y_min, y_max = -4.0, 4.0
        step = 1.5 
        
        waypoints.append((0.0, 0.0, z_hover))
        y = y_min
        direction = 1 
        
        while y <= y_max:
            if direction == 1:
                waypoints.append((x_min, y, z_hover))
                waypoints.append((x_max, y, z_hover))
            else:
                waypoints.append((x_max, y, z_hover))
                waypoints.append((x_min, y, z_hover))
            direction *= -1 
            y += step
            
        return waypoints


    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose.position


    def image_callback(self, msg):
        # Đã hạ cánh thì không cần xử lý ảnh nữa cho nhẹ máy
        if self.state == "LANDED":
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids = self.aruco_detector.detect_markers(cv_image)
        
        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i]

                if marker_id == 42:
                    self.get_logger().info("ID 42 detected: Bomb threat! Landing aborted!", throttle_duration_sec=2.0)
                elif marker_id == 23:
                    self.get_logger().info("ID 23 detected! Supply drop located!", throttle_duration_sec=2.0)
                
                elif marker_id == 50:
                    marker_points = corners[i][0]
                    marker_center = np.mean(marker_points, axis=0)
                    image_center = np.array([320.0, 240.0])
                    self.marker_center_error = marker_center - image_center


                    if self.state == "SEARCHING":
                        self.get_logger().info("Landing pad ID 3 detected! Aborting search, initiating alignment!")
                        self.state = "ALIGNING" 


    def control_loop(self):
        if self.current_pose is None:
            return 
            
        msg = Twist()

        if self.state == "SEARCHING":
            if self.current_wp_index >= len(self.waypoints):
                self.get_logger().info('Đã quét xong toàn bộ bản đồ nhưng không thấy bãi đáp.', throttle_duration_sec=3.0)
                self.cmd_vel_pub.publish(Twist()) 
                return

            target = self.waypoints[self.current_wp_index]
            ex = target[0] - self.current_pose.x
            ey = target[1] - self.current_pose.y
            ez = target[2] - self.current_pose.z
            
            distance = math.sqrt(ex**2 + ey**2 + ez**2)
            
            if distance < self.tolerance:
                self.current_wp_index += 1
                self.cmd_vel_pub.publish(Twist())
                return

            msg.linear.x = max(-1.0, min(1.0, self.kp_nav_xy * ex))
            msg.linear.y = max(-1.0, min(1.0, self.kp_nav_xy * ey))
            msg.linear.z = max(-0.5, min(0.5, self.kp_nav_z * ez))
            msg.angular.z = 0.0 
            
            self.cmd_vel_pub.publish(msg)

        # TRẠNG THÁI: CĂN CHỈNH TÂM (X, Y)
        elif self.state == "ALIGNING":
            if self.marker_center_error is None:
                self.cmd_vel_pub.publish(Twist())
                return # Đợi dữ liệu từ camera

            error_u = self.marker_center_error[0] # Lệch trái/phải trong ảnh
            error_v = self.marker_center_error[1] # Lệch trên/dưới trong ảnh

            # Camera nhìn xuống: dùng pixel error để đưa marker về giữa khung hình.
            msg.linear.x = max(-0.2, min(0.2, -self.kp_align_px * error_v))
            msg.linear.y = max(-0.2, min(0.2, -self.kp_align_px * error_u))
            msg.linear.z = 0.0 # Giữ nguyên độ cao trong lúc căn tâm
            msg.angular.z = 0.0

            self.cmd_vel_pub.publish(msg)

            # Nếu marker gần tâm ảnh, bắt đầu hạ độ cao
            if abs(error_u) < self.center_tolerance_px and abs(error_v) < self.center_tolerance_px:
                self.get_logger().info("Đã căn xong tâm. Bắt đầu hạ độ cao...")
                self.state = "DESCENT"


        # TRẠNG THÁI: HẠ ĐỘ CAO (Z)
        elif self.state == "DESCENT":
            if self.marker_center_error is None:
                self.state = "ALIGNING"
                self.cmd_vel_pub.publish(Twist())
                return

            error_u = self.marker_center_error[0]
            error_v = self.marker_center_error[1]

            # Vẫn bám tâm ảnh thật nhẹ trong lúc hạ.
            msg.linear.x = max(-0.12, min(0.12, -self.kp_align_px * error_v))
            msg.linear.y = max(-0.12, min(0.12, -self.kp_align_px * error_u))
            
            # Ép vận tốc Z âm để hạ cánh từ từ
            msg.linear.z = -0.18 
            msg.angular.z = 0.0
            
            self.cmd_vel_pub.publish(msg)

            # Odom z thấp thì coi như đã hạ cánh.
            if self.current_pose.z < 0.18:
                self.get_logger().info("HẠ CÁNH THÀNH CÔNG!")
                self.state = "LANDED"

        elif self.state == "LANDED":
            self.cmd_vel_pub.publish(Twist())       # Ngắt động cơ


def main(args=None):
    rclpy.init(args=args)
    node = DroneMissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()