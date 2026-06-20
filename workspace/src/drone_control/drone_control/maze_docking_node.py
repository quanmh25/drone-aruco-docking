import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import math
import numpy as np


from drone_control.aruco_detector import ArucoMarkerDetector 



class DroneMazeMissionNode(Node):
    def __init__(self):
        super().__init__('drone_maze_mission_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.lidar = self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.timer = self.create_timer(0.05, self.control_loop) 

        camera_matrix = [[530.0, 0.0, 320.0], [0.0, 530.0, 240.0], [0.0, 0.0, 1.0]]
        dist_coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.aruco_detector = ArucoMarkerDetector(camera_matrix, dist_coeffs)
        self.bridge = CvBridge()

        self.current_pose = None
        self.marker_center_error = None
        
        # --- HỆ SỐ PD CONTROLLER ---
        self.kp_z = 0.5
        self.z_hover = 1.6
  
        self.kp_align_px = 0.002 # Căn tâm marker bằng sai số pixel
        self.center_tolerance_px = 25.0
        
        # for lidar
        self.front_min = float('inf')
        self.left_min = float('inf')
        self.right_min = float('inf')

        #state machine
        self.state = "TAKEOFF"
        self.mission = "GO"
        self.avoid_dir = None

        self.stop_dist = 0.3  
        self.clear_dist = 0.5    


    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))


    def hold_altitude_velocity(self):
        altitude_error = self.z_hover - self.current_pose.z
        if abs(altitude_error) <= 0.05:
            return 0.0

        return self.clamp(self.kp_z * altitude_error, -0.25, 0.25)


    def lidar_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        for i in range(len(ranges)):
            if not np.isfinite(ranges[i]) or ranges[i] < 0.02:
                ranges[i] = np.nan

        def sector(angle_start, angle_end):
            a_start = math.radians(angle_start)
            a_end = math.radians(angle_end)
            i_start = int((a_start - msg.angle_min) / msg.angle_increment) 
            i_end = int((a_end - msg.angle_min) / msg.angle_increment)
            i_start = max(0, min(len(ranges) - 1, i_start))
            i_end = max(0, min(len(ranges) - 1, i_end))
            if i_start > i_end:
                i_start, i_end = i_end, i_start 
            sector_slice = ranges[i_start:i_end+1]
            if np.all(np.isnan(sector_slice)):
                return float('inf')
            return float(np.nanmin(sector_slice))
           
        self.front_min = sector(-20, 20)        
        self.left_min = sector(60, 120)
        self.right_min = sector(-120, -60)
        # self.get_logger().info(f"front_min: {self.front_min:.3f}, left_min: {self.left_min:.3f}, right_min: {self.right_min:.3f}")


    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose.position


    def image_callback(self, msg):
        # Đã hạ cánh thì không cần xử lý ảnh nữa cho nhẹ máy
        if self.state == "LANDED":
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids = self.aruco_detector.detect_markers(cv_image)
        
        if ids is not None:
            for i, marker_id in enumerate(np.ravel(ids).astype(int)):
                self.get_logger().info(f"Detected ArUco ID {marker_id}, state={self.state}", throttle_duration_sec=1.0)
                
                if marker_id == 50:
                    marker_points = corners[i][0]
                    marker_center = np.mean(marker_points, axis=0)
                    image_center = np.array([320.0, 240.0])
                    self.marker_center_error = marker_center - image_center

                    if self.state == "SEARCHING":
                        self.get_logger().info("Phát hiện bãi đáp ID 50! Hủy quét map, bắt đầu căn chỉnh hạ cánh.")
                        self.state = "ALIGNING" # Chuyển state để dừng bay zigzag


    def control_loop(self):
        if self.current_pose is None:
            return 
            
        msg = Twist()
        if self.state == "TAKEOFF":
            ez = self.z_hover - self.current_pose.z
            
            if abs(ez) <= 0.05:
                self.get_logger().info(f"Reached stable altitude!")
                self.cmd_vel_pub.publish(Twist())
                self.state = "SEARCHING"
                return

            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = self.clamp(self.kp_z * ez, 0.0, 0.5)
            msg.angular.z = 0.0 
            
            self.cmd_vel_pub.publish(msg)

        elif self.state == "SEARCHING":
            if self.mission == "GO":
                if self.front_min < self.stop_dist:  # Có vật cản
                    self.mission = "AVOID"
                    self.avoid_dir = "TURN_LEFT" if self.left_min > self.right_min else "TURN_RIGHT"
            
            elif self.mission == "AVOID":
                if self.front_min > self.clear_dist:  # Đã thoát
                    self.mission = "GO"
                    self.avoid_dir = None

            if self.mission == "GO":
                msg.linear.x = 0.3
                msg.linear.z = self.hold_altitude_velocity()
                msg.angular.z = 0.0      

                if self.left_min < 0.22:
                    msg.linear.x = 0.1
                    msg.linear.z = self.hold_altitude_velocity()
                    msg.angular.z = -0.3                        
                elif self.right_min < 0.22:
                    msg.linear.x = 0.1
                    msg.linear.z = self.hold_altitude_velocity()
                    msg.angular.z = 0.3

            elif self.mission== "AVOID":
                msg.linear.z = self.hold_altitude_velocity()
                msg.angular.z = 1.0 if self.avoid_dir == "TURN_LEFT" else -1.0
            
            self.cmd_vel_pub.publish(msg)
       
        elif self.state == "ALIGNING":
            if self.marker_center_error is None:
                msg.linear.z = 0.0
                self.cmd_vel_pub.publish(msg)
                return 

            error_u = self.marker_center_error[0] # Lệch trái/phải trong ảnh
            error_v = self.marker_center_error[1] # Lệch trên/dưới trong ảnh

            # Camera nhìn xuống: dùng pixel error để đưa marker về giữa khung hình.
            msg.linear.x = self.clamp(-self.kp_align_px * error_v, -0.2, 0.2)
            msg.linear.y = self.clamp(-self.kp_align_px * error_u, -0.2, 0.2)
            msg.linear.z = 0.0 
            msg.angular.z = 0.0

            self.cmd_vel_pub.publish(msg)

            # Nếu marker gần tâm ảnh, bắt đầu hạ độ cao
            if abs(error_u) < self.center_tolerance_px and abs(error_v) < self.center_tolerance_px:
                self.get_logger().info("Đã căn xong tâm. Bắt đầu hạ độ cao...")
                self.state = "DESCENT"

        elif self.state == "DESCENT":
            if self.marker_center_error is not None:
                error_u = self.marker_center_error[0]
                error_v = self.marker_center_error[1]

                # Nếu còn thấy marker thì bám tâm thật nhẹ trong lúc hạ.
                msg.linear.x = max(-0.12, min(0.12, -self.kp_align_px * error_v))
                msg.linear.y = max(-0.12, min(0.12, -self.kp_align_px * error_u))
      
            # Tốc độ hạ tỉ lệ với độ cao hiện tại: cao thì hạ nhanh hơn, gần đất thì chậm lại.
            msg.linear.z = -self.clamp(self.kp_z * max(self.current_pose.z, 0.0), 0.08, 0.22)
            msg.angular.z = 0.0
            
            self.cmd_vel_pub.publish(msg)

            if self.current_pose.z < 0.18:
                self.get_logger().info("HẠ CÁNH THÀNH CÔNG!")
                self.state = "LANDED"

        elif self.state == "LANDED":
            self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = DroneMazeMissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()