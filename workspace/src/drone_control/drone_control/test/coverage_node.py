import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math



class DroneCoverageNode(Node):
    def __init__(self):
        super().__init__('drone_coverage_node')
        
        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop) # Chạy ở tần số 20Hz

        # Thông số cấu hình
        self.current_pose = None
        self.waypoints = self.generate_zigzag_waypoints()
        self.current_wp_index = 0
        
        # Hệ số Proportional (P) của bộ điều khiển vị trí
        self.kp_xy = 0.5
        self.kp_z = 0.8
        self.tolerance = 0.25 # Sai số chấp nhận được (mét) để chuyển sang điểm tiếp theo

    def generate_zigzag_waypoints(self):
        waypoints = []
        z_hover = 2.0 # Độ cao quét lý tưởng
        
        # Giới hạn an toàn (cách tường 1 mét để tránh va chạm)
        x_min, x_max = -4.0, 4.0
        y_min, y_max = -4.0, 4.0
        step = 1.5 # Khoảng cách giữa các hàng quét (phụ thuộc vào FOV của camera)
        
        # 1. Điểm xuất phát: Bay thẳng lên không trung
        waypoints.append((0.0, 0.0, z_hover))
        
        # 2. Sinh quỹ đạo Zigzag
        y = y_min
        direction = 1 # 1: quét từ trái sang phải, -1: quét từ phải sang trái
        
        while y <= y_max:
            if direction == 1:
                waypoints.append((x_min, y, z_hover)) # Điểm bắt đầu hàng
                waypoints.append((x_max, y, z_hover)) # Điểm kết thúc hàng
            else:
                waypoints.append((x_max, y, z_hover))
                waypoints.append((x_min, y, z_hover))
            
            direction *= -1 # Đảo chiều cho hàng tiếp theo
            y += step
            
        return waypoints

    def odom_callback(self, msg):
        # Lấy tọa độ thực tế của drone
        self.current_pose = msg.pose.pose.position

    def control_loop(self):
        if self.current_pose is None:
            return # Đợi cho đến khi nhận được dữ liệu odom đầu tiên
        
        # Kiểm tra nếu đã bay hết danh sách waypoints
        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().info('Đã quét xong toàn bộ bản đồ! Chuyển sang Hovering.', throttle_duration_sec=3.0)
            self.cmd_vel_pub.publish(Twist()) # Xuất vận tốc 0 để hover tại chỗ
            return

        target = self.waypoints[self.current_wp_index]
        
        # Tính toán sai số (Error) trên các trục
        ex = target[0] - self.current_pose.x
        ey = target[1] - self.current_pose.y
        ez = target[2] - self.current_pose.z
        
        # Tính khoảng cách Euclidean đến điểm mục tiêu
        distance = math.sqrt(ex**2 + ey**2 + ez**2)
        
        # Nếu đã đến đủ gần điểm mục tiêu, chuyển sang điểm tiếp theo
        if distance < self.tolerance:
            self.get_logger().info(f'Đã đến waypoint {self.current_wp_index}: (X: {target[0]:.1f}, Y: {target[1]:.1f})')
            self.current_wp_index += 1
            self.cmd_vel_pub.publish(Twist()) # Phanh nhẹ
            return

        # Tính toán lệnh điều khiển (P-Controller)
        msg = Twist()
        # Áp dụng giới hạn vận tốc (clamp) để drone bay mượt, không bị lật
        msg.linear.x = max(-1.0, min(1.0, self.kp_xy * ex))
        msg.linear.y = max(-1.0, min(1.0, self.kp_xy * ey))
        msg.linear.z = max(-0.5, min(0.5, self.kp_z * ez))
        
        # Giữ nguyên hướng quay, drone sẽ bay kiểu "trượt ngang/dọc" như một cái máy scan
        msg.angular.z = 0.0 
        
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneCoverageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()