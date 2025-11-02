import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion
import threading
import sys
import termios
import tty

def keyboard_listener(node):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            ch = sys.stdin.read(1)
            if ch == 'w':
                node.t_increment = min(node.t_increment + 0.01, 0.3)
                print(f'Speed increased. t_increment = {node.t_increment:.3f}')
            elif ch == 's':
                node.t_increment = max(node.t_increment - 0.01, 0.01)
                print(f'Speed decreased. t_increment = {node.t_increment:.3f}')
            elif ch == 'r':
                node.t_increment = -abs(node.t_increment)
                print('Direction reversed')
            elif ch == 'f':
                node.t_increment = abs(node.t_increment)
                print('Direction forward')
            elif ch == 'x':
                node.t_increment = 0.0
                print('Motion stopped')
            elif ch == 'q':
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

class FigureEightController(Node):
    
    def __init__(self):
        super().__init__('figure_eight_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('kp_angular', 4.0)
        self.declare_parameter('max_t', 2 * math.pi)
        self.declare_parameter('velocity_linear_max', 1)
        self.declare_parameter('velocity_angular_max', 4.0)
        self.declare_parameter('distance_threshold', 0.1)
        self.declare_parameter('t_increment', 0.05)
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.max_t = self.get_parameter('max_t').value
        self.velocity_linear_max = self.get_parameter('velocity_linear_max').value
        self.velocity_angular_max = self.get_parameter('velocity_angular_max').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.t_increment = self.get_parameter('t_increment').value
        self.t = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.t_increment = 0.05  # default value
        threading.Thread(target=keyboard_listener, args=(self,), daemon=True).start()

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w])
        self.current_yaw = yaw

    def figure_eight_point(self, t):
        a = 1.0
        x = a * math.cos(t) / (1 + math.sin(t)**2)
        y = a * math.cos(t) * math.sin(t) / (1 + math.sin(t)**2)
        return x, y

    def timer_callback(self):
        target_x, target_y = self.figure_eight_point(self.t)
        error_x = target_x - self.current_x
        error_y = target_y - self.current_y
        distance_error = math.sqrt(error_x**2 + error_y**2)
        target_angle = math.atan2(error_y, error_x)

        angle_error = self.normalize_angle(target_angle - self.current_yaw)

        v = self.kp_linear * distance_error
        w = self.kp_angular * angle_error

        v = max(min(v, self.velocity_linear_max), 0)
        w = max(min(w, self.velocity_angular_max), -self.velocity_angular_max)

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_vel_pub.publish(twist)

        if distance_error < self.distance_threshold:
            self.t += self.t_increment
            if self.t > self.max_t:
                self.t -= self.max_t

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = FigureEightController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
