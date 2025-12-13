import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math


class EvaluatorNode(Node):
    def __init__(self):
        super().__init__('evaluator_node')

        # Subscriptions
        self.truth_sub = self.create_subscription(
            Odometry, '/odom/ground_truth', self.truth_callback, 10
        )
        self.ukf_sub = self.create_subscription(
            Odometry, '/go2/ukf_odom', self.ukf_callback, 10
        )

        # rqt_plot friendly publishers
        self.dx_pub = self.create_publisher(Float64, '/go2/eval/dx', 10)
        self.dy_pub = self.create_publisher(Float64, '/go2/eval/dy', 10)
        self.pos_err_pub = self.create_publisher(Float64, '/go2/eval/pos_error', 10)
        self.vel_err_pub = self.create_publisher(Float64, '/go2/eval/vel_error', 10)
        self.yaw_err_pub = self.create_publisher(Float64, '/go2/eval/yaw_error', 10)

        # RViz debug
        self.aligned_pub = self.create_publisher(
            Odometry, '/go2/eval/aligned_truth', 10
        )

        self.current_ukf_msg = None

        # GT alignment
        self.gt_initialized = False
        self.gt_x0 = 0.0
        self.gt_y0 = 0.0
        self.gt_yaw0 = 0.0

    def ukf_callback(self, msg):
        self.current_ukf_msg = msg

    def get_yaw_from_quat(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def truth_callback(self, msg):
        if self.current_ukf_msg is None:
            return

        # --- INITIAL ALIGNMENT (world -> odom) ---
        if not self.gt_initialized:
            self.gt_x0 = msg.pose.pose.position.x
            self.gt_y0 = msg.pose.pose.position.y
            self.gt_yaw0 = self.get_yaw_from_quat(msg.pose.pose.orientation)
            self.gt_initialized = True

            self.get_logger().info(
                f"ALIGNMENT LOCKED: x0={self.gt_x0:.3f}, y0={self.gt_y0:.3f}, yaw0={self.gt_yaw0:.3f}"
            )
            return

        # --- ALIGN GT INTO ODOM ---
        x_w = msg.pose.pose.position.x
        y_w = msg.pose.pose.position.y
        yaw_w = self.get_yaw_from_quat(msg.pose.pose.orientation)

        dx = x_w - self.gt_x0
        dy = y_w - self.gt_y0

        c = math.cos(self.gt_yaw0)
        s = math.sin(self.gt_yaw0)

        gt_x =  c * dx + s * dy
        gt_y = -s * dx + c * dy
        gt_yaw = yaw_w - self.gt_yaw0

        # --- Publish aligned GT for RViz ---
        aligned = Odometry()
        aligned.header.stamp = self.get_clock().now().to_msg()
        aligned.header.frame_id = "odom"
        aligned.pose.pose.position.x = gt_x
        aligned.pose.pose.position.y = gt_y
        self.aligned_pub.publish(aligned)

        # --- UKF values ---
        ukf_x = self.current_ukf_msg.pose.pose.position.x
        ukf_y = self.current_ukf_msg.pose.pose.position.y
        ukf_yaw = self.get_yaw_from_quat(self.current_ukf_msg.pose.pose.orientation)

        # --- Errors ---
        dx_err = gt_x - abs(ukf_x)
        dy_err = gt_y - abs(ukf_y)
        pos_err = math.sqrt(dx_err**2 + dy_err**2)

        gt_vel = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        )
        ukf_vel = math.hypot(
            self.current_ukf_msg.twist.twist.linear.x,
            self.current_ukf_msg.twist.twist.linear.y
        )
        vel_err = gt_vel - ukf_vel
        yaw_err = gt_yaw - ukf_yaw

        # --- Publish scalars for rqt_plot ---
        self.dx_pub.publish(Float64(data=dx_err))
        self.dy_pub.publish(Float64(data=dy_err))
        self.pos_err_pub.publish(Float64(data=pos_err))
        self.vel_err_pub.publish(Float64(data=vel_err))
        self.yaw_err_pub.publish(Float64(data=yaw_err))

        # --- Console logging ---
        self.get_logger().info(
            f"GT: ({gt_x:.2f}, {gt_y:.2f}) | "
            f"UKF: ({ukf_x:.2f}, {ukf_y:.2f}) | "
            f"dx={dx_err:.2f} dy={dy_err:.2f} | "
            f"yaw_err={yaw_err:.2f} | "
            f"pos_err={pos_err:.2f} | "
            f"vel_err={vel_err:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = EvaluatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


