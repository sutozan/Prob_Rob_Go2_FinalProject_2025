import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math


class EvaluatorNode(Node):
    def __init__(self):
        super().__init__('evaluator_node')

        # Subscriptions
        self.truth_sub = self.create_subscription(Odometry, '/odom/ground_truth', self.truth_cb, 10)
        self.ukf_sub = self.create_subscription(Odometry, '/go2/ukf_odom', self.ukf_cb, 10)

        # rqt_plot friendly publishers
        self.dx_pub = self.create_publisher(Float64, '/go2/eval/dx', 10)
        self.dy_pub = self.create_publisher(Float64, '/go2/eval/dy', 10)
        self.pos_err_pub = self.create_publisher(Float64, '/go2/eval/pos_error', 10)
        self.vel_err_pub = self.create_publisher(Float64, '/go2/eval/vel_error', 10)
        self.yaw_err_pub = self.create_publisher(Float64, '/go2/eval/yaw_error', 10)

        # Alignment - so in our initial studies we realized that the sim and our claculation are off by 90 degrees. so we have our UKF output be translated based on the
        # coordinate frame orientation of the ground truth. To be able to access this in RVIZ as well, we made it a publisher but for noting reasons it is mainly used for
        # just aligning the coordinate frames and start point.
        self.aligned_pub = self.create_publisher(Odometry, '/go2/eval/aligned_truth', 10)

        # Need to initalize for the first ttime the message comes through
        self.ukf_msg = None

        # GT alignment
        self.gt_initialized = False
        self.gt_x0 = 0.0
        self.gt_y0 = 0.0
        self.gt_yaw0 = 0.0

    def ukf_cb(self, msg):
        self.ukf_msg = msg

    def quat_yaw(self, q): # Keep it here just in case - we need it for the alignment and use it quite often.
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
        return math.atan2(siny_cosp, cosy_cosp)

    def truth_cb(self, msg):
        if self.ukf_msg is None:
            return

        if not self.gt_initialized:
            # This is where we get the aligned values. so the position and orientation and we'll adjust our coordinate frame via the yaw
            self.gt_x0 = msg.pose.pose.position.x
            self.gt_y0 = msg.pose.pose.position.y
            self.gt_yaw0 = self.quat_yaw(msg.pose.pose.orientation)
            self.gt_initialized = True

            self.get_logger().info(
                f"Alignment: x0={self.gt_x0:.3f}, y0={self.gt_y0:.3f}, yaw0={self.gt_yaw0:.3f}"
            )
            return

        # Align GT into Odom
        # We figured this out through trial and error. 
        x_w = msg.pose.pose.position.x
        y_w = msg.pose.pose.position.y
        yaw_w = self.quat_yaw(msg.pose.pose.orientation)

        dx = x_w - self.gt_x0
        dy = y_w - self.gt_y0

        # Now do typical 2D rotation based on the alignment of the ground truth
        c = math.cos(self.gt_yaw0)
        s = math.sin(self.gt_yaw0)
        gt_x =  c * dx + s * dy
        gt_y = -s * dx + c * dy
        gt_yaw = yaw_w - self.gt_yaw0

        # Publish aligned GT for RViz
        aligned = Odometry()
        aligned.header.stamp = self.get_clock().now().to_msg()
        aligned.header.frame_id = "odom"
        aligned.pose.pose.position.x = gt_x
        aligned.pose.pose.position.y = gt_y
        self.aligned_pub.publish(aligned)

        # Get UKF values
        ukf_x = self.ukf_msg.pose.pose.position.x
        ukf_y = self.ukf_msg.pose.pose.position.y
        ukf_yaw = self.quat_yaw(self.ukf_msg.pose.pose.orientation)

        # Errors
        dx_err = gt_x - ukf_x
        dy_err = gt_y - ukf_y
        position_err = math.sqrt(dx_err**2 + dy_err**2)

        # Sqrt(vx^2+vy^2+vz^2 (vz is 0 so we ommit))
        gt_vel = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        )
        ukf_vel = math.hypot(
            self.ukf_msg.twist.twist.linear.x,
            self.ukf_msg.twist.twist.linear.y
        )
        vel_err = gt_vel - ukf_vel
        yaw_err = gt_yaw - ukf_yaw

        # Publish for rqt_plot 
        self.dx_pub.publish(Float64(data=dx_err))
        self.dy_pub.publish(Float64(data=dy_err))
        self.pos_err_pub.publish(Float64(data=position_err))
        self.vel_err_pub.publish(Float64(data=vel_err))
        self.yaw_err_pub.publish(Float64(data=yaw_err))

        # Log in terminal
        self.get_logger().info(
            f"GT: ({gt_x:.2f}, {gt_y:.2f}) | "
            f"UKF: ({ukf_x:.2f}, {ukf_y:.2f}) | "
            f"dx={dx_err:.2f} dy={dy_err:.2f} | "
            f"yaw_err={yaw_err:.2f} | "
            f"pos_err={position_err:.2f} | "
            f"vel_err={vel_err:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = EvaluatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


