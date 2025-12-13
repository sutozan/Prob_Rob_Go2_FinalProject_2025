import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from champ_msgs.msg import ContactsStamped
from geometry_msgs.msg import TransformStamped

import message_filters
import numpy as np
from math import sin, cos
import math

from tf2_ros import TransformBroadcaster

from go2_ukf.go2_kinematics import Go2Kinematics
from go2_ukf.ukf_filter import UKF


class Go2OdomNode(Node):
    def __init__(self):
        super().__init__('go2_odom_node')

        self.kinematics = Go2Kinematics()
        self.ukf = UKF(dt=0.02)

        self.latest_contacts = [True, True, True, True]
        self.initialized_pose = False
        # IMU + joint sync
        imu_sub = message_filters.Subscriber(self, Imu, '/imu/data')
        joint_sub = message_filters.Subscriber(self, JointState, '/joint_states')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [imu_sub, joint_sub],
            queue_size=10,
            slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)

        # Foot contacts
        self.create_subscription(
            ContactsStamped,
            '/foot_contacts',
            self.contacts_callback,
            10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/go2/ukf_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = self.get_clock().now()
        self.initialized_orientation = False

        self.get_logger().info("Go2 UKF Odometry Started with TF broadcasting")

    # ----------------------------------------------------------

    def contacts_callback(self, msg):
        if len(msg.contacts) >= 4:
            self.latest_contacts = list(msg.contacts[:4])

    # ----------------------------------------------------------

    def euler_to_quat(self, roll, pitch, yaw):
        qx = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2)
        qy = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2)
        qz = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2)
        qw = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2)
        return qx, qy, qz, qw

    # ----------------------------------------------------------

    def sync_callback(self, imu_msg, joint_msg):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 0.2:
            dt = 0.02

        # ---------- INITIAL ORIENTATION ----------
        if not self.initialized_orientation:
            ax = imu_msg.linear_acceleration.x
            ay = imu_msg.linear_acceleration.y
            az = imu_msg.linear_acceleration.z

            roll = np.arctan2(ay, az)
            pitch = np.arctan2(-ax, np.sqrt(ay*ay + az*az))

            self.ukf.mu[6] = roll
            self.ukf.mu[7] = pitch
            self.ukf.mu[8] = 0.0

            self.initialized_orientation = True
            self.get_logger().info(
                f"Initialized orientation R={roll:.3f}, P={pitch:.3f}"
            )
            return
        
        if not self.initialized_pose:
            self.ukf.mu[0:3] = 0.0
            self.ukf.mu[3:6] = 0.0 # Set initial velocities to zero
            self.ukf.mu[9:] = 0.0 # Initialize biases to zero
            
            self.initialized_pose = True
            self.get_logger().info("Initialized starting pose and biases.")
            return

        # ---------- JOINT SORTING ----------
        target_order = [
            'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint',
            'lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint',
            'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint',
            'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint'
        ]

        q = np.zeros(12)
        dq = np.zeros(12)

        try:
            for i, name in enumerate(target_order):
                idx = joint_msg.name.index(name)
                q[i] = joint_msg.position[idx]
                dq[i] = joint_msg.velocity[idx]
        except ValueError:
            return

        gyro = np.array([
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z
        ])

        foot_forces = [30.0 if c else 0.0 for c in self.latest_contacts]

        v_body = self.kinematics.get_body_velocity(
            q.tolist(),
            dq.tolist(),
            gyro.tolist(),
            foot_forces
        )

        # Fix sign convention
        v_body[0] = -v_body[0]
        v_body[1] = -v_body[1]
        v_body[2] = 0.0

        feet_count = sum(self.latest_contacts)

        if feet_count == 4:
            # Stationary clamp
            self.ukf.mu[3:6] = 0.0
        else:
            scale = 1.0
            v_body[0] *= scale
            v_body[1] *= scale

            u = np.zeros(9)
            u[3:6] = gyro
            u[6:9] = np.zeros(3)

            self.ukf.predict(dt, u)

            yaw = self.ukf.mu[8]

            c = math.cos(yaw)
            s = math.sin(yaw)

            v_world_x = c * v_body[0] - s * v_body[1]
            v_world_y = s * v_body[0] + c * v_body[1]

            v_world = np.array([v_world_x, v_world_y, 0.0])

            self.ukf.update(v_world)
            self.ukf.mu[3:6] = v_world

        self.publish_odom(now)
        self.publish_tf(now)


    def publish_tf(self, timestamp):
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"

        t.transform.translation.x = float(self.ukf.mu[0])
        t.transform.translation.y = float(self.ukf.mu[1])
        t.transform.translation.z = float(self.ukf.mu[2])

        qx, qy, qz, qw = self.euler_to_quat(
            self.ukf.mu[6],
            self.ukf.mu[7],
            self.ukf.mu[8]
        )

        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(t)

    # ----------------------------------------------------------

    def publish_odom(self, timestamp):
        msg = Odometry()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = float(self.ukf.mu[0])
        msg.pose.pose.position.y = float(self.ukf.mu[1])
        msg.pose.pose.position.z = float(self.ukf.mu[2])

        msg.twist.twist.linear.x = float(self.ukf.mu[3])
        msg.twist.twist.linear.y = float(self.ukf.mu[4])
        msg.twist.twist.linear.z = float(self.ukf.mu[5])

        qx, qy, qz, qw = self.euler_to_quat(
            self.ukf.mu[6],
            self.ukf.mu[7],
            self.ukf.mu[8]
        )

        msg.pose.pose.orientation.x = float(qx)
        msg.pose.pose.orientation.y = float(qy)
        msg.pose.pose.orientation.z = float(qz)
        msg.pose.pose.orientation.w = float(qw)

        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Go2OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
