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

#Import our other codes - this is a styalistic choice so that everything isnt super long.
from go2_ukf.go2_kinematics import Go2Kinematics
from go2_ukf.ukf_filter import UKF


class Go2OdomNode(Node):
    def __init__(self):
        super().__init__('go2_odom_node')
        # call in our kinematics and ukf code.
        self.kinematics = Go2Kinematics()
        self.ukf = UKF(dt=0.02)

        # Initalizatoin - we first assume that the robot is fully standing and that the intial pose is false.
        # assume feet are down initially so the robot does not drift immediately.
        self.latest_contacts = [True, True, True, True]
        # Initalizing pose and orientation is a method we came up with to ensure that our intiial contact stance is well detected.
        # Our intial runs showed that although the topic is reliable there are times it will send the wrong message (reasons
        # unknown as it is related to how the topic is formulated). We therefore come up with an intial "filter".
        self.initialized_pose = False
        self.initialized_orientation = False 

        # IMU + joint sync - from Lab 4 we know we need to sync these. hence we first bring in the messagse.
        imu_sub = message_filters.Subscriber(self, Imu, '/imu/data')
        joint_sub = message_filters.Subscriber(self, JointState, '/joint_states')

        # https://docs.ros.org/en/kilted/p/message_filters/doc/Tutorials/Approximate-Synchronizer-Python.html
        # create synchronized callback
        self.t = message_filters.ApproximateTimeSynchronizer([imu_sub, joint_sub],queue_size=10,slop=0.05)
        # sends only when both messages are ready
        self.t.registerCallback(self.sync_cb)

        # Foot contacts subscriptions - this is a built in topic
        self.create_subscription(ContactsStamped,'/foot_contacts',self.contacts_cb,10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/go2/ukf_odom', 10) #publishing to odom topic from UKF
        self.tf_broadcaster = TransformBroadcaster(self) # to transform so we can visluaize in RVIZ
        self.last_time = self.get_clock().now()
        
        self.get_logger().info("Go2 UKF Odometry Started")

    def contacts_cb(self, msg):
        if len(msg.contacts) >= 4: # 4 legs means we always get 4 booleans
            self.latest_contacts = list(msg.contacts[:4]) #update class variable each time topic arrives.

    def sync_cb(self, imu_msg, joint_msg):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now #set time

        if dt <= 0.0 or dt > 0.2:
            dt = 0.02

        # initial orietnation - we need this to align.
        # Continuing from above - we observed that we have to start the UKF at the right tilt. If it starts at a tilt
        # (which it does) the math interprets gravity as acceleration and our position calculation start accumulating even though the robot has not moved yet.
        # these codes each run only once and that is at the beginning
        if not self.initialized_orientation:
            ax = imu_msg.linear_acceleration.x
            ay = imu_msg.linear_acceleration.y
            az = imu_msg.linear_acceleration.z

            # calculate roll and pitch
            roll = np.arctan2(ay, az)
            pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
            # By settig them to the know value we prevent it from thinking the tilt is acceleration
            self.ukf.mu[6] = roll
            self.ukf.mu[7] = pitch
            self.ukf.mu[8] = 0.0 # assume yaw is 0 at start up - so its looking straight

            self.initialized_orientation = True #prevent from starting again
            self.get_logger().info(f"Initialized orientation R={roll:.3f}, P={pitch:.3f}")
            return
        
        if not self.initialized_pose:
            # repeat same process for the pose.
            self.ukf.mu[0:3] = 0.0 #set intiial position to 0,0,0
            self.ukf.mu[3:6] = 0.0 # Set initial velocities to zero
            self.ukf.mu[9:] = 0.0 # Initialize biases to zero
            
            self.initialized_pose = True
            self.get_logger().info("Initialized starting pose and biases.")
            return

        # joint sorting
        # found this through the topics - need to make sure everything is in order and then match them because apparently it is not consistent within the topic.
        # meaning it keeps changing as the message is brought through - weird
        target_order = [
            'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint',
            'lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint',
            'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint',
            'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint'
        ]

        # Now based on the sorting we refill the joint states.
        q = np.zeros(12)
        dq = np.zeros(12)

        # Trial and error - found that we had to also consider for when nothing comes through hence value error
        try:
            for i, name in enumerate(target_order):
                idx = joint_msg.name.index(name) # get teh name
                q[i] = joint_msg.position[idx] # add the joint position to q
                dq[i] = joint_msg.velocity[idx] #add the joint velocity to dq (derivative q)
        except ValueError:
            return

        # Define inputs
        gyro = np.array([
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z
        ])

        accel = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ])

        # Filter through the foot forces using list comprehension - this is making sure if all foots are in contact with the ground or not
        # fuses into measurment
        foot_forces = [30.0 if c else 0.0 for c in self.latest_contacts] #this is used in go2_kinematics

        # Uncomment for Motion Constraint Test!! --> Proof of Concept of the impact of yaw drift.
        """ if feet_count == 4:
            pass
        else:
            
            # 1. Kinematics
            foot_forces = [30.0 if c else 0.0 for c in self.latest_contacts]
            v_body = self.kinematics.get_body_velocity(
                q.tolist(), dq.tolist(), gyro.tolist(), foot_forces
            )
            v_body[1] = -v_body[1]
            v_body[2] = 0.0 

            # 2. Predict
            u = np.zeros(9)
            u[3:6] = gyro 
            self.ukf.predict(dt, u)

            # 3. Update
            v_body_meas = np.array([v_body[0], v_body[1], v_body[2]])
            self.ukf.update(v_body_meas)

            self.publish_odom(now)
            self.publish_tf(now) """


        #function expects list - we get the vbody velocities from our go2_kinematics
        v_body = self.kinematics.get_body_velocity(
            q.tolist(),
            dq.tolist(),
            gyro.tolist(),
            foot_forces
        )
        # Fix sign convention
        #v_body[0] = -v_body[0]
        # Apply necessary sign flips to raw kinematics velocity (v_body is in Body Frame)
        v_body[1] = -v_body[1]
        v_body[2] = 0.0 
        u = np.zeros(9)
        u[3:6] = gyro
        u[6:9] = np.zeros(3) 

        # Trigger prediction based on our control inputs
        self.ukf.predict(dt, u)

        # Fusing foot contact information - this is again another way to filter
        # To further ensure that it can robustly detect velocity vs no velocity.
        feet_count = sum(self.latest_contacts)#this is for our filters
        
        if feet_count == 4:
            #The measurement should be zero in the velocity frame when all the foots are in contact - trigger it
            v_body_meas = np.zeros(3)
        else:
            # Moving: The measurement is the kinematics-derived velocity in the body frame
            v_body_meas = np.array([v_body[0], v_body[1], v_body[2]])

        # Trigger update in the UKF handles which handles the rotation internally
        self.ukf.update(v_body_meas)
        
        # Publish to topic
        self.publish_odom(now)
        self.publish_tf(now) 

    # Now we also have to publish for bth transform and our odometry
    # need this for RVIZ
    def publish_tf(self, timestamp):
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"

        # Get posiiton from UKF
        t.transform.translation.x = float(self.ukf.mu[0])
        t.transform.translation.y = float(self.ukf.mu[1])
        t.transform.translation.z = float(self.ukf.mu[2])

        roll = self.ukf.mu[6]
        pitch = self.ukf.mu[7]
        yaw = self.ukf.mu[8]
        

        # have to convert to quarternions
        qx = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2)
        qy = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2)
        qz = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2)
        qw = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2)

        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(t)


    def publish_odom(self, timestamp):
        msg = Odometry()
        msg.header.stamp = timestamp.to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = "base_link"

        #pose and velocity in world frame
        msg.pose.pose.position.x = float(self.ukf.mu[0])
        msg.pose.pose.position.y = float(self.ukf.mu[1])
        msg.pose.pose.position.z = float(self.ukf.mu[2])

        msg.twist.twist.linear.x = float(self.ukf.mu[3])
        msg.twist.twist.linear.y = float(self.ukf.mu[4])
        msg.twist.twist.linear.z = float(self.ukf.mu[5])

        # now we need to convert to quternions from the euler to be able to publish orientation
        roll, pitch, yaw = self.ukf.mu[6], self.ukf.mu[7], self.ukf.mu[8]
        qx = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2)
        qy = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2)
        qz = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2)
        qw = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2)

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
