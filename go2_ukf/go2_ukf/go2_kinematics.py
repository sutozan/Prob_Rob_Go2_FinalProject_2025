import numpy as np
from math import sin, cos, pi

# Legged Odometry Code!

class Go2Kinematics:
    def __init__(self):
        # Unitree Go2 Link Lengths (meters) --> from URDF: https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go2_description
        self.l1 = 0.0955  # Hip 
        self.l2 = 0.213   # Thigh
        self.l3 = 0.213   # Calf

        # Body Dimensions (center to hip)
        self.length_half = 0.1934
        self.width_half = 0.0465
       
        # Signs for each leg [x, y] relative to center -> from urdf. This is how the urdf has it define each leg. 
        # think of it like a coordinate system with the the origin at the center of the robot and the robot is facing your left.
        self.leg_signs = [
            (1, -1),  # Front right
            (1, 1),   # Front left
            (-1, -1), # Rear right
            (-1, 1)   # Rear left
        ]
    
    def get_foot_pos_rel_body(self, leg_index, q):
        # Goal of this function is to get the foot position relative to the body. We utilize these values to then 
        l_hip = 0.0955   # hip offset - same as above technically just more context here
        l_thigh = 0.213
        l_calf = 0.213

        # Simplifications for the Go2. We are assuming no hip yaw and that planar motion in x z for each leg.
        # yaw mainly contirbutes to foot orientation - we do not consider this in our model - reduced jacobian complexity.
        q1, q2, q3 = q  # hip roll, hip pitch, knee pitch

        # standard - redefining these because we made too many errors with the jacobians when we dont.
        s1, c1 = sin(q1), cos(q1)
        s2, c2 = sin(q2), cos(q2)
        s23, c23 = sin(q2 + q3), cos(q2 + q3)

        # Note - since our world is very much limited by terrain and the movement of our robot in the context of hte project is very limited
        #  We kept everything purely lateral - refreence report for approximations for all of the equations that are utilized below and how it is derived.
        # leg base offset - places the hip joint relative to the body center.
        sign_x, sign_y = self.leg_signs[leg_index]
        px = sign_x * self.length_half
        py = sign_y * self.width_half
        pz = 0.0

        # forward kinematics in leg frame 
        # we perform small angle approximatin here - reference report for more information as to why
        # forward kinematics in leg frame
        x_leg = l_thigh * s2 + l_calf * s23
        z_leg = -(l_thigh * c2 + l_calf * c23)
        # hip roll rotation -- add
        y_rot =  c1 * l_hip
        z_rot =  s1 * l_hip
        x = px + x_leg
        y = py + y_rot
        z = pz + z_leg + z_rot

        return np.array([x, y, z])


    def get_jacobian(self, leg_index, q):
        l1, l2, l3 = self.l1, self.l2, self.l3
        q1, q2, q3 = q

        # standard - redefining these because we made too many errors with the jacobians when we dont.
        s1, c1 = sin(q1), cos(q1)
        s2, c2 = sin(q2), cos(q2)
        s23, c23 = sin(q2 + q3), cos(q2 + q3)

        # Reference report for these calculations
        j11 = 0.0
        j12 = l2 * c2 + l3 * c23
        j13 = l3 * c23

        j21 = -l1 * s1
        j22 = 0.0
        j23 = 0.0

        j31 = l1 * c1
        j32 = l2 * s2 + l3 * s23
        j33 = l3 * s23

        return np.array([
            [j11, j12, j13],
            [j21, j22, j23],
            [j31, j32, j33]
        ])


    def get_body_velocity(self, all_q, all_dq, gyro_body, foot_forces):

        velocities = []
       
        # Threshold: If force is > 20 Newtons, we assume foot is planted.
        # This is purely done to filter through the foot contact topic. There is actually no force values since its all boolean - just a choice we made as we found it
        # easy to understand and also robust to errors that happens in the contact boolean (property of sim)
        # Main assumption is that when the foot is in contact we assume that the velocity in the world frame is 0.
        contact_threshold = 20.0
       
        for i in range(4):
            # contact filter: Only use leg if it is touching the ground
            if foot_forces[i] < contact_threshold:
                continue
            
            # All of the velocity equations can be found in this paper: https://www.ri.cmu.edu/app/uploads/2022/07/Online_Kinematic_Calibration_for_Legged_Robots.pdf
           
            q = all_q[3*i : 3*i+3]
            dq = all_dq[3*i : 3*i+3] #joint velocities - we get these from the topic within odom node
           
            # Linear velocity of the foot relative to the body
            J = self.get_jacobian(i, q)
            v_leg_rel = J @ dq # tells us the relative speed of the foot to the hip
           
            # Tangential Velocity
            p_leg_rel = self.get_foot_pos_rel_body(i, q) # we also get the position of the foot
            tangential_velocity = np.cross(gyro_body, p_leg_rel) # to add more robustness we consider how much
            # the foots velocity is caused purely by robot turning or rotating. Extra element to add more consideration.
           
            # Body Velocity contribution from this leg - 0 = vbody + vrel + vtang - so we solve for vbody
            # 0 because of foot assumption.
            v_body_s = -v_leg_rel - tangential_velocity
            velocities.append(v_body_s) 

        # Since we have 4 legs we need a final velocity - we opted to just take the mean of all the legs - valid enough
        if len(velocities) > 0:
            return np.mean(velocities, axis=0)
        else:
            return np.zeros(3)


