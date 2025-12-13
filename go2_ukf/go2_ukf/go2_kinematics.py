import numpy as np
from math import sin, cos, pi


class Go2Kinematics:
    def __init__(self):
        # Unitree Go2 Link Lengths (meters)
        self.l1 = 0.0955  # Hip Abduction
        self.l2 = 0.213   # Thigh
        self.l3 = 0.213   # Calf


        # Body Dimensions (Center to Hip)
        self.length_half = 0.1934
        self.width_half = 0.0465
       
        # Signs for each leg [x, y] relative to center
        self.leg_signs = [
            (1, -1),  # FR (0)
            (1, 1),   # FL (1)
            (-1, -1), # RR (2)
            (-1, 1)   # RL (3)
        ]


    def get_foot_pos_rel_body(self, leg_index, q):
        l_hip = 0.0955   # hip offset
        l_thigh = 0.213
        l_calf = 0.213

        q1, q2, q3 = q  # hip roll, hip pitch, knee pitch

        s1, c1 = sin(q1), cos(q1)
        s2, c2 = sin(q2), cos(q2)
        s23, c23 = sin(q2 + q3), cos(q2 + q3)

        # leg base offset
        sign_x, sign_y = self.leg_signs[leg_index]
        px = sign_x * self.length_half
        py = sign_y * self.width_half
        pz = 0.0

        # forward kinematics in leg frame
        x_leg = l_thigh * s2 + l_calf * s23
        z_leg = -(l_thigh * c2 + l_calf * c23)

        # hip roll rotation
        y_rot =  c1 * l_hip
        z_rot =  s1 * l_hip

        x = px + x_leg
        y = py + y_rot
        z = pz + z_leg + z_rot

        return np.array([x, y, z])



    def get_analytical_jacobian(self, leg_index, q):
        l1, l2, l3 = self.l1, self.l2, self.l3
        q1, q2, q3 = q

        s1, c1 = sin(q1), cos(q1)
        s2, c2 = sin(q2), cos(q2)
        s23, c23 = sin(q2 + q3), cos(q2 + q3)

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
        CONTACT_THRESHOLD = 20.0
       
        for i in range(4):
            # CONTACT FILTER: Only use leg if it is touching the ground
            if foot_forces[i] < CONTACT_THRESHOLD:
                continue
           
            q = all_q[3*i : 3*i+3]
            dq = all_dq[3*i : 3*i+3]
           
            # 1. Foot Velocity relative to Body
            J = self.get_analytical_jacobian(i, q)
            v_leg_rel = J @ dq
           
            # 2. Tangential Velocity (Angular Compensation)
            p_leg_rel = self.get_foot_pos_rel_body(i, q)
            tangential_vel = np.cross(gyro_body, p_leg_rel)
           
            # 3. Body Velocity contribution from this leg
            v_body_sample = -v_leg_rel - tangential_vel
            velocities.append(v_body_sample)


        # Robustness Check:
        if len(velocities) > 0:
            return np.mean(velocities, axis=0)
        else:
            return np.zeros(3)


