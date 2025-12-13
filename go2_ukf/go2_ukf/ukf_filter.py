import numpy as np
from math import sin, cos




def rpy_to_rotation_matrix(roll, pitch, yaw):
    cr, sr = cos(roll), sin(roll)
    cp, sp = cos(pitch), sin(pitch)
    cy, sy = cos(yaw), sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ])

class UKF:
    def __init__(self, dt):
        self.dt = dt
        self.n = 15
        self.m = 3
       
        # State: [p_x, p_y, p_z, v_x, v_y, v_z, r, p, y, bw_x... ba_x...]
        # NOTE: v_x, v_y, v_z are now in WORLD FRAME
        self.mu = np.zeros(self.n)
        self.Sigma = np.eye(self.n) * 0.1
       
        # Tuned Noise Matrices
        self.R = np.diag([
            1e-2, 1e-2, 1e-2,   # Position (Increased from 1e-4 to 1e-2)
            1.0, 1.0, 1.0,      # World Velocity 
            1e-5, 1e-5, 1e-5,   # Orientation 
            1e-8, 1e-8, 1e-8,   # Gyro Bias 
            1e-8, 1e-8, 1e-8    # Accel Bias
        ])
       
        self.Q = np.diag([
            1e-3, 1e-3, 1e-3 # Very small noise for v_x, v_y, v_z measurement
        ])
        self.GRAVITY = np.array([0.0, 0.0, 9.81]) # Positive gravity for subtraction logic
       
        # UKF Weights (Standard Merwe/Van der Wan)
        self.alpha = 0.1
        self.beta = 2.0
        self.kappa = 0.0
        self.lamda = self.alpha**2 * (self.n + self.kappa) - self.n
        self.wm_0 = self.lamda / (self.n + self.lamda)
        self.wc_0 = self.wm_0 + (1 - self.alpha**2 + self.beta)
        self.w_i = 1.0 / (2 * (self.n + self.lamda))




    def generate_sigma_points(self, mu, Sigma):
        # Added small epsilon for numerical stability
        L = np.linalg.cholesky((self.n + self.lamda) * Sigma + np.eye(self.n)*1e-9)
        X = np.zeros((2 * self.n + 1, self.n))
        X[0] = mu
        for i in range(self.n):
            X[i + 1] = mu + L[:, i]
            X[self.n + i + 1] = mu - L[:, i]
        return X




    def predict(self, dt, u):
        self.dt = dt
       
        u_w = np.array(u[3:6])   # Gyro
        u_a = np.array(u[6:9])   # Accel


        X = self.generate_sigma_points(self.mu, self.Sigma)
        x_pred = np.zeros_like(X)


        for i in range(len(X)):
            p = X[i, 0:3]
            v_world = X[i, 3:6]
            o = X[i, 6:9]
            b_w = X[i, 9:12]
            b_a = X[i, 12:15]


            Rot = rpy_to_rotation_matrix(o[0], o[1], o[2])


            # 1. Bias Correction
            w_corrected = u_w - b_w
            a_corrected = u_a - b_a


            # --- FIX 2: CORRECT ORIENTATION INTEGRATION ---
            # Convert Body Rates (Gyro) to Euler Rates (Roll/Pitch/Yaw rates)
            # This prevents the "Mathematical Drift"
            phi, theta = o[0], o[1]
           
            # Pre-calc trig for readability
            sp, cp = np.sin(phi), np.cos(phi)
            tt = np.tan(theta)
            ct = np.cos(theta) # used for secant (1/cos)


            # Avoid division by zero if pitch is exactly 90 (unlikely for a dog)
            if abs(ct) < 1e-4: ct = 1e-4


            # Conversion Matrix Multiplications written out:
            # roll_rate  = wx + wy*sin(phi)*tan(theta) + wz*cos(phi)*tan(theta)
            # pitch_rate =      wy*cos(phi)            - wz*sin(phi)
            # yaw_rate   =      wy*sin(phi)/cos(theta) + wz*cos(phi)/cos(theta)


            d_roll = w_corrected[0] + w_corrected[1]*sp*tt + w_corrected[2]*cp*tt
            d_pitch = w_corrected[1]*cp - w_corrected[2]*sp
            d_yaw = (w_corrected[1]*sp + w_corrected[2]*cp) / ct


            o_new = np.array([
                o[0] + d_roll * self.dt,
                o[1] + d_pitch * self.dt,
                o[2] + d_yaw * self.dt
            ])
            # ---------------------------------------------


            # 3. Velocity Update (World Frame)
            acc_world = Rot @ a_corrected - self.GRAVITY
            v_new = v_world + acc_world * self.dt


            # 4. Position Update
            p_new = p + v_world * self.dt + 0.5 * acc_world * self.dt**2
           
            x_pred[i] = np.concatenate([p_new, v_new, o_new, b_w, b_a])


        # Recombine Sigma Points
        mu_bar = x_pred[0] * self.wm_0 + np.sum(x_pred[1:] * self.w_i, axis=0)
       
        Sigma_bar = self.wc_0 * np.outer(x_pred[0] - mu_bar, x_pred[0] - mu_bar)
        diff = x_pred[1:] - mu_bar
       
        Sigma_bar += self.w_i * (diff.T @ diff)
        Sigma_bar += self.R
       
        self.mu = mu_bar
        self.Sigma = Sigma_bar
        return mu_bar




    def update(self, z_meas):
        """
        z_meas: [vx, vy, vz] in BODY FRAME (from Kinematics)
        """
        X = self.generate_sigma_points(self.mu, self.Sigma)
        Z_pred = np.zeros((len(X), self.m))


        # Pass Sigma Points through Measurement Function h(x)
        for i in range(len(X)):
            v_world = X[i, 3:6]
            o = X[i, 6:9]
           
            # Create Rotation Matrix (Body -> World)
            Rot = rpy_to_rotation_matrix(o[0], o[1], o[2])
           
            # Measurement Function: Transform World Velocity to Body Velocity
            # v_body = Rot.T * v_world
            Z_pred[i] = Rot.T @ (v_world)

        # Mean predicted measurement
        z_bar = Z_pred[0] * self.wm_0 + np.sum(Z_pred[1:] * self.w_i, axis=0)
       
        # Covariances
        S = self.wc_0 * np.outer(Z_pred[0] - z_bar, Z_pred[0] - z_bar) + self.Q
        Sigma_xz = self.wc_0 * np.outer(X[0] - self.mu, Z_pred[0] - z_bar)
       
        diff_z = Z_pred[1:] - z_bar
        diff_x = X[1:] - self.mu
       
        S += self.w_i * (diff_z.T @ diff_z)
        Sigma_xz += self.w_i * (diff_x.T @ diff_z)




        # Kalman Gain
        K = Sigma_xz @ np.linalg.inv(S)
       
        # Update State
        innovation = z_meas - z_bar
        self.mu = self.mu + K @ innovation
        self.Sigma = self.Sigma - K @ S @ K.T