import numpy as np
from math import sin, cos


def rpy_to_rotation_matrix(roll, pitch, yaw): # Equtions found from: https://msl.cs.uiuc.edu/planning/node102.html
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
        self.n = 15 # State: [p_x, p_y, p_z, v_x, v_y, v_z, r, p, y, bw_x... ba_x...] #overkill but we also considered the biases to bring more robusts
        self.m = 3 # v_x, v_y, v_z
       
        # ALL EQUATIONS AND FORMATS FROM TEXTBOOK UKF ALGORITHM PG 59

        self.mu = np.zeros(self.n)
        self.Sigma = np.eye(self.n) * 0.1
       
        # Tuned Noise Matrices
        self.R = np.diag([
            1e-2, 1e-2, 1e-2,   # Position (increased from 1e-4 to 1e-2) --> fine tunning
            1.0, 1.0, 1.0,      # World Velocity 
            1e-5, 1e-5, 1e-5,   # Orientation 
            1e-8, 1e-8, 1e-8,   # Gyro Bias 
            1e-8, 1e-8, 1e-8    # Accel Bias
        ])
       
        self.Q = np.diag([
            1e-2, 1e-2, 1e-2 # Very small noise for v_x, v_y, v_z measurement -> fine tunned
        ])

        self.gravity = np.array([0.0, 0.0, 9.81]) # Positive gravity for subtraction logic - its negative below so positive here
       
        # UKF Weights (standrd from textbook)
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
        # following indexing format from texxtbook - i originally did this differently but this seems more efficient
        # not sure if true - but this change porduced more accurate results.
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

        # now we do prediction for each of our states. each is approached differently.
        for i in range(len(X)):
            p = X[i, 0:3]
            v_world = X[i, 3:6]
            o = X[i, 6:9]
            b_w = X[i, 9:12]
            b_a = X[i, 12:15]

            Rot = rpy_to_rotation_matrix(o[0], o[1], o[2]) #3D rotation via the orientation to go from world frame to body frame

            # Bias Correction
            w_corrected = u_w - b_w
            a_corrected = u_a - b_a

            # Convert Body Rates (gyro - angular velocity) to euler rates (roll/pitch/yaw) - all formulas used below can be referenced to this link
            # https://robotics.stackexchange.com/questions/8463/angular-velocity-to-translational-velocity
            
            phi, theta = o[0], o[1]
           
            # Trig to make it easier to implement equations
            sp, cp = np.sin(phi), np.cos(phi)
            tt = np.tan(theta)
            ct = np.cos(theta) 

            # Avoid division by zero if pitch is exactly 90 (unlikely for a dog)
            if abs(ct) < 1e-4: ct = 1e-4

            # Had to do these by hand - when we used inverse function it would not be as precise - weird - basically the full matrix multiplication of the inverse in 
            # mentioned like utilize to get euler rates.
            d_roll = w_corrected[0] + w_corrected[1]*sp*tt + w_corrected[2]*cp*tt
            d_pitch = w_corrected[1]*cp - w_corrected[2]*sp
            d_yaw = (w_corrected[1]*sp + w_corrected[2]*cp) / ct
            #d_yaw = 0.0 # Testing yaw drift error impact

            o_new = np.array([
                o[0] + d_roll * self.dt,
                o[1] + d_pitch * self.dt,
                o[2] + d_yaw * self.dt
            ])

            # Velocity update in world frame
            # our sensors live in the body frame but world frame math is easier for state updates so we need to rotate from body to world
            acc_world = Rot @ a_corrected - self.gravity
            v_new = v_world + (acc_world * self.dt)

            # Position Update
            p_new = p + v_world * self.dt + 0.5 * acc_world * self.dt**2
            x_pred[i] = np.concatenate([p_new, v_new, o_new, b_w, b_a]) #add everything to new predicted state

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
        X = self.generate_sigma_points(self.mu, self.Sigma)
        Z_pred = np.zeros((len(X), self.m))

        # Pass Sigma Points through measurment function h(x) (the rotation)
        for i in range(len(X)):
            v_world = X[i, 3:6]
            o = X[i, 6:9]
           
            # Create Rotation Matrix (Body -> World)
            Rot = rpy_to_rotation_matrix(o[0], o[1], o[2])
           
            # Measurement Function: world velocity to body velocty - from the paper we realized this was the easiest and least error prone way to do this. Predicting
            # body velocity is veyr difficult for a dog robot due to the multiple limbs moving at different rates. Oversimplified but works
            # v_body = Rot.T * v_world - this is where the measurment nonlinarity comes from.
            Z_pred[i] = Rot.T @ (v_world)

        # Mean predicted measurement
        z_bar = Z_pred[0] * self.wm_0 + np.sum(Z_pred[1:] * self.w_i, axis=0)
       
        # Covariances
        # have to use np.outer due to operation - @ throws error
        S = self.wc_0 * np.outer(Z_pred[0] - z_bar, Z_pred[0] - z_bar) + self.Q
        Sigma_xz = self.wc_0 * np.outer(X[0] - self.mu, Z_pred[0] - z_bar)
       
        # note when I put these inside the matrix multiplication code was more error prone - not 100% sure but seemed like it so we kept it outside
        # Future note : Look into it.
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