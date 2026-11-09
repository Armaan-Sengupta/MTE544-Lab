import numpy as np

# TODO Part 3: Comment the code explaining each part
class kalman_filter:
    
    # TODO Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R,x, dt:float):
        
        self.P = P.copy()
        self.Q = Q.copy()
        self.R = R.copy()
        self.x = x.copy()
        self.dt = dt
        
    # TODO Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):

        self.A = self.jacobian_A() # Linearzed motion model 

        #Step 1 of EKF: find mu_hat_k
        self.x = self.motion_model() # Update the state using the motion model 

        # Measurment model uses most updated predicted state, C must be linearized around this
        self.C = self.jacobian_H() # Linearzed measurment model 

        #Step 2 of EKF: find sigma_hat_k
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q

    # TODO Part 3: Calculate the Kalman gain and correct your estimates
    def update(self, z):

        # Step 3 of EKF: find the Kalman gain
        kalman_gain = self.P @ self.C.T @ np.linalg.inv((self.C @ self.P @ self.C.T + self.R))

        # Step 4 of EKF: update the predicted state with new measurments
        #                                         mu_hat_k
        #                                           ^^
        self.x = self.x + kalman_gain @ (z - self.measurement_model())

        # Step 5 of EKF: update the predicted covariance with new measurments
        self.P= (np.identity() - kalman_gain @ self.C) @ self.P
        
    
    # TODO Part 3: Implement here the measurement model
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,     # v: predicted linear velocity
            w,     # w: predicted angular velocity
            vdot,  # ax 
            v * w, # ay
        ])
        
    # TODO Part 3: Impelment the motion model (state-transition matrice)
    def motion_model(self):
        """Does not modify self"""
        x, y, th, w, v, vdot = self.x
        dt = self.dt

        # Here we use the non linear model
        return np.array([
            x + v * np.cos(th) * dt,
            y + v * np.sin(th) * dt,
            th + w * dt,
            w,
            v  + vdot*dt,
            vdot,
        ])
        


    
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt

        # Take partial derivatives of motion model WRT current state
        
        return np.array([
            #x, y,               th,     w,             v, vdot
            [1, 0, -v * np.sin(th) * dt, 0, np.cos(th) * dt, 0],
            [0, 1,  v * np.cos(th) * dt, 0, np.sin(th) * dt, 0],
            [0, 0,                1,    dt,            0,    0],
            [0, 0,                0,    1,             0,    0],
            [0, 0,                0,    0,             1,    dt],
            [0, 0,                0,    0,             0,    1 ]
        ])
    
    
    # TODO Part 3: Implement here the jacobian of the H matrix (measurements)    
    def jacobian_H(self):
        x, y, th, w, v, vdot=self.x
        return np.array([
            #x, y,th, w, v,vdot
            [0,0,0  , 0, 1, 0], # v
            [0,0,0  , 1, 0, 0], # w
            [0,0,0  , 0, 0, 1], # ax
            [0,0,0  , v, w, 0], # ay = v * w
        ])
        
    # TODO Part 3: return the states here    
    def get_states(self):
        return self.x
