


import numpy as np




class kalman_filter:
    
    
    def __init__(self, P, Q, R, x, dt: float = 0.1):
        
        self.P = P.copy()
        self.Q = Q.copy()
        self.R = R.copy()
        self.x = x.copy()
        self.dt = dt

        
        
    def predict(self, dt=None):
        
        if dt is not None:
            self.dt = dt

        self.A = self.jacobian_A()
        self.x = self.motion_model()
        self.C = self.jacobian_H()
        
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
    
    def update(self, z):

        kalman_gain = self.P @ self.C.T @ np.linalg.inv((self.C @ self.P @ self.C.T + self.R))

        self.x = self.x + kalman_gain @ (z - self.measurement_model())
        self.P = (np.identity(6) - kalman_gain @ self.C) @ self.P
        
    

    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,
            w,
            vdot,
            v*w
        ])
    
    def motion_model(self):
        
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        return np.array([
            x + v * np.cos(th) * dt,
            y + v * np.sin(th) * dt,
            th + w * dt,
            w,
            v  + vdot*dt,
            vdot,
        ])
        

    def get_states(self):
        return self.x
    
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        return np.array([
            #x, y,               th, w,             v, vdot
            [1, 0, -v*np.sin(th)*dt, 0, np.cos(th)*dt, 0],
            [0, 1,  v*np.cos(th)*dt, 0, np.sin(th)*dt, 0],
            [0, 0,                1, dt,           0,  0],
            [0, 0,                0, 1,            0,  0],
            [0, 0,                0, 0,            1,  dt],
            [0, 0,                0, 0,            0,  1 ]
        ])
    
    
    
    def jacobian_H(self):
        x, y, th, w, v, vdot=self.x
        return np.array([
            #x, y,th, w, v,vdot
            [0,0,0  , 0, 1, 0],
            [0,0,0  , 1, 0, 0],
            [0,0,0  , 0, 0, 1],
            [0,0,0  , v, w, 0],
        ])
        