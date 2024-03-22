from filterpy.kalman import KalmanFilter
import numpy as np
from scipy.spatial.transform import Rotation as R

class Filter():
    def __init__(self) -> None:
        dt = 0.01 #assuming mocap runs at 100 hz
        self.my_filter = KalmanFilter(dim_x=6, dim_z=6)
        self.my_filter.x = np.array([0, 0, 0, 0, 0, 0, 0])
        self.my_filter.F = np.array([[1, 1, 1, 1, 1, 1, dt, dt, dt, dt, dt, dt],
                        [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1]])    # state transition matrix

        self.my_filter.H = np.zeros((6, 6))
        self.my_filter.H[0, 0] = 1
        self.my_filter.H[1, 1] = 1
        self.my_filter.H[2, 2] = 1
        self.my_filter.P = np.identity(6)
        self.my_filter.R = np.identity(6)*0.0001 
        self.my_filter.Q = np.identity(6)*0.1

    def update_measurement(self, meas):
        position = meas[:3]
        attitude = self.to_euler(meas[3:7])
        measurement = np.concatenate((position, attitude), axis=None)
        self.my_filter.predict()
        self.my_filter.update(measurement)

    def estimate(self):
        return self.my_filter.x
    
    def to_euler(self, quat):
        return np.array(
            R.from_quat(quat).as_euler("xyz", degrees=False)
        )[0, 2]
