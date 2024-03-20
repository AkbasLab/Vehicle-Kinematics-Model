from math import cos, sin, tan, atan2

import scenarioxp as sxp
import utils

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors

def normalise_angle( angle ):
    """
    :param angle:       (float) angle [rad]
    :return angle:      (float) angle [rad]
    """
    return atan2(sin(angle), cos(angle))

def n_intervals(a : float, b : float , n : int) -> list[float]:
    """
    Provides values for @n intervals between @a and @b
    """
    assert n >= 2
    assert a != b
    nums = [i for i in range(n)]
    return [sxp.project(a, b, num/(n-1)) for num in nums]

class KinematicBicycleModel:
    """
    Summary
    -------
    This class implements the 2D Kinematic Bicycle Model for vehicle dynamics
    Model by https://github.com/winstxnhdw/KinematicBicycleModel
    
    Attributes
    ----------
    dt (float) : discrete time period [s]
    wheelbase (float) : vehicle's wheelbase [m]
    max_steer (float) : vehicle's steering limits [rad]

    Methods
    -------
    __init__(wheelbase: float, max_steer: float, delta_time: float=0.05)
        initialises the class

    update(x, y, yaw, velocity, acceleration, steering_angle)
        updates the vehicle's state using the kinematic bicycle model

    """
    def __init__(self, wheelbase: float, max_steer: float, delta_time: float=0.05):

        self.delta_time = delta_time
        self.wheelbase = wheelbase
        self.max_steer = max_steer


    def update(self, x: float, y: float, yaw: float, velocity: float, acceleration: float, steering_angle: float) -> tuple[float, ...]:
        """
        Summary
        -------
        Updates the vehicle's state using the kinematic bicycle model

        Parameters
        ----------
        x (int) : vehicle's x-coordinate [m]
        y (int) : vehicle's y-coordinate [m]
        yaw (int) : vehicle's heading [rad]
        velocity (int) : vehicle's velocity in the x-axis [m/s]
        acceleration (int) : vehicle's accleration [m/s^2]
        steering_angle (int) : vehicle's steering angle [rad]

        Returns
        -------
        new_x (int) : vehicle's x-coordinate [m]
        new_y (int) : vehicle's y-coordinate [m]
        new_yaw (int) : vehicle's heading [rad]
        new_velocity (int) : vehicle's velocity in the x-axis [m/s]
        steering_angle (int) : vehicle's steering angle [rad]
        angular_velocity (int) : vehicle's angular velocity [rad/s]
        """
        # Compute the local velocity in the x-axis
        new_velocity = velocity + self.delta_time * acceleration

        # Limit steering angle to physical vehicle limits
        steering_angle = -self.max_steer if steering_angle < -self.max_steer else self.max_steer if steering_angle > self.max_steer else steering_angle

        # Compute the angular velocity
        angular_velocity = new_velocity*tan(steering_angle) / self.wheelbase

        # Compute the final state using the discrete time model
        new_x   = x + velocity*cos(yaw)*self.delta_time
        new_y   = y + velocity*sin(yaw)*self.delta_time
        new_yaw = normalise_angle(yaw + angular_velocity*self.delta_time)
        
        return new_x, new_y, new_yaw, new_velocity, steering_angle, angular_velocity
    
class KinematicBicycleModelScenario(sxp.Scenario):
    def __init__(self, params : pd.Series):
        super().__init__(params)

        wheelbase = params["wheelbase.m"]
        max_steer = params["max_steer.rad"]
        time_window = params["time_window.s"]
        delta_time = params["delta_time.s"]

        x0 = params["x.m"]
        y0 = params["y.m"]
        yaw0 = params["yaw.rad"]
        v0 = params["v.mps"]
        a_min = params["a_min.mps^2"]
        a_max = params["a_max.mps^2"]

        kbm = KinematicBicycleModel(wheelbase, max_steer, delta_time)

        
        # Make a manager for the accel and steering angle
        # params_df = pd.DataFrame({
        #     "feat" : ["accel", "steering_angle"],
        #     "min" : [a_min,-max_steer],
        #     "max" : [a_max, max_steer],
        #     "inc" : [0.27, utils.deg2rad(1)]
        # })
        # manager = sxp.ScenarioManager(params_df)
        

        # Get intervals
        n = 5
        a_intervals = n_intervals(a_min, a_max, n)
        n = 10
        steering_angle_intervals = n_intervals(0, max_steer, n)

        # Get initial datapoints.
        init_data = []
        for a in a_intervals:
            for steering_angle in steering_angle_intervals:
                s = pd.Series({
                    "x" : x0,
                    "y" : y0,
                    "yaw" : yaw0,
                    "v" : v0, 
                    "a" : a,
                    "steering_angle" : steering_angle,
                    "angular_velocity" : 0
                })
                init_data.append(s)
                continue
            continue
        init_df = pd.DataFrame(init_data)
        
        # Get trajectories
        all_traj : list[pd.DataFrame] = []
        for i_traj, traj_s in enumerate(init_data):
            #  Get initial params
            x, y, yaw, v, a, \
                steering_angle, angular_velocity = traj_s.to_list()

            #  Record inital params
            traj_history = [traj_s]

            # Run through the time window
            time = delta_time
            invert_steering_angle = True
            while time <= time_window:
                time += delta_time

                x, y, yaw, v, steering_angle, angular_velocity = \
                    kbm.update(x,y, yaw, v, a, steering_angle)
                if v < 0:
                    v = 0
                s = pd.Series({
                    "x" : x,
                    "y" : y,
                    "yaw" : yaw,
                    "v" : v, 
                    "a" : a,
                    "steering_angle" : steering_angle,
                    "angular_velocity" : angular_velocity,
                    "t" : time
                })
                traj_history.append(s)

            df = pd.DataFrame(traj_history)
            df["traj_id"] = i_traj
            all_traj.append(df)
            continue
        self._traj_history : list[pd.DataFrame] = all_traj
        

        print(self.traj_history)

        # Arbitrary scrore.
        self._score = pd.Series({"driveable_area" : 10})

        # self.plot_traj()
        return

    @property
    def traj_history(self) -> pd.DataFrame:
        return pd.concat(self._traj_history)\
            .reset_index(drop=True)
    
    @property
    def score(self) -> pd.Series:
        return self._score

    def plot_traj(self):

        fig = plt.figure(figsize=(5,5))
        ax = fig.gca()
        for df in self._traj_history:
            ax.plot(
                df["y"],
                df["x"],        
            )
        
        plt.savefig("out/rawr.png")
        return