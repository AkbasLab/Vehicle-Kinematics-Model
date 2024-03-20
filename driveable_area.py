import warnings
warnings.simplefilter('ignore')

import utils
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from shapely.geometry import Polygon


class DriveableAreaEstimator:
    def __init__(self,
            a_min : float,
            a_max : float,
            n_intervals_a : int,
            delta_min : float,
            delta_max : float,
            n_intervals_delta : int,
            v_max : float,
            lf : float,
            lr : float,
            time_window : float,
            dt : float,
            x0 : float,
            y0 : float,
            v0 : float,
            phi0 : float
        ):
        """
        Generates vehicle trajectories using a kinematic bicycle driving model
        with wheel sleep.

        :: PARAMETERS ::
            a_min : Min acceleration (mps^2)
            a_max : Max acceleration (mps^2)
            n_intervals_a : # of samples between a_min and a_max
            delta_min : min steering angle of front axle (degrees)
            delta_max : min steering angle of front axle (degrees)
            n_intervals_delta : # of samples between delta_min and delta_max
            v_max : Max vehicle speed (mps)
            lf : Distance from front axle to center of mass (m)
            lr : Distance from rear axle to center of mass (m)
            time_window : Simulation Length (s)
            dt : simulation timestep size (s)
            x0 : Initial x position (m)
            y0 : Initial y position (m)
            v0 : Initial speed (mps)
            phi_0 : Initial vehicle heading (degrees).
        """

        self._a_min = float(a_min)
        self._a_max = float(a_max)
        self._n_intervals_a = int(float(n_intervals_a))
        self._a_samples = utils.n_intervals(
            self.a_min, 
            self.a_max, 
            self.n_intervals_a
        )

        self._delta_min = utils.deg2rad(float(delta_min))
        self._delta_max = utils.deg2rad(float(delta_max))
        self._n_intervals_delta = int(float(n_intervals_delta))
        self._delta_samples = utils.n_intervals(
            self.delta_min, 
            self.delta_max, 
            self.n_intervals_delta
        )
        
        self._v_max = float(v_max)
        self._lf = float(lf)
        self._lr = float(lr)

        self._time_window = float(time_window)
        self._dt = float(dt)

        self._v0 = float(v0)
        self._x0 = float(x0)
        self._y0 = float(y0)
        self._phi0 = utils.deg2rad(float(phi0))

        self._predict_trajectories()
        self._create_concise_trajectories()
        self._find_exterior_boundary()
        # self._plot_summary()
        return
    
    @property
    def a_min(self) -> float:
        """
        Min acceleration in mps^2
        """
        return self._a_min
    
    @property
    def a_max(self) -> float:
        """
        Max acceleration in mps^2
        """
        return self._a_max
    
    @property
    def n_intervals_a(self) -> int:
        """
        Number of intervals sampled between @a_min and @a_max
        """
        return self._n_intervals_a
    
    @property
    def a_samples(self) -> list[float]:
        """
        List of acceleration (mps^2) samples to be simulated.
        """
        return self._a_samples

    @property
    def delta_min(self) -> float:
        """
        Min steering angle of front axle in radians.
        """
        return self._delta_min
    
    @property
    def delta_max(self) -> float:
        """
        Max stering angle of front axle in radians.
        """
        return self._delta_max
    
    @property
    def n_intervals_delta(self) -> int:
        """
        Number of intervals sampled between @delta_min and @delta_max
        """
        return self._n_intervals_delta

    @property
    def delta_samples(self) -> list[float]:
        """
        List of steering angle (radians) samples to be simulated.
        """
        return self._delta_samples

    @property
    def v_max(self) -> float:
        """
        Upper velocity limit, i.e. max vehicle speed in mps.
        """
        return self._v_max
    
    @property
    def lf(self) -> float:
        """
        Distance from the front axle to center of mass in meters.
        """
        return self._lf
    
    @property
    def lr(self) -> float:
        """
        Distance from the rear axle to center of mass in meters.
        """
        return self._lr
    
    @property
    def time_window(self) -> float:
        """
        Upper limit of simulated time in 0 to @time_window seconds.
        """
        return self._time_window
    
    @property
    def dt(self) -> float:
        """
        Time interval between simulation steps in seconds.
        """
        return self._dt
    
    @property
    def v0(self) -> float:
        """
        Initial velocity/speed in mps.
        """
        return self._v0
    
    @property
    def x0(self) -> float:
        """
        Initial x position in meters.
        """
        return self._x0
    
    @property
    def y0(self) -> float:
        """
        Initial y position in meters
        """
        return self._y0
    
    @property
    def phi0(self) -> float:
        """
        Initial vehicle heading in meters.
        """
        return self._phi0
    
    @property
    def traj_summary(self) -> pd.DataFrame:
        """
        Summary dataframe of all trajectories
        """
        return self._traj_summary
    
    @property
    def traj_summary_concise(self) -> pd.DataFrame:
        """
        A concise summary of all trajectories with 1 row per trajectory.
        """
        return self._traj_summary_concise
    
    @property
    def boundary(self) -> Polygon:
        """
        Boundary of driveable-area as a shapely.geometry.Polygon.
        """
        return self._boundary
    
    @property
    def driveable_area(self) -> float:
        """
        The area of the boudary polygon in m^2
        Area is determined using shoelace formula.
        """
        return self.boundary.area
    
    def predict(self, 
            x : float, 
            y : float, 
            v : float,
            a : float,
            phi : float, 
            delta : float,
            lf : float,
            lr : float,
            d : float
        )-> list[float, float, float, float]:
        """
        @param x : x position (m)
        @param y : y position (m)
        @param v : Speed (mps)
        @param a : acceleration (mps^2)
        @param phi : heading angle (rad)
        @param delta : steering angle of front axle (rad) 
        @param lf : Distance from front axle to center of gravity (m) 
        @param lr : Distance from rear axle to center of gravity (m) 
        @param d : longitudinal distance (m)

        @return next x, y, phi, and v value.
        """
        beta = np.arctan( (lr/(lf+lr)) * np.tan(delta))
        next_x = x + v * np.cos(phi + beta) * d
        next_y = y + v * np.sin(phi + beta) * d
        next_v = v + a * d
        next_phi = phi + (v/lr) * np.sin(beta) * d
        return next_x, next_y, next_v, next_phi
    
    def _predict_trajectories(self):
        """
        Gets vehicle trajectories
        """
        traj_hist : list[pd.Series] = []
        i_traj = 0
        for a in self.a_samples:
            for delta in self.delta_samples:
                x,y,v,phi = self.x0, self.y0, self.v0, self.phi0
                time = self.dt
                while time < self.time_window:
                    time += self.dt
                    x,y,v,phi = self.predict(
                        x,y,v,a,phi,delta,self.lf,self.lr,self.dt)
                    if v < 0:
                        v = 0
                    elif v > self.v_max:
                        v = self.v_max
                    s = pd.Series({
                        "x" : x,
                        "y" : y,
                        "v" : v,
                        "phi" : phi,
                        "a" : a,
                        "delta" : delta,
                        "i_traj" : i_traj
                    })
                    traj_hist.append(s)
                    if v == 0:
                        break
                    continue
                i_traj += 1
                continue
            continue
        
        self._traj_summary = pd.DataFrame(traj_hist)
        return
    
    def _create_concise_trajectories(self):
        df = self.traj_summary.copy()
        data : list[pd.Series] = []
        for i_traj in df["i_traj"].unique():
            traj_df = df[df["i_traj"] == i_traj]
            path = traj_df[["x","y"]].to_numpy().tolist()
            s = pd.Series({
                "a" : traj_df["a"].iloc[0],
                "delta" : traj_df["delta"].iloc[0],
                "path" : path
            })
            data.append(s)
            continue
        self._traj_summary_concise = pd.DataFrame(data)
        return
    
    def _find_exterior_boundary(self):
        df = self.traj_summary

        # The shape begins at the origin.
        origin = pd.Series({"x" : self.x0, "y" : self.y0})
        points = [origin]

        """
        Start with the leftmost trajectory with the small accel and largest
        steering angle.
        """
        leftmost_traj = df[
            (df["a"] == df["a"].min()) &
            (df["delta"] == df["delta"].max())
        ][["x", "y"]]
        for i in range(len(leftmost_traj.index)):
            points.append(leftmost_traj.iloc[i])
        
        """
        Next, follow the trajectory with the same delta increasing in accel
        Starting with the closest points from the previous acceleration
        """
        sorted_accel = np.sort(df["a"].unique()).tolist()
        for i, a in enumerate(sorted_accel[1:], start=1):
            # print("%d accel: %f" % (i,a))
            traj_df = df[
                (df["a"] == a) &
                (df["delta"] == df["delta"].max())
            ][["x", "y"]]
            closest_pos = self._closest_point(points[-1], traj_df)
            traj_df = traj_df[traj_df.index >= closest_pos.name]
            for i in range(len(traj_df.index)):
                points.append(traj_df.iloc[i])
            continue

        """
        Then get the trajectories with the max accelerations sorting from
        highest to lowest angle.
        """
        sorted_delta = np.sort(df["delta"].unique())[::-1].tolist()[1:-1]
        for delta in sorted_delta:
            traj_df = df[
                (df["a"] == df["a"].max()) &
                (df["delta"] == delta)
            ][["x","y"]]
            pos = traj_df[traj_df.index == traj_df.index.max()].iloc[0]
            points.append(pos)
            continue

        """
        Get the rightmost trajectory with the smallest acceleration
        """
        rightmost_points : list[pd.Series] = []
        rightmost_traj = df[
            (df["a"] == df["a"].min()) &
            (df["delta"] == df["delta"].min())
        ][["x", "y"]]
        for i in range(len(rightmost_traj.index)):
            rightmost_points.append(rightmost_traj.iloc[i])

        """
        Next, follow the trajectory with the same delta increasing in accel
        Starting with the closest points from the previous acceleration
        """
        sorted_accel = np.sort(df["a"].unique()).tolist()
        for i, a in enumerate(sorted_accel[1:], start=1):
            # print("%d accel: %f" % (i,a))
            traj_df = df[
                (df["a"] == a) &
                (df["delta"] == df["delta"].min())
            ][["x", "y"]]
            closest_pos = self._closest_point(rightmost_points[-1], traj_df)
            traj_df = traj_df[traj_df.index >= closest_pos.name]
            for i in range(len(traj_df.index)):
                rightmost_points.append(traj_df.iloc[i])
            continue

        """
        Add the points in reverse order
        """
        for i in range(len(rightmost_points)):
            points.append(rightmost_points[-i-1])
        
        
        shape_df = pd.DataFrame(points)
        self._boundary = Polygon(shape_df.to_numpy())
        return
    
    def _closest_point(self, xy : pd.Series, df : pd.DataFrame) -> pd.Series:
        """
        Finds the closest point of @xy to a point in @df.
        """
        assert all([feat in ["x", "y"] for feat in xy.index])
        assert all([feat in ["x", "y"] for feat in df.columns])
        df = df.copy()
        df["dist"] = df.apply(
            lambda s : utils.distance_to(*xy.to_list(), s["x"], s["y"]),
            axis = 1
        )
        return df[df["dist"] == df["dist"].min()].iloc[0][["x", "y"]]
    
    

    
    
    

    def plot_summary(self, 
            figsize : tuple[float,float] = (4.5,4.5)) -> plt.Figure:
        df = self.traj_summary

        # Plot
        plt.clf()
        fig = plt.figure(figsize=figsize)
        ax = fig.gca()
        ax.scatter(
            df["x"], 
            df["y"], 
            marker="+",
            color="black"
        )
        ax.plot(self.x0,self.y0,color="red", marker="+")

        # ax.plot(
        #     shape_df["x"], 
        #     shape_df["y"], 
        #     marker=".",
        #     color="blue"
        # )

        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")

        # plt.savefig("out/compy.png")
        return fig
    
    
# if __name__ == "__main__":
    
#     Drive)