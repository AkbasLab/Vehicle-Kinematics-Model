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
            v_max : float,
            lf : float,
            lr : float,
            time_window : float,
            dt : float,
            x0 : float,
            y0 : float,
            v0 : float,
            phi0 : float,
            n_intervals_delta : int = 5,
            delta_samples : list[float] = [],
            width : float = 0
        ):
        """
        Generates vehicle trajectories using a kinematic bicycle driving model
        with wheel sleep.

        :: REQUIRED PARAMETERS ::
            a_min : Min acceleration (mps^2)
            a_max : Max acceleration (mps^2)
            n_intervals_a : # of samples between a_min and a_max
            delta_min : min steering angle of front axle (degrees)
            delta_max : min steering angle of front axle (degrees)
            v_max : Max vehicle speed (mps)
            lf : Distance from front axle to center of mass (m)
            lr : Distance from rear axle to center of mass (m)
            time_window : Simulation Length (s)
            dt : simulation timestep size (s)
            x0 : Initial x position (m)
            y0 : Initial y position (m)
            v0 : Initial speed (mps)
            phi_0 : Initial vehicle heading (degrees).

        :: OPTIONAL PARAMETERS ::
            n_intervals_delta : # of samples between delta_min and delta_max
            delta_samples : Explicit steering angle values values of front axle
                (degrees)
            width : Vehicle width (m)
        """
        self._width = width

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
        if not delta_samples:
            self._delta_samples = utils.n_intervals(
                self.delta_min, 
                self.delta_max, 
                self.n_intervals_delta
            )
        else:
            self._delta_samples = \
                [utils.deg2rad(delta) for delta in delta_samples]
        
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
        self._boundary, self._shape_df = \
            self._find_exterior_boundary(self.traj_summary)
        self._find_all_trajectory_boundaries()
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
    
    @property
    def shape_df(self) -> pd.DataFrame:
        """
        Boundary shape as a pandas Dataframe
        """
        return self._shape_df
    
    @property
    def width(self) -> float:
        """
        Vehicle Width in meters
        """
        return self._width
    
    @property
    def trajectory_polygons(self) -> pd.DataFrame:
        """
        Dataframe of polygons for each tracjectoy.
        """
        return self._trajectory_polygons

    def predict(self, 
            x : float, 
            y : float, 
            v : float,
            a : float,
            phi : float, 
            delta : float,
            lf : float,
            lr : float,
            dt : float
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
        @param dt : Time elapsed

        @return next x, y, phi, and v value.
        """
        beta = np.arctan( (lr/(lf+lr)) * np.tan(delta))
        next_x = x + v * np.cos(phi + beta) * dt
        next_y = y + v * np.sin(phi + beta) * dt
        next_v = v + a * dt
        next_phi = phi + (v/lr) * np.sin(beta) * dt
        return next_x, next_y, next_v, next_phi

    # def 

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

                # Root
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
    
    def _widen_trajectory(self, 
            df : pd.DataFrame, angle : float) -> pd.DataFrame:
        """
        Widens a trajectory to half of the vehicles width

        :: PARAMETERS ::
        df : Trajectory dataframe
        angle : Angle to adjust (in degrees)
            Use 90 for left and -90 for right
        """
        if self.width == 0:
            return df
        df = df.copy()
        angle = utils.deg2rad(angle)
        half_veh_width = self.width/2
        for i in range(len(df.index)):
            s = df.iloc[i]
            x,y = utils.project_point(
                s["x"],
                s["y"],
                half_veh_width,
                angle + s["phi"]
            )
            df.iloc[i]["x"] = np.round(x, decimals=6)
            df.iloc[i]["y"] = np.round(y, decimals=6)
        return df

    def _find_exterior_boundary(self, 
            df : pd.DataFrame
        ) -> list[Polygon, pd.DataFrame]:
        """
        Finds the exterior boundary given a dataframe @df of trajectory
        information.

        :: Return ::
        Returns the boundary as a Shapely Polygon and a Dataframe
        """
        df = df.copy()

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
        ]
        leftmost_traj = self._widen_trajectory(leftmost_traj, 90)[["x", "y"]]
        for i in range(len(leftmost_traj.index)):
            points.append(leftmost_traj.iloc[i])
        

        """
        Next, follow the trajectory with the same delta increasing in accel
        Starting with the closest points from the previous acceleration
        """
        sorted_accel = np.sort(df["a"].unique()).tolist()
        for i, a in enumerate(sorted_accel[1:], start=1):
            traj_df = df[
                (df["a"] == a) &
                (df["delta"] == df["delta"].max())
            ]
            traj_df = self._widen_trajectory(traj_df, 90)[["x", "y"]]
            closest_pos = self._closest_point(points[-1], traj_df)
            traj_df = traj_df[traj_df.index >= closest_pos.name]
            for i in range(len(traj_df.index)):
                points.append(traj_df.iloc[i])
            continue

        """
        Then get the trajectories with the max accelerations sorting from
        highest to lowest angle.
        """
        sorted_delta = np.sort(df["delta"].unique())[::-1].tolist()
        if self.width == 0:
            sorted_delta = sorted_delta[1:-1]
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
        ]
        rightmost_traj = self._widen_trajectory(rightmost_traj, -90)[["x", "y"]]
        for i in range(len(rightmost_traj.index)):
            rightmost_points.append(rightmost_traj.iloc[i])

        """
        Next, follow the trajectory with the same delta increasing in accel
        Starting with the closest points from the previous acceleration
        """
        sorted_accel = np.sort(df["a"].unique()).tolist()
        for i, a in enumerate(sorted_accel[1:], start=1):
            traj_df = df[
                (df["a"] == a) &
                (df["delta"] == df["delta"].min())
            ]
            traj_df = self._widen_trajectory(traj_df, -90)[["x", "y"]]
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
        boundary = Polygon(shape_df.to_numpy())
        # self._shape_df = shape_df
        return boundary, shape_df
    
    def _find_all_trajectory_boundaries(self):
        df = self.traj_summary.copy()
        sorted_delta = np.sort(df["delta"].unique())[::-1].tolist()
        polygons = []
        for delta in sorted_delta:
            delta_df = df[df["delta"] == delta]
            polygon, _ = self._find_exterior_boundary(delta_df)
            s = pd.Series({
                "delta" : delta, 
                "polygon" : polygon
            })
            polygons.append(s)
            continue
        self._trajectory_polygons = pd.DataFrame(polygons)
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
            figsize : tuple[float,float] = (4.5,4.5),
            boundary : bool = False,
        ) -> plt.Figure:
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

        if boundary:
            ax.plot(
                self.shape_df["x"], 
                self.shape_df["y"], 
                marker=".",
                color="blue"
            )

        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")

        plt.savefig("out/traj.pdf",bbox_inches="tight")
        return fig


def test():
    dae = DriveableAreaEstimator(
        a_min = -6,
        a_max = 4,
        n_intervals_a = 3,
        delta_min = -10,
        delta_max = 10,
        n_intervals_delta = 10,
        v_max = 14,
        lf = 2.5,
        lr = 2.5,
        time_window = 3,
        dt = 0.1,
        x0 = 0,
        y0 = 0,
        v0 = 0,
        phi0 = 0,
        delta_samples = [-10,-8,-6,-4,-3,-2,-1,0,1,2,3,4,6,8,10],
        # delta_samples = [0],
        width = 5
    )
    dae.plot_summary(boundary=True)
    return

    
if __name__ == "__main__":
    test()