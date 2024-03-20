import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import scenarioxp as sxp


class UnitTime:
    def __init__(self, s : float = None):
        """
        Time Units

        :: PARAMETERS ::
        @param s : float
            Seconds
        """
        if s != None:
            self._s = s
        else:
            raise NotImplementedError
        return
    
    @property
    def s(self) -> float:
        return self._s
    
    def __str__(self) -> str:
        return "%.3f s" % self.s

class UnitSpeed:
    def __init__(self, mps : float = None):
        """
        Speed Units
        
        :: PARMAETERS :: 
        @param mps : float
            Meters per second
        """
        if mps != None:
            self._mps = mps
        else:
            raise NotImplementedError
        return
    
    @property
    def mps(self) -> float:
        """
        Meters per Second
        """
        return self._mps
    
    def __str__(self) -> str:
        return "%.3f mps" % self.mps
    
class UnitAcceleration:
    def __init__(self, mpsps : float = None):
        """
        Unit Acceleration

        :: Parameters ::
        @param mpsps : float
            Meters per second ^2
        """
        if mpsps != None:
            self._mpsps = mpsps
        else:
            raise NotImplementedError
        return
    
    @property
    def mpsps(self) -> float:
        """
        Meters per second ^2
        """
        return self._mpsps
    
    @property
    def meters_per_second_squared(self) -> float:
        return self._mpsps
    
    def __str__(self) -> str:
        return "%.3f mps^2" % self.mpsps


class UnitLength:
    def __init__(self, m : float = None):
        """
        Length Units

        :: PARAMETERS ::
        @param m : flaot
            Meters
        """
        if m != None:
            self._m = m
        else:
            raise NotImplementedError
        return
    
    @property
    def m(self) -> float:
        """
        Meter
        """
        return self._m
    
    def __str__(self) -> str:
        return "%.3f m" % self.m
    
class UnitAngle:
    def __init__(self, radians : float = None, degrees : float = None):
        """
        Angle Units

        :: PARAMETERS ::
        @param radians : float
            Radians

        @param degrees : float
            Degrees
        """
        if radians != None:
            self._radians = radians
        elif degrees != None:
            self._radians = degrees * np.pi/180
        else:
            raise NotImplementedError
        return
    
    @property
    def radians(self) -> float:
        return self._radians
    
    @property
    def degrees(self) -> float:
        return self._radians * 180/np.pi
    
    def __str__(self) -> str:
        return "%.3f rad %.3f deg" % (self.radians, self.degrees)


def pred_x(
    x : UnitLength,
    v : UnitSpeed, 
    phi : UnitAngle, 
    beta : UnitAngle, 
    d : UnitLength
) -> UnitLength:
    """
    Predict next x postion

    :: Parameters ::
    @param x : UnitLength
        x position
    @param v : UnitSpeed
        vehicle speed
    @param phi : UnitAngle
        heading angle
    @param beta : UnitAngle
        Slipe angle at venter of mass
    @param d : unitlength
        distance

    :: Return ::
    @return next x position
    """
    next_x = x.m + v.mps * np.cos(phi.radians + beta.radians) * d.m
    return UnitLength(m = next_x)

def pred_y(
    y : UnitLength, 
    v : UnitSpeed, 
    phi : UnitAngle, 
    beta : UnitAngle, 
    d : UnitLength
) -> UnitLength:
    """
    Predict next y position

    :: Parameters ::
    @param y : UnitLength
        y position
    @param v : UnitSpeed
        vehicle speed
    @param phi : UnitAngle
        heading angle
    @param beta : UnitAngle
        Slipe angle at venter of mass
    @param d : unitlength
        distance

    :: Return ::
    @return Next y position
    """
    next_y = y.m + v.mps * np.cos(phi.radians + beta.radians) * d.m
    return UnitLength(next_y)

def pred_phi(
    phi : UnitAngle,
    v : UnitSpeed,
    lr : UnitLength,
    beta : UnitAngle,
    d : UnitLength
) -> UnitAngle:
    """
    Predict next heading angle.

    :: Parameters ::
    @param phi : UnitAngle
        Current heading angle
    @param v : UnitSpeed
        Current Speed
    @param lr : UnitLength
        Distance from center of gravity to rear axes
    @param beta : UnitAngle
        Slip angle at center of mass
    @param d : UnitLength
        distance

    :: Return ::
    @return next heading angle
    """
    next_phi = phi.radians + (v.mps/lr.m) * np.sin(beta.radians) * d.m
    return UnitAngle(radians = next_phi)

def pred_v(
    v : UnitSpeed,
    a : UnitAcceleration,
    d : UnitLength
) -> UnitSpeed:
    """
    Predict next velocity

    :: PARAMETERS ::
    @param v : UnitSpeed
        Current speed
    @param a : UnitAcceleration
        Absolute acceleration
    @param d : UnitLengh
        distance
    
    :: RETURN ::
    @return next velocity
    """
    next_v = v.mps + a.mpsps * d.m
    return UnitSpeed(mps = next_v)

def calc_beta(
    lr : UnitLength,
    lf : UnitLength,
    delta : UnitAngle
) -> UnitAngle:
    """
    Calculate slip angle at center of mass

    :: PARAMETERS ::
    @param lr : UnitLength
        Distance from center of gravity to rear axes
    @param lf : UnitLength
        Distance from center of gravity to front axes
    @param delta : UnitAngle
        Steering angle of the front wheel
    """
    beta = np.arctan( (lr.m/(lf.m+lr.m)) * np.tan(delta.radians) )
    return UnitAngle(radians = beta)

def calc_distance(v : UnitSpeed, a : UnitAcceleration, t : UnitTime) -> UnitLength:
    """
    Calculates distance given a velocity @v and 
     acceleration @a over a time window @t.
    """
    d = v.mps * t.s + 0.5 * a.mpsps * t.s
    return UnitLength(m = d)

def n_intervals(a : float, b : float , n : int) -> list[float]:
    """
    Provides values for @n intervals between @a and @b
    """
    assert n >= 2
    assert a != b
    assert isinstance(a, float) and isinstance(b, float)
    nums = [i for i in range(n)]
    return [sxp.project(a, b, num/(n-1)) for num in nums]




class VehicleKinematicsScenario(sxp.Scenario):
    def __init__(self, params : pd.Series):
        req_features = [
            "lf", "lr", "x", "y", "v", "phi", "t",
            "a_min", "a_max", "delta_min", "delta_max"
        ]
        assert all([feat in req_features for feat in params.index])
        super().__init__(params)

        # Arbitrary scrore.
        self._score = pd.Series({"driveable_area" : 10})

        # Determine distance and beta
        d_min = calc_distance(params["v"], params["a_min"], params["t"])
        d_max = calc_distance(params["v"], params["a_max"], params["t"])

        # Number of intervals
        n = 5

        # Get intervals
        d_intervals = [UnitLength(m = d) for d in\
                        n_intervals(d_min.m, d_max.m, n)]
        a_intervals = [UnitAcceleration(mpsps = a) for a in \
            n_intervals(params["a_min"].mpsps, params["a_max"].mpsps, n)]
        delta_intervals = [UnitAngle(degrees = delta) for delta in \
            n_intervals(params["delta_min"].degrees, 
                        params["delta_max"].degrees, n)]

        # Get combinations
        control_vars = []
        for d in d_intervals:
            for a in a_intervals:
                for delta in delta_intervals:
                    s = pd.Series({"d" : d, "a" : a, "delta" : delta})
                    control_vars.append(s)
                continue
            continue
        df_cv = pd.DataFrame(control_vars)
        
        # Perform Estimates
        estimates = []
        for i in range(len(df_cv.index)):
            d, a, delta = df_cv.iloc[i].to_list()
            beta = calc_beta(params["lr"], params["lf"], delta)
            x = pred_x(params["x"], params["v"], params["phi"], 
                       beta, d)
            y = pred_y(params["y"], params["v"], params["phi"], 
                       beta, d)
            phi = pred_phi(params["phi"], params["v"], params["lr"],
                         beta, d)
            v = pred_v(params["v"], a, d)
            s = pd.Series({
                "lf" : params["lf"],
                "lr" : params["lr"],
                "x" : x,
                "y" : y,
                "d" : d,
                "phi" : phi,
                "delta" : delta,
                "v" : v,
                "a" : a,
                "t" : params["t"]
            })
            estimates.append(s)
            continue
        df = pd.DataFrame(estimates)

        print(df.to_csv("out/hhh.csv", index=False))
    
        
        fig = plt.figure(figsize=(5,5))
        ax = fig.gca()
        x = [pos.m for pos in df["x"]]
        y = [pos.m for pos in df["y"]]
        ax.scatter(x, y, color="black", marker=".")
        ax.scatter(params["x"].m, params["y"].m, color="red", marker=".")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        plt.savefig("out/rawr.png")
        return
    
    @property
    def score(self) -> pd.Series:
        return self._score

class VehicleKinematicsSUMOScenario(VehicleKinematicsScenario):
    def __init__(self, params : pd.Series):
        """
        Wraps the VehicleKinematicsScenario with SUMO terms.
        """
        req_features = [
            "veh_len.m", "x.m", "y.m", "speed.mps", 
            "max_accel.mps^2", "min_accel.mps^2",
            "cur_traj.deg", "steering_angle.deg",
            "phi.deg", "time_window.s"
        ]
        assert all([feat in req_features for feat in params.index])
            
        lf = UnitLength(m = params["veh_len.m"]/2)
        lr = UnitLength(m = lf.m)
        x = UnitLength(m = params["x.m"])
        y = UnitLength(m = params["y.m"])
        v = UnitSpeed(mps = params["speed.mps"])
        phi = UnitAngle(degrees = params["phi.deg"])
        t = UnitTime(s = params["time_window.s"])
        a_min = UnitAcceleration(mpsps = params["min_accel.mps^2"])
        a_max = UnitAcceleration(mpsps = params["max_accel.mps^2"])
        delta_min = UnitAngle(degrees = -params["steering_angle.deg"])
        delta_max = UnitAngle(degrees = params["steering_angle.deg"])

        params = pd.Series({
            "lf" : lf,
            "lr" : lr,
            "x" : x,
            "y" : y,
            "v" : v,
            "phi" : phi,
            "t" : t,
            "a_min" : a_min,
            "a_max" : a_max,
            "delta_min" : delta_min,
            "delta_max" : delta_max
        }) 

        super().__init__(params)
        return




def _test():
    
    return

if __name__ == "__main__":
    _test()