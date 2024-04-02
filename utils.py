import numpy as np
import scenarioxp as sxp
import pickle

def mps2kph(mps : float) -> float:
    return 3.6 * mps

def kph2mps(kph : float) -> float:
    return kph/3.6

def deg2rad(deg : float) -> float:
    return deg * np.pi/180

def n_intervals(a : float, b : float, n : int) -> list[float]:
    """
    Provides values for @n intervals between @a and @b
    """
    assert a != b
    nums = [i for i in range(n)]
    return [sxp.project(a, b, num/(n-1)) for num in nums]

def distance_to(x0, y0, x1, y1) -> float:
    return np.sqrt((x1-x0)**2+(y1-y0)**2)

def save(data, fn : str):
    with open(fn, "wb") as f:
        pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)
    return

def load(fn : str):
    with open(fn, "rb") as f:
        return pickle.load(f)