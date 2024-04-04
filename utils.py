import numpy as np
import scenarioxp as sxp
import pickle
import shapely.geometry

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
    
def project_point(
        x : float, 
        y : float, 
        distance : float, 
        angle_radians : float
    ) -> list[float, float]:
    """
    Projects an @x,@y point @distance by @angle_radians

    Returns new x,y point.
    """
    new_x = x + distance * np.cos(angle_radians)
    new_y = y + distance * np.sin(angle_radians)
    return new_x, new_y

def linestring2polygon(
        linestring : shapely.geometry.LineString, 
        width : float
    ) -> shapely.geometry.Polygon:
    """
    Transforms a @linestring into a Polygon given a @width
    by buffering the LineString with the given width to create a polygon
    """
    return linestring.buffer(width / 2, cap_style=3, join_style=2)


def gaussian_pdf(x, mu :float = 0, sigma : float = 1):
    """
    Calculate the probability density function (PDF) of a Gaussian distribution at point x.
    
    Parameters:
        x: float or array-like, the point(s) at which to evaluate the PDF
        mu: float, the mean of the Gaussian distribution
        sigma: float, the standard deviation of the Gaussian distribution
        
    Returns:
        pdf_value: float or array-like, the value(s) of the PDF at point(s) x
    """
    pdf_value = 1 / (np.sqrt(2 * np.pi) * sigma) * np.exp(-0.5 * ((x - mu) / sigma)**2)
    return pdf_value

def _test():
    return

if __name__ == "__main__":
    _test()