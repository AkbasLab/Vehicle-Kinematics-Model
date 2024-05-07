import numpy as np
import scenarioxp as sxp
import pickle
import shapely.geometry

import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.axes import Axes
import pandas as pd

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

def entropy(px : float) -> float:
    """
    PDF of value x.
    returns entropy for one case.
    """
    return -px * np.log2(px)

def closest_polygon(
        point : shapely.geometry.Point, 
        multipolygon : shapely.geometry.multipolygon.MultiPolygon
    ) -> shapely.geometry.Polygon:
    """
    Finds the closest subpolygon in @multipolygon to @point
    """
    distances = [point.distance(poly.centroid) for poly in multipolygon]
    closest = multipolygon[distances.index(min(distances))]
    return closest

def lattice_plot(
        df : pd.DataFrame,
        features : list[str],
        score_feat : str,
        fig_size : list[float, float],
        dpi : int,
        img_fn : str = "",
        trim : float = 0,
        rasterized : bool = False
    ):
    """
    Generates a lattice plot from a dataframe.

    :: Parameters ::
    df : pd.Dataframe
        Dataframe with features and scores.
    features : list[str]
        List of feature names
    score_feat : str
        Score feature name
    fig_size : list[float, float]
        Figure width and height in inches
    dpi : int
        Dots per inch
    img_fn : str
        Filename to save img. Reccomended is .png or .pdf
    trim : float
        Trim the data by p proportion of outliers.
    """
    assert trim >= 0 and trim < 1
    if trim > 0:
        df = remove_outliers(df, trim)

    # Clear plot
    plt.clf()

    # Colors
    cmap_id = "gist_heat"
    cmap = mpl.colormaps[cmap_id]
    
    # Normalize score
    scores_norm = (df[score_feat]-df[score_feat].min())\
                    /(df[score_feat].max()-df[score_feat].min())
    df["color"] = scores_norm.apply(cmap)
    a = df[score_feat].min()
    b = df[score_feat].max()
    norm_ticks = [.0, .1, .2, .3, .4, .5, .6, .7, .8, .9, 1.]
    score_ticks = [np.round((b-a)*x + a, decimals=2) for x in norm_ticks]
    
    # Make the plots
    plt.rc("font", size=8)
    fig = plt.figure(figsize=fig_size, dpi=dpi)
    nrow = len(features)
    ncol = len(features)
    for irow, y_feat in enumerate(features):
        for icol, x_feat in enumerate(features):
            ax : Axes = plt.subplot2grid((ncol, nrow), (irow, icol))

            # Setup labels
            is_left = icol == 0
            is_bottom = irow == nrow - 1

            ax.tick_params(
                left = is_left, 
                right = False , 
                labelleft = is_left , 
                labelbottom = is_bottom, 
                bottom = is_bottom
            )

            if is_left:
                ax.set_ylabel(y_feat)
            if is_bottom:
                ax.set_xlabel(x_feat)

            # Skip same plot
            if x_feat == y_feat:
                ax.set_facecolor("grey")
                continue

            # Make scatter plot
            ax.scatter(
                df[x_feat],
                df[y_feat],
                color = df["color"],
                marker=",",
                s=(72/dpi)**2,
                rasterized = rasterized
            )

            ax.set_facecolor("lightgray")
            continue
        continue

    
    # plt.tight_layout()
    plt.subplots_adjust(hspace=0,wspace=0)
    

    # Put the colorbar on the grid
    cbar_ax = fig.add_axes([0.91, 0.12, 0.01, .75])
    cb = fig.colorbar(
        mpl.cm.ScalarMappable(
            norm=mpl.colors.Normalize(0, 1), 
            cmap=cmap_id
        ),
        cax=cbar_ax, 
        orientation="vertical",
        label=score_feat,
    )
    cb.set_ticks(norm_ticks)
    cb.set_ticklabels(score_ticks)

    
    if not img_fn == "":
        plt.savefig(img_fn, bbox_inches="tight")
    return

def remove_outliers(df : pd.DataFrame, p : float):
    """
    Remove a proportion p of outliers from a dataframe @df
    """
    assert p > 0 and p < 1.0
    # Calculate z-scores for numerical columns
    z_scores = np.abs((df - df.mean()) / df.std())
    
    # Compute the maximum z-score across all columns for each row
    max_z_scores = z_scores.max(axis=1)
    
    # Define a threshold for outliers (e.g., z-score greater than 3)
    threshold = max_z_scores.quantile(1 - p)
    
    # Remove rows with z-scores exceeding the threshold
    cleaned_df = df[max_z_scores <= threshold]
    
    return cleaned_df

def scatter_grid_plot(
        df : pd.DataFrame,
        features : list[str],
        score_feat : str,
        fig_size : list[float,float] = [7,7],
        dpi : float = 300,
        img_fn : str = "out/rawr.png",
        ncol : int = None,
        nrow : int = None
    ):
    """
    Generates a lattice plot from a dataframe.

    :: Parameters ::
    df : pd.Dataframe
        Dataframe with features and scores.
    features : list[str]
        List of feature names
    score_feat : str
        Score feature name
    fig_size : list[float, float]
        Figure width and height in inches
    dpi : int
        Dots per inch
    img_fn : str
        Filename to save img. Reccomended is .png or .pdf
    """
    
    
    fig = plt.figure(figsize=fig_size, dpi=dpi)
    
    if ncol is None and nrow is None:
        ncol = 2
    if not (ncol is None):
        nrow = int(np.ceil(len(features)/ncol))
    elif not (nrow is None):
        ncol = int(np.ceil(len(features)/nrow))
    else:
        raise NotImplementedError
    icol = 0
    irow = 0
    for feat in features:
        # print("col:", icol, "row:", irow)
        ax : Axes = plt.subplot2grid((nrow, ncol), (irow, icol))

        # Setup labels
        is_bottom = irow == nrow - 1

        ax.scatter(
            df[feat],
            df[score_feat],
            color = "black",
            marker = "."
        )

        ax.set_xlabel(feat)
        ax.set_ylabel(score_feat)

        icol += 1
        if icol >= ncol:
            irow += 1
            icol = 0
        
        continue

    
    plt.tight_layout()


    if not img_fn == "":
        plt.savefig(img_fn, bbox_inches="tight")
    return

def _test():
    return

if __name__ == "__main__":
    _test()