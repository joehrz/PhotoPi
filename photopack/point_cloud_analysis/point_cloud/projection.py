import logging
from typing import Iterable, Union

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import alphashape
from shapely.geometry import Polygon, MultiPolygon
from scipy.spatial import cKDTree

logger = logging.getLogger(__name__)


def _iter_polys(geom: Union[Polygon, MultiPolygon]) -> Iterable[Polygon]:
    """Yield individual :class:`~shapely.geometry.Polygon` objects.

    Parameters
    ----------
    geom
        Either a single ``Polygon`` or a ``MultiPolygon``.  Shapely 1.x allows
        direct iteration over ``MultiPolygon``; Shapely ≥ 2.0 stores the
        component polygons inside the ``.geoms`` attribute.  This helper hides
        that behavioural change so the rest of the codebase can remain
        agnostic to the Shapely version installed.
    """
    if isinstance(geom, Polygon):
        yield geom
    elif isinstance(geom, MultiPolygon):
        for g in getattr(geom, "geoms", geom):  # Shapely 1.x ↔ 2.x
            yield g


def _auto_alpha(points_xy: np.ndarray, k: float = 3.0) -> float:
    """Compute a data-driven alpha value.

    The heuristic sets

        alpha = k x d,

    where d is the median nearest-neighbour distance in the point
    pattern. Empirically, k gives a tight but still
    contiguous footprint for canopy point clouds.

    Parameters
    ----------
    points_xy
        (N, 2) array of x,y coordinates.
        k Multiplicative factor applied to the median nearest-neighbour distance.

    Returns
    -------
    float
        Suggested alpha in the same units as points_xy.
    """
    distances, _ = cKDTree(points_xy).query(points_xy, k=2)  # 1st neighbour = self
    median_nn = np.median(distances[:, 1])
    return k * median_nn


class ProjectionAnalyzer:
    """Analyse the 2D ground-plane projection of a 3D point cloud.

    The class offers a one-stop interface to

    * compute an alpha shape footprint of the canopy,
    * scale raw (unit-less) area into real-world units, and
    * generate figures.

    A robust "auto" mode selects the alpha parameter from the point density,
    giving consistent footprints across datasets captured at different
    resolutions.

    Notes
    -----
    * Dimensionality -Only the *x* and *y* coordinates are used for the
      alpha shape.  *z* is ignored.
    * Scaling -scale converts internal mesh units (e.g. "voxel units") to
      physical units (e.g. millimetres).  All reported areas therefore scale
      by scale**2.
    * One-shot workflow

      >>> proj = ProjectionAnalyzer(point_cloud, scale=30.0)
      >>> area_physical = proj.compute_alpha_shape_area(alpha="auto")
      >>> proj.plot_points_and_alpha_shape()
    """

    def __init__(self, point_cloud: o3d.geometry.PointCloud, scale: float = 1.0) -> None:
        """Construct the analyzer.

        Parameters
        ----------
        point_cloud
            Input point cloud (any Open3D PointCloud). Only the point
            coordinates are accessed; colours/normals are ignored.
        scale
            Conversion factor S such that 1 mesh-unit x S = 1 physical-unit
            (e.g. 1 voxel x 30 = 30mm).  Lengths scale with S; **areas scale
            with S².  Set scale=1.0 to keep results unit-less.
        """
        self.point_cloud = point_cloud
        self.scale: float = float(scale)
        self.points: np.ndarray = np.asarray(point_cloud.points)
        self.alpha_shape: Union[Polygon, MultiPolygon, None] = None
        self.alpha_shape_area: float | None = None  # physical units² if computed

    # ------------------------------------------------------------------
    # α‑shape computation
    # ------------------------------------------------------------------
    def compute_alpha_shape_area(self, *, alpha: Union[float, str] = "auto", apply_scale: bool = True) -> float:
        """Compute the alpha shape footprint area.

        Parameters
        ----------
        alpha
            "auto" -Choose alpha from the point density (see :func:`_auto_alpha`).
            float  -Use this value verbatim (must be in the same units as the
            x-y coordinates).
        apply_scale
            If True, multiply the raw area by scale**2 so the return value
            is expressed in physical units.  If False, return the raw
            unit-less area.

        Returns
        -------
        float
            Footprint area.  Units depend on apply_scale.
        """
        xy: np.ndarray = self.points[:, :2]

        # auto‑select α if requested
        if isinstance(alpha, str) and alpha.lower() == "auto":
            alpha = _auto_alpha(xy)
            logger.debug("auto-selected α = %.4f", alpha)

        self.alpha_shape = alphashape.alphashape(xy, alpha)

        if not self.alpha_shape or self.alpha_shape.is_empty:
            logger.warning("α-shape is empty for α=%.3f", alpha)
            self.alpha_shape_area = 0.0
            return 0.0

        raw_area: float = self.alpha_shape.area
        self.alpha_shape_area = raw_area * (self.scale ** 2) if apply_scale else raw_area
        return self.alpha_shape_area

    # ------------------------------------------------------------------
    # Plotting helpers
    # ------------------------------------------------------------------
    def plot_original_points(self, *, save_fig: bool = False, filename: str = "original_points.png") -> None:
        """Scatter-plot the x-y projection of the raw point cloud."""
        xy = self.points[:, :2]
        plt.figure()
        plt.scatter(xy[:, 0], xy[:, 1], s=6, label="points")
        plt.axis("equal")
        plt.title("XY projection")
        if save_fig:
            plt.savefig(filename, dpi=300, bbox_inches="tight")
        plt.show()

    def plot_alpha_shape(self, *, save_fig: bool = False, filename: str = "alpha_shape.png") -> None:
        """Visualise the α-shape alone."""
        if self.alpha_shape is None:
            logger.warning("compute_alpha_shape_area() has not been called yet.")
            return

        plt.figure()
        for poly in _iter_polys(self.alpha_shape):
            plt.fill(*poly.exterior.xy, alpha=0.25)
        plt.axis("equal")
        plt.title(f"α-shape area = {self.alpha_shape_area:.2f}")
        if save_fig:
            plt.savefig(filename, dpi=300, bbox_inches="tight")
        plt.show()

    def plot_points_and_alpha_shape(self, *, save_fig: bool=False, filename: str ="points_and_alpha_shape.svg") -> None:
        """
        Plots the original points and the alpha shape together.
        
        Args:
            save_fig (bool, optional): Whether to save the figure. Defaults to False.
            filename (str, optional): Filename for saving the figure. Defaults to "points_and_alpha_shape.svg".
        """
        if self.alpha_shape is None:
            logger.warning("Alpha shape has not been computed yet.")
            return
        
        xy_points = self.points[:, :2]
        alpha_shape_area = self.alpha_shape_area
        plt.figure(figsize=(12, 6))  # Adjust figure size if desired
        
        # Plot points
        plt.plot(xy_points[:, 0], xy_points[:, 1], 'o', label='Points', markersize=3)
        
        # Plot the alpha shape
        if isinstance(self.alpha_shape, Polygon):
            plt.fill(*self.alpha_shape.exterior.xy, color='blue', alpha=0.2,
                    label=f'Alpha Shape (Area: {alpha_shape_area:.4f})')
        elif isinstance(self.alpha_shape, MultiPolygon):
            for polygon in self.alpha_shape:
                plt.fill(*polygon.exterior.xy, color='blue', alpha=0.2)
        
        # Increase axis labels and tick labels
        plt.xlabel("X-axis coordinate of voxel", fontsize=16)
        plt.ylabel("Y-axis coordinate of voxel", fontsize=16)
        plt.xticks(fontsize=16)
        plt.yticks(fontsize=16)
        
        # Increase legend font size
        plt.legend(fontsize=14)
        
        # Save the figure if requested (as both SVG and PNG)
        if save_fig:
            plt.savefig(filename, bbox_inches='tight')
            plt.savefig("points_and_alpha_shape.png", bbox_inches='tight')
        plt.show()
