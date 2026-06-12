import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
import math

from geometry_msgs.msg import PoseStamped

class PathSmoother:
    def __init__(self, master_track_ps: list[PoseStamped]):
        self._raw_waypoints = []
        for pose in master_track_ps:
            x = pose.pose.position.x
            y = pose.pose.position.y
            self._raw_waypoints.append(np.array([x,y]))
    
    def smooth_track(self, num_points=100, initial_smoothing=2.0, max_offset=3.0, safety_margin=1.0, num=15):
        """
        Generates a B-Spline and iteratively increases smoothing until the 
        Minimum Radius of Curvature is large enough to prevent swallowtail singularities.
        """
        x = [wp[0] for wp in self._raw_waypoints]
        y = [wp[1] for wp in self._raw_waypoints]

        pad_start_x = np.linspace(x[0], x[1], num)[1:-1]
        pad_start_y = np.linspace(y[0], y[1], num)[1:-1]
        pad_end_x = np.linspace(x[-2], x[-1], num)[1:-1]
        pad_end_y = np.linspace(y[-2], y[-1], num)[1:-1]

        padded_x = [x[0]] + list(pad_start_x) + x[1:-1] + list(pad_end_x) + [x[-1]]
        padded_y = [y[0]] + list(pad_start_y) + y[1:-1] + list(pad_end_y) + [y[-1]]

        smoothing = initial_smoothing
        max_iterations = 20  

        for iteration in range(max_iterations):
            tck, u = splprep([padded_x, padded_y], s=smoothing, k=3)
            u_fine = np.linspace(0, 1, num_points)

            dx, dy = splev(u_fine, tck, der=1)

            ddx, ddy = splev(u_fine, tck, der=2)

            numerator = np.abs(dx * ddy - dy * ddx)
            denominator = np.power(dx**2 + dy**2, 1.5)

            kappa = np.divide(numerator, denominator, out=np.zeros_like(numerator), where=denominator!=0)

            max_kappa = np.max(kappa)
            r_min = float('inf') if max_kappa == 0 else 1.0 / max_kappa

            if r_min >= (max_offset + safety_margin):
                print(f"Path safe at smoothing={smoothing:.1f} | R_min={r_min:.2f}m")
                master_x, master_y = splev(u_fine, tck)
                return master_x, master_y, u_fine, tck
            else:
                smoothing += 1.0
        print(f"Warning: Reached max iterations. Outputting best effort at smoothing={smoothing:.1f}.")
        master_x, master_y = splev(u_fine, tck)
        return master_x, master_y, u_fine, tck
    
    def compute_dynamic_tracks(self, master_x, master_y, u_fine, tck, num_robots, formation_width=4.0):
        """
        Dynamically computes parallel tracks where Robot 0 drives exactly on the 
        Master Path (offset 0.0), and subsequent robots are distributed up to the full width.

        Returns: list of lists containing np.array([x, y])
        """
        dx, dy = splev(u_fine, tck, der=1)
        magnitude = np.hypot(dx, dy)
        nx_unit = -dy / magnitude 
        ny_unit = dx / magnitude

        # CHANGED: Anchor the first robot at 0.0 (the master path itself)
        if num_robots == 1:
            offsets = np.array([0.0]) 
        else:
            offsets = np.linspace(0.0, formation_width, num=num_robots)

        all_tracks = []
        for i, offset in enumerate(offsets):
            track_x = master_x + (nx_unit * offset)
            track_y = master_y + (ny_unit * offset)

            track_coords = [np.array([tx, ty]) for tx, ty in zip(track_x, track_y)]
            all_tracks.append(track_coords)

        return all_tracks

    def master_arclength(self, master_x, master_y) -> np.ndarray:
        """
        Cumulative arc length of the master path at each sampled point.

        This is the SHARED progress coordinate for the formation: every robot's
        PathParameterizer maps this same value onto its own offset track. Because
        every offset track is sampled at the same u_fine as the master, sample i
        of every track corresponds to master_arclength[i], so equal progress in
        this coordinate keeps all robots on the same master normal line.

        Returns a strictly-increasing array the same length as the master samples.
        """
        mxy = np.column_stack([np.asarray(master_x, dtype=float),
                               np.asarray(master_y, dtype=float)])
        seg = np.linalg.norm(np.diff(mxy, axis=0), axis=1)
        return np.concatenate([[0.0], np.cumsum(seg)])