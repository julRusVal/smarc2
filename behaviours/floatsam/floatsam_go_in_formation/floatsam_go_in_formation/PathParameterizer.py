import numpy as np
from geometry_msgs.msg import PoseStamped


class PathParameterizer:
    """
    Shared-parameter carrot follower.

    All robots in the formation advance a single shared progress variable measured
    in MASTER arc length (metres along the master path). This instance owns one
    robot's offset track, sampled at the same spline parameters as the master, and
    returns the carrot point on that track corresponding to the shared progress.

    Because every robot samples its own track at the same master-arc-length value,
    and the tracks are generated from the same `u_fine` sampling of the master,
    every robot's carrot lies on the same master normal line -> the formation line
    stays perpendicular to the master path by construction.

    Args:
        map_poses          : this robot's offset track, sampled at u_fine
                             (list of PoseStamped, one per master sample).
        master_s_table     : cumulative arc length of the MASTER path at each
                             sample. Must be the same length as `map_poses` and
                             strictly increasing.
        look_a_head_distance: lookahead, expressed in master arc length (m).
    """

    def __init__(self, map_poses: list[PoseStamped], master_s_table, look_a_head_distance: float) -> None:
        self._track_x = np.array([p.pose.position.x for p in map_poses], dtype=float)
        self._track_y = np.array([p.pose.position.y for p in map_poses], dtype=float)

        self._master_s = np.asarray(master_s_table, dtype=float)

        if len(self._track_x) != len(self._master_s):
            raise ValueError(
                f"Track samples ({len(self._track_x)}) and master_s_table "
                f"({len(self._master_s)}) must be the same length."
            )

        self._s_end = float(self._master_s[-1])
        self._current_s = 0.0
        self._look_a_head_distance = look_a_head_distance

    def advance_carrot(self, ds: float) -> None:
        """Advance the shared progress by `ds` metres of master arc length."""
        self._current_s = min(self._current_s + ds, self._s_end)

    def _sample_at_s(self, target_s: float) -> np.ndarray:
        """
        Return the [x, y] point on THIS robot's track corresponding to a given
        master arc length. Linear interpolation between samples; np.interp clamps
        outside [0, s_end].
        """
        target_s = min(max(target_s, 0.0), self._s_end)
        x = np.interp(target_s, self._master_s, self._track_x)
        y = np.interp(target_s, self._master_s, self._track_y)
        return np.array([x, y])

    def get_carrots(self) -> tuple[np.ndarray, np.ndarray]:
        """Returns (main_carrot_position, lookahead_carrot_position)."""
        main_carrot = self._sample_at_s(self._current_s)
        lookahead_carrot = self._sample_at_s(self._current_s + self._look_a_head_distance)
        return main_carrot, lookahead_carrot

    @property
    def current_s(self) -> float:
        return self._current_s

    @property
    def is_at_end(self) -> bool:
        return self._current_s >= self._s_end