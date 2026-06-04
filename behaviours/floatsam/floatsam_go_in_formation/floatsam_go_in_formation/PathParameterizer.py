import numpy as np
from geometry_msgs.msg import PoseStamped

class PathParameterizer:
    def __init__(self, map_poses: list[PoseStamped], look_a_head_distance: float) -> None:
        self._map_waypoints = []
        for pose in map_poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            self._map_waypoints.append(np.array([x, y]))
        
        self._s_table = [0.0] 
        for i in range(1, len(self._map_waypoints)):
            dist = np.linalg.norm(self._map_waypoints[i] - self._map_waypoints[i-1])
            self._s_table.append(self._s_table[-1] + dist)
        
        self._current_s = 0.0
        # Save the parameter, we will calculate the actual lookahead dynamically
        self._look_a_head_distance = look_a_head_distance 

    def advance_carrot(self, ds: float) -> None:
        new_s = self._current_s + ds
        if new_s > self._s_table[-1]:
            new_s = self._s_table[-1]
        self._current_s = new_s

    def _get_position_at_s(self, target_s: float) -> np.ndarray:
        """
        PRIVATE HELPER: Returns the [X, Y] coordinate for ANY given distance along the track.
        """
        # Clamp to the end of the track (Safeguard)
        if target_s >= self._s_table[-1]:
            return self._map_waypoints[-1]
            
        for i in range(len(self._s_table) - 1):
            s_start = self._s_table[i]
            s_end = self._s_table[i+1]
            
            if s_start <= target_s < s_end:
                ratio = (target_s - s_start) / (s_end - s_start)
                start_wp = self._map_waypoints[i]
                end_wp = self._map_waypoints[i+1]
                
                x_pos = start_wp[0] + ratio * (end_wp[0] - start_wp[0])
                y_pos = start_wp[1] + ratio * (end_wp[1] - start_wp[1])
                return np.array([x_pos, y_pos])
                
        return self._map_waypoints[-1]

    def get_carrots(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Returns a tuple containing: (main_carrot_position, lookahead_carrot_position)
        """
        main_carrot = self._get_position_at_s(self._current_s)
        
        lookahead_s = self._current_s + self._look_a_head_distance
        lookahead_carrot = self._get_position_at_s(lookahead_s)
        
        return main_carrot, lookahead_carrot