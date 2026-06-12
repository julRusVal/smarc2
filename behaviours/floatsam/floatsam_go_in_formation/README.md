ros2 action send_goal /floatsam_usv_0/go_in_formation smarc_msgs/action/BaseAction "{goal: {data: '{\"desired_speed\": 2.0, \"tracks\": [[{\"latitude\": 58.8405163983891, \"longitude\": 17.6517780965932}, {\"latitude\": 58.8406214504201, \"longitude\": 17.6519172412154}], [{\"latitude\": 58.8407163983891, \"longitude\": 17.6517780965932}, {\"latitude\": 58.8409163983891, \"longitude\": 17.6517780965932}]]}'}}"


ros2 action send_goal /floatsam_usv_0/go_in_formation smarc_msgs/action/BaseAction \
"{goal: {data: '{\"desired_speed\": 2.0, \"track\": [{\"latitude\": 58.8404946676474, \"longitude\": 17.6518338919726}, {\"latitude\": 58.8405449682116, \"longitude\": 17.6518419859228}, {\"latitude\": 58.8405926660047, \"longitude\": 17.6518436714537}, {\"latitude\": 58.840588636238, \"longitude\": 17.651973506389}, {\"latitude\": 58.8405855492816, \"longitude\": 17.6520711146255}]}'}}"








58.8404946676474 17.6518338919726
58.8405449682116 17.6518419859228
58.8405926660047 17.6518436714537
58.840588636238 17.651973506389
58.8405855492816 17.6520711146255



58.8404973621284 17.6517269223936
58.84057615482 17.6517320009233
58.8406244825704 17.6517697066712
58.8406329057627 17.6518488029239
58.8406262750795 17.6520694593868



def _prepare_loop(self) -> None: 
        self._node.get_logger().info('Preparing loop.')

        # 1. Initialize smoother with converted GeoPoints
        self._path_smoother = PathSmoother(master_track_ps=self._track_in_map)

        # 2. Get formation width parameter dynamically (defaulting to 4.0 if not declared)
        formation_width = 4.0 

        # 3. CRITICAL: In a master-anchored setup, the maximum offset from the 
        # master path is the full formation_width, not formation_width / 2.0.
        max_offset = float(formation_width)

        # 4. Run the curvature check loop against the full width
        master_x, master_y, u_fine, tck = self._path_smoother.smooth_track(
            num_points=100, 
            initial_smoothing=2.0,
            max_offset=max_offset,
            safety_margin=1.0,
            num=15 
        )

        # 5. Extract shared progress coordinate
        master_s = self._path_smoother.master_arclength(master_x, master_y)

        # 6. Generate the master-anchored parallel tracks
        generated_tracks_raw = self._path_smoother.compute_dynamic_tracks(
            master_x, master_y, u_fine, tck, 
            num_robots=self._num_robots,
            formation_width=formation_width 
        )

        # 7. Package coordinates back into PoseStamped structures
        self._tracks_in_map = []
        for track_list in generated_tracks_raw:
            pose_list = []
            for point in track_list:
                ps = PoseStamped()
                ps.pose.position.x = point[0]
                ps.pose.position.y = point[1]
                pose_list.append(ps)
            self._tracks_in_map.append(pose_list)

        # 8. Run Hungarian Task Assignment
        if not self._HungarianAssignment():
            self._node.get_logger().error('Assignment Failed. Aborting loop preparation')
            return 

        # 9. Initialize parameterizer with this agent's allocated track
        self._path_parametrizer = PathParameterizer(
            self._this_robot_waypoints, master_s, self._look_a_head_distance)
        self._move_to_pending = False
        self._node.get_logger().info('Loop correctly prepared.')