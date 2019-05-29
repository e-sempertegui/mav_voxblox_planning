Local Planner:

To launch simply run: roslaunch mav_local_planner local_planner.launch

Arguments:
		mav_name: change to name of your MAV
		voxel_size: adjust to correspondent voxel size (ensure it is the same as for voxblox node)
		robot_radius: Change to your robot radius (keep in mind this affects significantly traversable space of the robot)

Topics:
		odometry: remap to the odometry topic of your UAV
		mav_local_planner/esdf(tsdf)_map_in: Ensure these two match with the topic ESDF/TSDF provided by voxblox node
		
Parameters:
		planning_horizon_m: ONLY change if robot is consistenly not able to find path in a particular region of the space. Determines how far local plan/trajectory is computed at each replanning step.
		num_segments: Change to desired number (INT type!) of segments in polynomial trajectory. 6 segments tends to be less prone to fail in narrow spaces. If not specified takes the default value used by ASL (i.e. 3 segments).



Global Planner:

To launch run: roslaunch voxblox_rrt_planner firefly_rrt.launch

Arguments:
		mav_name: change to name of your MAV
		voxel_size: adjust to correspondent voxel size (ensure it is the same as for voxblox node)
		robot_radius: Change to your robot radius

Topics:
		odometry: remap to the odometry topic of your UAV
		voxblox_rrt_planner/esdf(tsdf)_map_in: Ensure these two match with the topic ESDF/TSDF provided by voxblox node		

Parameters:
		num_seconds_to_plan: Adjust for the number of seconds planner takes to provide a solution. Each replanning iteration in the case of obstacles found while flying also takes this time to plan.
		optimistic: Set to true or false according to whether you look for an optimistic or pessimistic (only able to plan in known space) global planner.
		constant_altitude: Set to true for adding constant altitude objective to cost function. Otherwise planner only cares about path lenght objective. 
		planner_type: Specify planner type to use. Available types are listed in the launch file
		activate_replanning: Set to true to enable replanning feature
		gp_replan_dt: Time taken by planner to replan AFTER a solution to the previous planning request has finished
		gp_replan_lookahead_sec_: Specifies how far along the trajectory MAV is projected as to check for collisions in advance. If modified, make sure it satisfies the following: gp_replan_lookahead_sec_>=num_seconds_to_plan
		alpha/beta: Weights for optimization objectives of global planner (altitude and path lenght) in the form [alpha*LenghtObj + beta*AltitudeObj]. Only VALID IF constant_altitude is set to true. 

