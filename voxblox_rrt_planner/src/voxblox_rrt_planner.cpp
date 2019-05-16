#include <geometry_msgs/PoseArray.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/utils.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include <voxblox/utils/planning_utils.h>

#include "voxblox_rrt_planner/voxblox_rrt_planner.h"

namespace mav_planning {

VoxbloxRrtPlanner::VoxbloxRrtPlanner(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      frame_id_("odom"),
      visualize_(true),
      do_smoothing_(true),
      last_trajectory_valid_(false),
      lower_bound_(Eigen::Vector3d::Zero()),
      upper_bound_(Eigen::Vector3d::Zero()),
      optimistic_(false),
      constant_altitude(false),
      gp_replan_dt(0.5),
      gp_replan_lookahead_sec_(1.0),
      gp_spinner_(1, &gp_queue_),
      path_idx(0),
      old_path_idx(0),
      // able_to_plan(true),
      // counter(0),
      init_pose_adjust_counter(0),
      replanning(false),
      activate_replanning(false),
      voxblox_server_(nh_, nh_private_),
      rrt_(nh_, nh_private_) {
  constraints_.setParametersFromRos(nh_private_);

  std::string input_filepath;
  // bool optimistic_;
  nh_private_.param("optimistic", optimistic_, optimistic_);
  nh_private_.param("voxblox_path", input_filepath, input_filepath);
  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("frame_id", frame_id_, frame_id_);
  nh_private_.param("do_smoothing", do_smoothing_, do_smoothing_);
  nh_private_.param("gp_replan_dt", gp_replan_dt, gp_replan_dt);
  nh_private_.param("gp_replan_lookahead_sec_", gp_replan_lookahead_sec_, gp_replan_lookahead_sec_);
  nh_private_.param("activate_replanning", activate_replanning, activate_replanning);
  nh_private_.param("constant_altitude", constant_altitude, constant_altitude);

  //Publishers and odometry subscriber
  gp_odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                    &VoxbloxRrtPlanner::odometryCallback, this);

  path_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true);
  polynomial_trajectory_pub_ =
      nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>(
          "polynomial_trajectory", 1);

  waypoint_list_pub_ =
      nh_.advertise<geometry_msgs::PoseArray>("waypoint_list", 1);

  // Services
  planner_srv_ = nh_private_.advertiseService(
      "plan", &VoxbloxRrtPlanner::plannerServiceCallback, this);
  path_pub_srv_ = nh_private_.advertiseService(
      "publish_path", &VoxbloxRrtPlanner::publishPathCallback, this);

  esdf_map_ = voxblox_server_.getEsdfMapPtr();
  CHECK(esdf_map_);
  tsdf_map_ = voxblox_server_.getTsdfMapPtr();
  CHECK(tsdf_map_);

  if (!input_filepath.empty()) {
    // Verify that the map has an ESDF layer, otherwise generate it.
    if (!voxblox_server_.loadMap(input_filepath)) {
      ROS_ERROR("Couldn't load ESDF map!");

      // Check if the TSDF layer is non-empty...
      if (tsdf_map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0) {
        ROS_INFO("Generating ESDF layer from TSDF.");
        // If so, generate the ESDF layer!

        const bool full_euclidean_distance = true;
        voxblox_server_.updateEsdfBatch(full_euclidean_distance);
      } else {
        ROS_ERROR("TSDF map also empty! Check voxel size!");
      }
    }
  }
  double voxel_size =
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size();

  ROS_INFO(
      "Size: %f VPS: %lu",
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size(),
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxels_per_side());

  // TODO(helenol): figure out what to do with optimistic/pessimistic here.
  rrt_.setRobotRadius(constraints_.robot_radius);
  rrt_.setOptimistic(optimistic_);

  rrt_.setTsdfLayer(voxblox_server_.getTsdfMapPtr()->getTsdfLayerPtr());
  rrt_.setEsdfLayer(voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());

  voxblox_server_.setTraversabilityRadius(constraints_.robot_radius);

  // Set up the path smoother as well.
  smoother_.setParametersFromRos(nh_private_);
  smoother_.setMinCollisionCheckResolution(voxel_size);
  smoother_.setMapDistanceCallback(std::bind(&VoxbloxRrtPlanner::getMapDistance,
                                             this, std::placeholders::_1));

  // Loco smoother!
  loco_smoother_.setParametersFromRos(nh_private_);
  loco_smoother_.setMinCollisionCheckResolution(voxel_size);
  loco_smoother_.setMapDistanceCallback(std::bind(
      &VoxbloxRrtPlanner::getMapDistance, this, std::placeholders::_1));

  // Ramp smoother!
  ramp_smoother_.setParametersFromRos(nh_private_);


  if (visualize_) {
    voxblox_server_.generateMesh();
    voxblox_server_.publishSlices();
    voxblox_server_.publishPointclouds();
    voxblox_server_.publishTraversable();
  }

  // Set timer options (especify callback queue and function) for global replanning.
  ros::TimerOptions gp_timer_options(ros::Duration(gp_replan_dt),
      boost::bind(&VoxbloxRrtPlanner::gpTimerCallback, this, _1),
      &gp_queue_, false, false);

  // Create timer object for replanning.
  gp_timer_ = nh_.createTimer(gp_timer_options);

  // Start global planner spinner (handling callbacks in gp_queue_)
  gp_spinner_.start();

  ROS_INFO("OPTIMISTIC PLANNER? %d", optimistic_);
  ROS_INFO("CONSTANT ALTITUDE PLANNING? %d", constant_altitude);
  ROS_INFO("REPLANNING ENABLE? %d", activate_replanning);
  if (activate_replanning)
  {
    ROS_INFO("REPLANNING TIMER dt = %f", gp_replan_dt);
  }

}

void VoxbloxRrtPlanner::odometryCallback(const nav_msgs::Odometry& msg) {
  mav_msgs::eigenOdometryFromMsg(msg, &odometry_);
}

bool VoxbloxRrtPlanner::publishPathCallback(std_srvs::EmptyRequest& request,
                                            std_srvs::EmptyResponse& response) {
  if (!last_trajectory_valid_) {
    ROS_ERROR("Can't publish trajectory, marked as invalid.");
    return false;
  }

  ROS_INFO("Publishing path.");

  if (!do_smoothing_) {
    geometry_msgs::PoseArray pose_array;
    pose_array.poses.reserve(last_waypoints_.size());
    for (const mav_msgs::EigenTrajectoryPoint& point : last_waypoints_) {
      geometry_msgs::PoseStamped pose_stamped;
      mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(point, &pose_stamped);
      pose_array.poses.push_back(pose_stamped.pose);
    }

    pose_array.header.frame_id = frame_id_;
    waypoint_list_pub_.publish(pose_array);
  } else {
    mav_planning_msgs::PolynomialTrajectory4D msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(
        last_trajectory_, &msg);

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    polynomial_trajectory_pub_.publish(msg);
  }

  if (activate_replanning)
  {
    // Start GLOBAL PLANNER TIMER ONCE PUBLISHED TO LOCAL PLANNER
    // (ideally/assumes robot starts moving instantenously)
    gp_timer_.start();
  }


  return true;
}

void VoxbloxRrtPlanner::computeMapBounds(Eigen::Vector3d* lower_bound,
                                         Eigen::Vector3d* upper_bound) const {
  if (esdf_map_) {
    voxblox::utils::computeMapBoundsFromLayer(*esdf_map_->getEsdfLayerPtr(),
                                              lower_bound, upper_bound);
  } else if (tsdf_map_) {
    voxblox::utils::computeMapBoundsFromLayer(*tsdf_map_->getTsdfLayerPtr(),
                                              lower_bound, upper_bound);
  }
}

void VoxbloxRrtPlanner::gpTimerCallback(const ros::TimerEvent& event){
  // counter++;
  // ROS_INFO("[Timer Callback] COUNTER = %d", counter);
  // if (able_to_plan){
  //   able_to_plan = 0;
  //   replanningRoutine();
  // }
  replanningRoutine();
}

// TODO: Check the 'collisions' and check what's the initial pose of the robot assumed for starting to replan
// is it actually taking the pose of the robot? (it seems is not as the new path started from a different location)
void VoxbloxRrtPlanner::replanningRoutine(){
  // Also has to use size_t for the path_idx in the case the plan is valid and you need to carry on (a record) with it
  size_t replan_start_idx;
  mav_msgs::EigenTrajectoryPointVector chunk_for_revision;
  bool new_gp_success;
  bool success_publish;
  mav_msgs::EigenTrajectoryPoint predicted_previous_pose;

  // Constants for adjusting estimated robot position (w.r.t. global path)
  constexpr double kCloseToOdometry = 1.5;
  constexpr double kPosesDistance = 0.03;   // How close poses need to be in order to consider them equal
  constexpr int kMaxFailureIterations = 7;
  constexpr int kSecondsInMotion = 5;  // must satisfy: kSecondsInMotion > num_seconds_to_plan + gp_replan_dt 
  const size_t step_size = 25;

  // (Over)Estimate current location in global trajectory (vector) w.r.t beginning of trajectory
  // If robot has just started following a new path the first time we replan the robot 
  // is expected to be timer_dt secs away. Otherwise just do the large overestimation. 
  if (path_idx > static_cast<size_t>(0))
  {
    path_idx += static_cast<size_t>((kSecondsInMotion)/constraints_.sampling_dt);  
  }
  else{
    path_idx += static_cast<size_t>((gp_replan_dt + 0.5)/constraints_.sampling_dt);
    ROS_INFO("[DEBUGGING] Using timer_dt to estimate location...");
  }
  // path_idx += static_cast<size_t>((kSecondsInMotion)/constraints_.sampling_dt);
  ROS_INFO("[DEBUGGING] path_idx = %d     global_path_size= %d", path_idx, global_path.size());
  mav_msgs::EigenTrajectoryPoint predicted_current_pose = *(global_path.begin() + path_idx);

  // Remap(pull) path idx to the inside of the sphere
  while ((odometry_.position_W - predicted_current_pose.position_W).norm() > 
          kCloseToOdometry){
    ROS_INFO("[REPLANNING ROUTINE] Readjusting path idx...");
    path_idx -= step_size;  //TODO: take care with what happens if norm is greater but because robot moved to fast so estimated pose is far behind
    predicted_current_pose = *(global_path.begin() + path_idx);
    ROS_INFO("[DEBUGGING] path_idx = %d", path_idx);
  }

  // If the prediction of the pose in the current replanning iteration is 'close' to
  // the predicted pose of the previous iteration increase counter as robot probably has not
  // moved due to error, otherwise reset the counter
  predicted_previous_pose = *(global_path.begin() + old_path_idx);
  if ((predicted_current_pose.position_W - predicted_previous_pose.position_W).norm() < 
      kPosesDistance){
    init_pose_adjust_counter++;
  } else{
    init_pose_adjust_counter = 0;
  }

  // Stop replanning if maximum number of repeats is exceeded (i.e. we are planning
  // from the same location repeatedly)
  if (init_pose_adjust_counter >= kMaxFailureIterations){
    // able_to_plan = 1;
    replanning = 0;
    gp_timer_.stop();
    ROS_INFO("Max number of replanning iterations from the same location have been exceeded. "
              "It is likely that the robot had a problem and is at halt! STOPPING REPLANNING...");
    return;
  }

  // Save current path idx as old path idx (i.e. current pose as old pose)
  old_path_idx = path_idx;

  // Reset the counter (NOT NEEDED ANYMORE)
  // counter = 0;

  // Take min to avoid overflow when path_idx is close to last waypoint in the trajectory
  replan_start_idx = std::min(path_idx + static_cast<size_t>((gp_replan_lookahead_sec_)/
                                                      constraints_.sampling_dt),
                              global_path.size());

  // TEST!!!!!!!!!!!!!!!!!!!===============================================
  ROS_INFO("[DEBUGGING] replan_start_idx = %d", replan_start_idx);
  mav_msgs::EigenTrajectoryPoint mytest = *(global_path.begin() + replan_start_idx);
  ROS_INFO_STREAM("TRYING TO COPY GLOBAL PATH FROM " << mytest.position_W.transpose() << " AS STARTING POINT...");
  // ===============================================================================

  std::copy(global_path.begin() + replan_start_idx, global_path.end(), 
            std::back_inserter(chunk_for_revision));

  // In case there's no more path to analyse (the goal has been reached)
  // then simply set goal pose into the chunk to avoid conflicts in later method calls
  if (chunk_for_revision.size()==0){
    chunk_for_revision.push_back(global_path.back());
  }

  // Only replan if your projected location has not reached the goal yet
  if (replan_start_idx < global_path.size()) {

    // Set replanning routine flag for the service callback
    replanning = 1;

    bool chunk_in_collision = checkPathForCollisions(chunk_for_revision, NULL);
    if (chunk_in_collision){
      ROS_INFO("Global path in collision! REPLANNING...");
      // Set start and end pose in the right form for planning 
      mav_planning_msgs::PlannerServiceRequest req;
      mav_planning_msgs::PlannerServiceResponse resp;
      mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(chunk_for_revision.front(), 
                                                        &req.start_pose);
      mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(chunk_for_revision.back(), 
                                                        &req.goal_pose);
      
      // Get global path for the new chunk
      new_gp_success = plannerServiceCallback(req, resp);
      if (new_gp_success)
      {
        // Remove old piece of trajectory that was in collision
        global_path.erase(global_path.begin() + replan_start_idx,
                          global_path.end());
        ROS_INFO("[DEBUGGING] Global path size after erasing old path = %d", global_path.size());

        // Insert new path chunk that you just replanned
        global_path.insert(global_path.end(), ramp_path.begin(), ramp_path.end());
        ROS_INFO("[DEBUGGING] Global path size after inserting new path = %d", global_path.size());

        // VERY IMPORTANT: Clear ramp_path!
        ramp_path.clear();

        // Publish new global path (waypoints only) to local planner
        std_srvs::EmptyRequest req_empty;
        std_srvs::EmptyResponse resp_empty;
        success_publish = publishPathCallback(req_empty, resp_empty);

        // If new global planning is succesful, then reset path_idx and counter
        // (implies new pose along the trajectory will be 1 gp_replan_dt away from the 
        // first waypoint of the new global path)
        // TODO: these resets might need to be set only if the publishing is succesful??
        //path_idx = 0; //(RESET NOT USED WITH THE CURRENT APPENDING METHOD FOR GP)
        // counter = 0;

        // able_to_plan = 1;
        if (!success_publish) {
          ROS_INFO("New global path could NOT be published to local planner...");
          return;
        }
        ROS_INFO("SUCCESS! new global path sent to local planner!...");
        return;
      }

      else{
        ROS_INFO("New global path could NOT be found. Continuing previous path..."
                  "Message above should tell what's the issue. If no other message is found"
                  "then RRT planner could not find a path");
        // able_to_plan = 1;
        return;
      }

    }
    ROS_INFO("Current global path is COLLISION FREE, keep rollin' with it.");
    // able_to_plan = 1;
    return;
  }
  ROS_INFO("NO MORE PATH TO REPLAN. Following trajectory planned on last" 
            " available information...");
  // able_to_plan = 1;
  // Clear replanning routine flag so that if a new call to plannerServiceCallback()
  // occurs, it must be a request from the user interface.
  replanning = 0;
  // Stop gp_timer (important to avoid overflow of path_idx)
  gp_timer_.stop();
}

bool VoxbloxRrtPlanner::plannerServiceCallback(
    mav_planning_msgs::PlannerServiceRequest& request,
    mav_planning_msgs::PlannerServiceResponse& response) {
  mav_msgs::EigenTrajectoryPoint start_pose, goal_pose;

  mav_msgs::eigenTrajectoryPointFromPoseMsg(request.start_pose, &start_pose);
  mav_msgs::eigenTrajectoryPointFromPoseMsg(request.goal_pose, &goal_pose);

  // Setup latest copy of map.
  if (!(esdf_map_ &&
        esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0) &&
      !(tsdf_map_ &&
        tsdf_map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0)) {
    ROS_ERROR("Both maps are empty!");
    return false;
  }
  // Figure out map bounds!
  computeMapBounds(&lower_bound_, &upper_bound_);

  ROS_INFO_STREAM("Map bounds: " << lower_bound_.transpose() << " to "
                                 << upper_bound_.transpose() << " size: "
                                 << (upper_bound_ - lower_bound_).transpose());

  // Inflate the bounds a bit.
  constexpr double kBoundInflationMeters = 0.5;
  // Don't in flate in z. ;)
  rrt_.setBounds(lower_bound_ - Eigen::Vector3d(kBoundInflationMeters,
                                                kBoundInflationMeters, 0.0),
                 upper_bound_ + Eigen::Vector3d(kBoundInflationMeters,
                                                kBoundInflationMeters, 0.0));

// Set OMPL objective (constant altitude or pathlenght only) based on param 
  if (constant_altitude){
    rrt_.setupMyProblem(start_pose);
    
  } else {
    rrt_.setupProblem();
  }

  ROS_INFO("[Service Callback] Planning path.");

  if (!optimistic_){
  	// Use ESDF to check starting pose
  	if (getMapDistance(start_pose.position_W) < constraints_.robot_radius) {
    	ROS_ERROR("Start pose occupied!");
    	return false;
  	}

    // Use ESDF to check goal pose
    if (getMapDistance(goal_pose.position_W) < constraints_.robot_radius) {
      ROS_INFO_STREAM("ESDF value of the goal pose is " << getMapDistance(goal_pose.position_W) << "!...");
      ROS_ERROR("Goal pose occupied!");
      return false;
    }
  } else{
  	// Use TSDF
  	if (CollisionCheckTsdf(start_pose.position_W, tsdf_map_->getTsdfLayerPtr())){
  		ROS_ERROR("Start pose occupied!");
    	return false;
  	}

    // if (CollisionCheckTsdf(goal_pose.position_W, tsdf_map_->getTsdfLayerPtr())){
    //   ROS_ERROR("Goal pose occupied!");
    //   return false;
    // }
  }

  // (OLD (HELEN) STARTING POSE COLLISION CHECK)
  // if (getMapDistance(start_pose.position_W) < constraints_.robot_radius) {
  //   ROS_ERROR("Start pose occupied!");
  //   return false;
  // }

  // COMMENT BELOW TO GET A OPTIMISTIC PLANNING (WRT ALLOWING THE GOAL POSE)
  // if (getMapDistance(goal_pose.position_W) < constraints_.robot_radius) {
  //   ROS_INFO_STREAM("ESDF value of the goal pose is " << getMapDistance(goal_pose.position_W) << "!...");
  //   ROS_ERROR("Goal pose occupied!");
  //   return false;
  // }

  mav_msgs::EigenTrajectoryPoint::Vector waypoints;
  mav_trajectory_generation::timing::Timer rrtstar_timer("plan/rrt_star");

  bool success =
      rrt_.getPathBetweenWaypoints(start_pose, goal_pose, &waypoints);
  rrtstar_timer.Stop();
  double path_length = computePathLength(waypoints);
  int num_vertices = waypoints.size();
  ROS_INFO("RRT* Success? %d Path length: %f Vertices: %d", success,
           path_length, num_vertices);

  if (!success) {
    return false;
  }

  visualization_msgs::MarkerArray marker_array;
  if (visualize_) {
    marker_array.markers.push_back(createMarkerForPath(
        waypoints, frame_id_, mav_visualization::Color::Green(), "rrt_star",
        0.075));
    marker_array.markers.push_back(createMarkerForWaypoints(
        waypoints, frame_id_, mav_visualization::Color::Green(),
        "rrt_star_waypoints", 0.15));
  }

  last_waypoints_ = waypoints;

  if (!do_smoothing_) {
    last_trajectory_valid_ = true;

    // Produce global trajectory vector ramp_path (only used for moving along the trajectory for 
    // replanning, i.e. not sent to local planner)
    generateFeasibleTrajectoryRamp(last_waypoints_, &ramp_path);
    // Save planned path as global path if the current planning call was not done
    // by the replanning routine.
    if (!replanning){
      ROS_INFO("INTERFACE REQUEST. Setting currently planned path as global path...");
      global_path = ramp_path;	// Makes a hard copy of ramp_path (i.e. not pointing to same object)
      // VERY IMPORTANT RESETS!
      path_idx = 0;
      ramp_path.clear();
    }

    // CLEAR HERE!

  } else {
    mav_msgs::EigenTrajectoryPointVector poly_path;
    mav_trajectory_generation::timing::Timer poly_timer("plan/poly");
    bool poly_has_collisions =
        !generateFeasibleTrajectory(waypoints, &poly_path);
    poly_timer.Stop();

    mav_msgs::EigenTrajectoryPointVector loco_path;
    mav_trajectory_generation::timing::Timer loco_timer("plan/loco");
    bool loco_has_collisions =
        !generateFeasibleTrajectoryLoco(waypoints, &loco_path);
    loco_timer.Stop();

    mav_msgs::EigenTrajectoryPointVector loco2_path;
    mav_trajectory_generation::timing::Timer loco2_timer("plan/loco2");
    bool loco2_has_collisions =
        !generateFeasibleTrajectoryLoco2(waypoints, &loco2_path);
    loco2_timer.Stop();

    // mav_msgs::EigenTrajectoryPointVector ramp_path;
    // mav_trajectory_generation::timing::Timer ramp_timer("plan/ramp");
    // bool ramp_has_collisions =
    //     !generateFeasibleTrajectoryRamp(waypoints, &ramp_path);
    // ramp_timer.Stop();    

    ROS_INFO(
        // "Poly Smoothed Path has collisions? %d Loco Path has collisions? %d "
        // "Loco 2 has collisions? %d Velocity Ramp has collisions? %d",
        "Poly Smoothed Path has collisions? %d Loco Path has collisions? %d "
        "Loco 2 has collisions? %d ",
        poly_has_collisions, loco_has_collisions, loco2_has_collisions);

    if (!poly_has_collisions) {
      last_trajectory_valid_ = true;
    }

    if (visualize_) {
      marker_array.markers.push_back(createMarkerForPath(
          poly_path, frame_id_, mav_visualization::Color::Orange(), "poly",
          0.075));
      marker_array.markers.push_back(
          createMarkerForPath(loco_path, frame_id_,
                              mav_visualization::Color::Pink(), "loco", 0.075));
      marker_array.markers.push_back(createMarkerForPath(
          loco2_path, frame_id_, mav_visualization::Color::Teal(), "loco2",
          0.075));
      // marker_array.markers.push_back(createMarkerForPath(
      //     ramp_path, frame_id_, mav_visualization::Color::Black(), "ramp",
      //     0.075));
    }
  }

  if (visualize_) {
    path_marker_pub_.publish(marker_array);
  }

  response.success = success;

  ROS_INFO_STREAM("All timings: "
                  << std::endl
                  << mav_trajectory_generation::timing::Timing::Print());
  ROS_INFO_STREAM("Finished planning with start point: "
                  << start_pose.position_W.transpose()
                  << " and goal point: " << goal_pose.position_W.transpose());

  // // Set timer options (especify callback queue and function) for global replanning.
  // ros::TimerOptions gp_timer_options(ros::Duration(gp_replan_dt),
  //     boost::bind(&VoxbloxRrtPlanner::gpTimerCallback, this, _1),
  //     &gp_queue_);

  // // Create timer object for replanning.
  // gp_timer_ = nh_.createTimer(gp_timer_options);

  return success;
}

bool VoxbloxRrtPlanner::generateFeasibleTrajectoryRamp(
    const mav_msgs::EigenTrajectoryPointVector& coordinate_path,
    mav_msgs::EigenTrajectoryPointVector* path) {
  ramp_smoother_.getPathBetweenWaypoints(coordinate_path, path);

  bool path_in_collision = checkPathForCollisions(*path, NULL);
  if (path_in_collision) {
    ROS_INFO("Velocity Ramp in Collision!...");
    return false;
  }
  return true;
}

bool VoxbloxRrtPlanner::generateFeasibleTrajectory(
    const mav_msgs::EigenTrajectoryPointVector& coordinate_path,
    mav_msgs::EigenTrajectoryPointVector* path) {
  smoother_.getPathBetweenWaypoints(coordinate_path, path);

  bool path_in_collision = checkPathForCollisions(*path, NULL);

  if (path_in_collision) {
    return false;
  }
  return true;
}

bool VoxbloxRrtPlanner::generateFeasibleTrajectoryLoco(
    const mav_msgs::EigenTrajectoryPointVector& coordinate_path,
    mav_msgs::EigenTrajectoryPointVector* path) {
  loco_smoother_.setResampleTrajectory(false);
  loco_smoother_.setAddWaypoints(false);

  loco_smoother_.getPathBetweenWaypoints(coordinate_path, path);

  bool path_in_collision = checkPathForCollisions(*path, NULL);

  if (path_in_collision) {
    return false;
  }
  return true;
}

bool VoxbloxRrtPlanner::generateFeasibleTrajectoryLoco2(
    const mav_msgs::EigenTrajectoryPointVector& coordinate_path,
    mav_msgs::EigenTrajectoryPointVector* path) {
  loco_smoother_.setResampleTrajectory(true);
  loco_smoother_.setAddWaypoints(false);

  loco_smoother_.getPathBetweenWaypoints(coordinate_path, path);

  bool path_in_collision = checkPathForCollisions(*path, NULL);

  if (path_in_collision) {
    return false;
  }
  return true;
}

bool VoxbloxRrtPlanner::checkPathForCollisions(
    const mav_msgs::EigenTrajectoryPointVector& path, double* t) const {
  // voxblox::Layer<voxblox::TsdfVoxel>* my_layer_ptr = voxblox_server_.getTsdfMapPtr()->getTsdfLayerPtr();
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    if(!optimistic_){
      if (getMapDistance(point.position_W) < constraints_.robot_radius) {
        if (t != NULL) {
          *t = mav_msgs::nanosecondsToSeconds(point.time_from_start_ns);
        }
        return true;
      }
    }
    else{ //IF OPTIMISTIC
      if (CollisionCheckTsdf(point.position_W,tsdf_map_->getTsdfLayerPtr()))
      {
        if (t != NULL) {
          *t = mav_msgs::nanosecondsToSeconds(point.time_from_start_ns);
        }
        return true;
      }
    }
  }
  return false;
}

double VoxbloxRrtPlanner::getMapDistance(
    const Eigen::Vector3d& position) const {
  if (!voxblox_server_.getEsdfMapPtr()) {
    ROS_INFO("THERE'S NO ESDF MAP!...");
    return 0.0;
  }
  double distance = 0.0;
  if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position,
                                                              &distance)) {
    ROS_INFO("[ESDF MAP DISTANCE VOXEL CHECK] AS SUSPECTED A DISTANCE (INTERPOLATED) COULDN'T REALLY BE COMPUTED SINCE NO VECTOR Q OR VOXELS DISTANCES WERE FOUND!... THE VALUE YOU SEE BELOW IS THE 0.0");
    return 0.0;
  }
  ROS_INFO("[ESDF MAP DISTANCE VOXEL CHECK] A DISTANCE IS COMPUTED AND THE VALUE YOU GET IS THE RESULTING COMPUTATION!...");
  return distance;

  // else{   // Optimistic planner uses TSDF layer
  //   if (!voxblox_server_.getTsdfMapPtr())
  //   {
  //     ROS_INFO("THERE'S NO TSDF MAP!...");
  //     return 0.0;
  //   }
  //   double distance = 0.0;
  //   // TODO: Define getDistanceAtPosition method for TSDF Map (if doesn't exists)
  //   if (!voxblox_server_.getTsdfMapPtr()->getDistanceAtPosition(position,
  //                                                               &distance)){
  //     ROS_INFO("[TSDF MAP DISTANCE VOXEL CHECK] AS SUSPECTED A DISTANCE (INTERPOLATED) COULDN'T REALLY BE COMPUTED SINCE NO VECTOR Q OR VOXELS DISTANCES WERE FOUND!... THE VALUE YOU SEE BELOW IS THE 0.0");
  //     return 0.0;
  //   }
  //   ROS_INFO("[TSDF MAP DISTANCE VOXEL CHECK] A DISTANCE IS COMPUTED AND THE VALUE YOU GET IS THE RESULTING COMPUTATION!...");
  //   return distance;
  // }
}

bool VoxbloxRrtPlanner::checkPhysicalConstraints(
    const mav_trajectory_generation::Trajectory& trajectory) {
  // Check min/max manually.
  // Evaluate min/max extrema
  std::vector<int> dimensions = {0, 1, 2};  // Evaluate dimensions in x, y and z
  mav_trajectory_generation::Extremum v_min, v_max, a_min, a_max;
  trajectory.computeMinMaxMagnitude(
      mav_trajectory_generation::derivative_order::VELOCITY, dimensions, &v_min,
      &v_max);
  trajectory.computeMinMaxMagnitude(
      mav_trajectory_generation::derivative_order::ACCELERATION, dimensions,
      &a_min, &a_max);

  ROS_INFO("V min/max: %f/%f, A min/max: %f/%f", v_min.value, v_max.value,
           a_min.value, a_max.value);

  // Create input constraints.
  // TODO(helenol): just store these as members...
  typedef mav_trajectory_generation::InputConstraintType ICT;
  mav_trajectory_generation::InputConstraints input_constraints;
  input_constraints.addConstraint(
      ICT::kFMin, (mav_msgs::kGravity -
                   constraints_.a_max));  // maximum acceleration in [m/s/s].
  input_constraints.addConstraint(
      ICT::kFMax, (mav_msgs::kGravity +
                   constraints_.a_max));  // maximum acceleration in [m/s/s].
  input_constraints.addConstraint(
      ICT::kVMax, constraints_.v_max);  // maximum velocity in [m/s].

  // Create feasibility object of choice (FeasibilityAnalytic,
  // FeasibilitySampling, FeasibilityRecursive).
  mav_trajectory_generation::FeasibilityAnalytic feasibility_check(
      input_constraints);
  feasibility_check.settings_.setMinSectionTimeS(0.01);

  mav_trajectory_generation::InputFeasibilityResult feasibility =
      feasibility_check.checkInputFeasibilityTrajectory(trajectory);
  if (feasibility !=
      mav_trajectory_generation::InputFeasibilityResult::kInputFeasible) {
    ROS_ERROR_STREAM(
        "Trajectory is input infeasible: "
        << mav_trajectory_generation::getInputFeasibilityResultName(
            feasibility));
    return false;
  }
  return true;
}

bool VoxbloxRrtPlanner::CollisionCheckTsdf (const Eigen::Vector3d& robot_position, 
                          voxblox::Layer<voxblox::TsdfVoxel>* my_tsdf_layer) const {
  voxblox::Point robot_point = robot_position.cast<voxblox::FloatingPoint>();
  voxblox::HierarchicalIndexMap block_voxel_list;
  voxblox::utils::getSphereAroundPoint(*my_tsdf_layer, robot_point, constraints_.robot_radius,
                                        &block_voxel_list);

  for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList>& kv :
         block_voxel_list) {

    voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr = my_tsdf_layer->getBlockPtrByIndex(kv.first);
    
    if (!block_ptr)
    {
      continue;
    }

    for (const voxblox::VoxelIndex& voxel_index : kv.second){
      if(!block_ptr->isValidVoxelIndex(voxel_index)){
        if (!optimistic_){
          return true;
        }
        continue;
      }
      const voxblox::TsdfVoxel& tsdf_voxel = block_ptr->getVoxelByVoxelIndex(voxel_index);
      if (tsdf_voxel.weight < voxblox::kEpsilon){
        if(!optimistic_){
          return true;
        }
        continue;
      }
      if (tsdf_voxel.distance <= 0.0f){
        return true;
      }
    }
  }

    // No collision if nothing in the sphere had a negative or 0 distance.
    // Unknown space is unoccupied, since this is a very optimistic global
    // planner.
  return false;
}

}  // namespace mav_planning
