#include <mav_msgs/default_topics.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include "mav_local_planner/mav_local_planner.h"

namespace mav_planning {

MavLocalPlanner::MavLocalPlanner(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      command_publishing_spinner_(1, &command_publishing_queue_),
      planning_spinner_(1, &planning_queue_),
      verbose_(false),
      //global_frame_id_("map"),
      global_frame_id_("map"),
      local_frame_id_("odom"),
      mpc_prediction_horizon_(300),
      command_publishing_dt_(1.0),
      replan_dt_(1.0),
      replan_lookahead_sec_(0.1),
      avoid_collisions_(true),
      autostart_(true),
      plan_to_start_(true),
      smoother_name_("loco"),
      current_waypoint_(-1),
      path_index_(0),
      max_failures_(5), // Specifies how many times whole planning process is repeated for a waypoint in case of failures
      num_failures_(0),
      abort_executed_(false),
      recovery_mode_enabled_(false),
      fully_reached_waypoint_(true),
      counter_for_recovery_(0),
      done_once_(false),
      esdf_server_(nh_, nh_private_),
      loco_planner_(nh_, nh_private_) {
  // Set up some settings.
  constraints_.setParametersFromRos(nh_private_);
  esdf_server_.setTraversabilityRadius(constraints_.robot_radius);
  loco_planner_.setEsdfMap(esdf_server_.getEsdfMapPtr());
  goal_selector_.setParametersFromRos(nh_private_);
  goal_selector_.setTsdfMap(esdf_server_.getTsdfMapPtr());

  nh_private_.param("verbose", verbose_, verbose_);
  nh_private_.param("global_frame_id", global_frame_id_, global_frame_id_);
  nh_private_.param("local_frame_id", local_frame_id_, local_frame_id_);
  nh_private_.param("mpc_prediction_horizon", mpc_prediction_horizon_,
                    mpc_prediction_horizon_);
  nh_private_.param("replan_dt", replan_dt_, replan_dt_);
  nh_private_.param("replan_lookahead_sec", replan_lookahead_sec_,
                    replan_lookahead_sec_);
  nh_private_.param("command_publishing_dt", command_publishing_dt_,
                    command_publishing_dt_);
  nh_private_.param("avoid_collisions", avoid_collisions_, avoid_collisions_);
  nh_private_.param("autostart", autostart_, autostart_);
  nh_private_.param("plan_to_start", plan_to_start_, plan_to_start_);
  nh_private_.param("smoother_name", smoother_name_, smoother_name_);
  nh_private_.param("recovery_mode_enabled_", recovery_mode_enabled_, recovery_mode_enabled_);

  // Publishers and subscribers.
  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                &MavLocalPlanner::odometryCallback, this);
  waypoint_sub_ =
      nh_.subscribe("waypoint", 1, &MavLocalPlanner::waypointCallback, this);
  waypoint_list_sub_ = nh_.subscribe(
      "waypoint_list", 1, &MavLocalPlanner::waypointListCallback, this);

  command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);

  path_marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "local_path", 1, true);
  full_trajectory_pub_ =
      nh_private_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          "full_trajectory", 1, true);

  // Services.
  start_srv_ = nh_private_.advertiseService(
      "start", &MavLocalPlanner::startCallback, this);
  pause_srv_ = nh_private_.advertiseService(
      "pause", &MavLocalPlanner::pauseCallback, this);
  stop_srv_ = nh_private_.advertiseService(
      "stop", &MavLocalPlanner::stopCallback, this);

  position_hold_client_ =
      nh_.serviceClient<std_srvs::Empty>("back_to_position_hold");

  // Start the planning timer. Will no-op most cycles.
  ros::TimerOptions timer_options(
      ros::Duration(replan_dt_),
      boost::bind(&MavLocalPlanner::planningTimerCallback, this, _1),
      &planning_queue_);

  planning_timer_ = nh_.createTimer(timer_options);

  // Start the command publishing spinner.
  command_publishing_spinner_.start();
  planning_spinner_.start();

  // Set up yaw policy.
  yaw_policy_.setPhysicalConstraints(constraints_);
  yaw_policy_.setYawPolicy(YawPolicy::PolicyType::kVelocityVector);

  // Set up recovery yaw policy
  recovery_yaw_policy_.setPhysicalConstraints(constraints_);
  recovery_yaw_policy_.setYawPolicy(YawPolicy::PolicyType::kFromPlan);

  // Set up smoothers.
  const double voxel_size = esdf_server_.getEsdfMapPtr()->voxel_size();

  // Straight-line smoother.
  ramp_smoother_.setParametersFromRos(nh_private_);

  // Poly smoother.
  poly_smoother_.setParametersFromRos(nh_private_);
  poly_smoother_.setMinCollisionCheckResolution(voxel_size);
  poly_smoother_.setMapDistanceCallback(
      std::bind(&MavLocalPlanner::getMapDistance, this, std::placeholders::_1));
  poly_smoother_.setOptimizeTime(true);
  poly_smoother_.setSplitAtCollisions(avoid_collisions_);

  // Loco smoother!
  loco_smoother_.setParametersFromRos(nh_private_);
  loco_smoother_.setMinCollisionCheckResolution(voxel_size);
  loco_smoother_.setDistanceAndGradientFunction(
      std::bind(&MavLocalPlanner::getMapDistanceAndGradient, this,
                std::placeholders::_1, std::placeholders::_2));
  loco_smoother_.setOptimizeTime(true);
  loco_smoother_.setResampleTrajectory(true);
  loco_smoother_.setResampleVisibility(true);
  loco_smoother_.setNumSegments(6); //DEfault value = 5
}

void MavLocalPlanner::odometryCallback(const nav_msgs::Odometry& msg) {
  mav_msgs::eigenOdometryFromMsg(msg, &odometry_);
}

void MavLocalPlanner::waypointCallback(const geometry_msgs::PoseStamped& msg) {
  // Plan a path from the current position to the target pose stamped.
  ROS_INFO("[Mav Local Planner] Got a waypoint!");
  // Cancel any previous trajectory on getting a new one.
  clearTrajectory();

  mav_msgs::EigenTrajectoryPoint waypoint;
  eigenTrajectoryPointFromPoseMsg(msg, &waypoint);

  waypoints_.clear();
  waypoints_.push_back(waypoint);
  current_waypoint_ = 0;

  // Execute one planning step on main thread.
  planningStep();
  startPublishingCommands();
}

void MavLocalPlanner::waypointListCallback(
    const geometry_msgs::PoseArray& msg) {
  // Plan a path from the current position to the target pose stamped.
  ROS_INFO("[Mav Local Planner] Got a list of waypoints, %zu long!",
           msg.poses.size());
  // Cancel any previous trajectory on getting a new one.
  clearTrajectory();

  waypoints_.clear();

  for (const geometry_msgs::Pose& pose : msg.poses) {
    mav_msgs::EigenTrajectoryPoint waypoint;
    eigenTrajectoryPointFromPoseMsg(pose, &waypoint);
    waypoints_.push_back(waypoint);
  }
  current_waypoint_ = 0;

  // Execute one planning step on main thread.
  planningStep();
  startPublishingCommands();
}

void MavLocalPlanner::planningTimerCallback(const ros::TimerEvent& event) {

  // ROS_INFO("Planning Timer CALLBACK!...");

  if (recovery_mode_enabled_) {
      if (!fully_reached_waypoint_){
        counter_for_recovery_ ++;
        if (counter_for_recovery_ >= 4){
          RecoveryMode();
          counter_for_recovery_ = 0;
          // return; // Added so that current planning step is finished and you can start a fresh step to the actual waypoint you were supposed to go
        }
      } else {
        counter_for_recovery_ = 0;
      }
  }

  // Wait on the condition variable from the publishing...
  if (should_replan_.wait_for(replan_dt_)) {
    if (verbose_) {
      ROS_WARN(
          "[Mav Planning Timer] Difference between real and expected: %f Real: "
          "%f Expected: %f Now: %f",
          (event.current_real - event.current_expected).toSec(),
          event.current_real.toSec(), event.current_expected.toSec(),
          ros::Time::now().toSec());
    }

    planningStep();
  }
}

void MavLocalPlanner::planningStep() {
  ROS_INFO(
      "[Mav Local Planner][Plan Step] Waypoint index: %zd Total waypoints: %zu",
      current_waypoint_, waypoints_.size());
  if (current_waypoint_ < 0 ||
      static_cast<int>(waypoints_.size()) <= current_waypoint_) {
    // This means that we probably planned to the end of the waypoints!

    // If we're done with sending waypoints, alllll good. Just quit.
    if (path_index_ >= path_queue_.size() || path_queue_.empty()) {
      return;
    }

    // If we're not though, we should probably double check the trajectory!
  }

  mav_trajectory_generation::timing::MiniTimer timer;
  constexpr double kCloseToOdometry = 0.1;

  // First, easiest case: if we're not avoiding collisions, just use the
  // favorite path smoother. We only do this on the first planning call then
  // ignore all the rest.
  if (!avoid_collisions_) {
    mav_msgs::EigenTrajectoryPointVector waypoints;
    mav_msgs::EigenTrajectoryPoint current_point;
    current_point.position_W = odometry_.position_W;
    current_point.orientation_W_B = odometry_.orientation_W_B;

    if (plan_to_start_) {
      waypoints.push_back(current_point);
    }
    waypoints.insert(waypoints.end(), waypoints_.begin(), waypoints_.end());

    mav_msgs::EigenTrajectoryPointVector path;

    if (planPathThroughWaypoints(waypoints, &path)) {
      replacePath(path);
      current_waypoint_ = waypoints_.size();
    } else {
      ROS_ERROR("[Mav Local Planner] Waypoint planning failed!");
    }
  } else if (path_queue_.empty()) {
    // First check how many waypoints we haven't covered yet are in free space.
    mav_msgs::EigenTrajectoryPointVector free_waypoints;
    // Do we need the odometry in here? Let's see.
    mav_msgs::EigenTrajectoryPoint current_point;
    current_point.position_W = odometry_.position_W;
    current_point.orientation_W_B = odometry_.orientation_W_B;

    // If the path doesn't ALREADY start near the odometry, the first waypoint
    // should be the current pose.

    // THIS IS WHAT MAKES IT COME TO THE FIRST WAYPOINT IN THE CASE OF FAILURE IN THE WAY TO THE GOAL
    int waypoints_added = 0;
    if (plan_to_start_ &&
        (current_point.position_W - waypoints_.front().position_W).norm() >
            kCloseToOdometry) {
      free_waypoints.push_back(current_point);
      waypoints_added = 1;
    }

    for (const mav_msgs::EigenTrajectoryPoint& waypoint : waypoints_) {
      if (getMapDistance(waypoint.position_W) < constraints_.robot_radius) {
        break;
      }
      free_waypoints.push_back(waypoint);
    }

    ROS_INFO("[Mav Local Planner] Of %zu waypoints, %zu are free.",
             waypoints_.size(), free_waypoints.size());
    bool success = false;
    if (free_waypoints.size() <= static_cast<size_t>(waypoints_added) ||
        free_waypoints.size() == 2) {
      // Okay whatever just search for the first waypoint.
      success = false;
    } else {
      // There is some hope! Maybe we can do path smoothing on these guys.
      mav_msgs::EigenTrajectoryPointVector path;
      success = planPathThroughWaypoints(free_waypoints, &path);
      if (success) {
        ROS_INFO(
            "[Mav Local Planner]  Successfully planned path through %zu free "
            "waypoints.",
            free_waypoints.size());
        success = isPathCollisionFree(path);
        if (success) {
          replacePath(path);
          current_waypoint_ = std::min(free_waypoints.size() - waypoints_added,
                                       waypoints_.size() - 1);
          ROS_INFO(
              "[Mav Local Planner] Used smoothing through %zu waypoints! Total "
              "waypoint size: %zu, current point: %zd, added? %d",
              free_waypoints.size(), waypoints_.size(), current_waypoint_,
              waypoints_added);
        } else {
          ROS_WARN("[Mav Local Planner] But path was not collision free. :(");
        }
      }
    }
    // Give up!
    if (!success) {
      avoidCollisionsTowardWaypoint();
    }
  } else {
    // Otherwise let's just keep exploring.
    avoidCollisionsTowardWaypoint();
  }

  ROS_INFO("[Mav Local Planner][Plan Step] Planning finished. Time taken: %f",
           timer.stop());
  visualizePath();
}

void MavLocalPlanner::avoidCollisionsTowardWaypoint() {
  if (current_waypoint_ >= static_cast<int64_t>(waypoints_.size())) {
    return;
  }
  mav_msgs::EigenTrajectoryPoint waypoint = waypoints_[current_waypoint_];
  const double kCloseEnough = 0.05;

  const int64_t kDtNs =
      mav_msgs::secondsToNanoseconds(constraints_.sampling_dt);

  ROS_INFO_STREAM("[Mav Local Planner][Plan Step] Current odometry: "
                  << odometry_.position_W.transpose() << " Tracking waypoint ["
                  << current_waypoint_
                  << "]: " << waypoint.position_W.transpose());

  // Save success and Trajectory.
  mav_trajectory_generation::Trajectory trajectory;
  bool success = false;
  double waypoint_heading_angle;
  double ro;
  double pitch;
  double yw;
  // size_t number_to_substract = static_cast<size_t>(2);

  if (!path_queue_.empty()) {
  // if(false) {
    std::lock_guard<std::recursive_mutex> guard(path_mutex_);

    ROS_INFO(
        "[Mav Local Planner][Plan Step] Trying to replan on existing path.");
    mav_msgs::EigenTrajectoryPointVector path_chunk;
    size_t replan_start_index;
    {
      replan_start_index =
          std::min(path_index_ + static_cast<size_t>((replan_lookahead_sec_) /
                                                     constraints_.sampling_dt),
                   path_queue_.size());
      // ROS_INFO(
      //     "[Mav Local Planner][Plan Step] Current path index: %zu Replan start "
      //     "index: %zu",
      //     path_index_, replan_start_index);
      // Cut out the remaining snippet of the trajectory so we can do
      // something with it.
      std::copy(path_queue_.begin() + replan_start_index, path_queue_.end(),
                std::back_inserter(path_chunk));
      if (path_chunk.size() == 0) {
        path_chunk.push_back(path_queue_.back());
        // Remove the below condition or only do it if for a number of times you have only been rotating and can't plan forward...
        // if (!nextWaypoint()) {
          // finishWaypoints();
        // }
      }
    }

    bool path_chunk_collision_free = isPathCollisionFree(path_chunk);
    ROS_INFO(
        "[Mav Local Planner][Plan Step] Existing chunk is collision free? %d",
        path_chunk_collision_free);
    // Check if the current path queue goes to the goal and is collision free.
    if ((path_chunk.back().position_W - waypoint.position_W).norm() <
        kCloseEnough) {
      // Collision check the remaining chunk of the trajectory.
      if (path_chunk_collision_free) {
        ROS_INFO(
            "[Mav Local Planner][Plan Step] Current plan is valid, just "
            "rollin' with it.");
        fully_reached_waypoint_ = 1;
        // ONlY PLAN FOR THE NEXT WAYPOINT IF YOU ARE CLOSE ENOUGH
        if ((odometry_.position_W - waypoint.position_W).norm() < 0.5){
          bool next_point = nextWaypoint();
          // // Only problem with this is that it doesn't exactly get to the intermediate waypoints
          // // PLUS it does start planning from the beggining (smoothing) as there is path_queue_ is empty
          // // iin planning_step() loop
          // // TRY with flags to know in which time (first or replanning) you are atm
          // if (next_point) {
          //   path_queue_.clear();
          //   path_index_ = 0;
          // }

        }
        // nextWaypoint();
        return;
      }
    }

    // if robot orientation close to desired heading proceed with replanning
    // else return (repeat and try again next iteration)
    toEulerAngle(odometry_.orientation_W_B, ro, pitch, yw);
    double desired_heading = compute_heading(waypoint);
    // double yaw_difference = computeYawDifference(yw, desired_heading);
    if (std::abs(yw - desired_heading) < 25*(M_PI/180)) {

      // Otherwise we gotta replan this thing anyway.
      success = loco_planner_.getTrajectoryTowardGoal(path_chunk.front(),
                                                      waypoint, &trajectory);
      if (!success) {
        fully_reached_waypoint_ = 0;
        if (path_chunk_collision_free) {
          ROS_INFO(
              "[Mav Local Planner][Plan Step] Couldn't find a solution :( "
              "Continuing existing solution.");
          // TODO: The problem I thought of this routine happing more than 4 times because the waypoint
          // is too far can be avoided if the branch lenght parameter is reduced (what Luca recommended) 
        } else {
          ROS_INFO(
              "[Mav Local Planner][Plan Step] ABORTING! No local solution "
              "found.");
          // TODO(helenol): which order to abort in?
          abort();
          dealWithFailure();
        }
        return;
      } else {
        ROS_INFO("[Mav Local Planner][Plan Step] Appending new path chunk.");
        fully_reached_waypoint_ = 1;
        // Reduce the number below to ensure the MAV gets closer to the current waypoint it is 
        // flying towards before planning to the next one
        if (trajectory.getMaxTime() <= 1e-6) {
          if ((odometry_.position_W - waypoint.position_W).norm() < 0.5){
            nextWaypoint();
          }
          else {
            faceToWaypoint(waypoint, waypoint_heading_angle);
          }
          // nextWaypoint();
        } else {
          num_failures_ = 0;
          mav_msgs::EigenTrajectoryPointVector new_path_chunk;
          mav_trajectory_generation::sampleWholeTrajectory(
              trajectory, constraints_.sampling_dt, &new_path_chunk);

          retimeTrajectoryWithStartTimeAndDt(
              path_chunk.front().time_from_start_ns, kDtNs, &new_path_chunk);

          new_path_chunk.front().orientation_W_B =
              path_chunk.front().orientation_W_B;
          yaw_policy_.applyPolicyInPlace(&new_path_chunk);

          // Remove what was in the trajectory before.
          if (replan_start_index < path_queue_.size()) {
            path_queue_.erase(path_queue_.begin() + replan_start_index,
                              path_queue_.end());
          }
          // Stick the new one in.
          path_queue_.insert(path_queue_.end(), new_path_chunk.begin(),
                             new_path_chunk.end());
        }
      }
    } else {
      // if (current_waypoint_ <= 1){
      faceToWaypoint(waypoint, waypoint_heading_angle);
      // }
      // path_index_ -= number_to_substract;
      // NO NEED ANYMORE TO INCREASE PATH QUEUE AS faceToWaypoint() is now increasing it
      // without problem
      // increasePathQueue();

    }
  } else {
    ROS_INFO("[Mav Local Planner][Plan Step] Trying to plan from scratch.");

    // There's nothing planned so far! So we plan from the current odometry.
    mav_msgs::EigenTrajectoryPoint current_point;
    current_point.position_W = odometry_.position_W;
    current_point.orientation_W_B = odometry_.orientation_W_B;

    // Check if the current waypoint is basically the odometry.
    if ((current_point.position_W - waypoint.position_W).norm() <
        kCloseEnough) {
      if (nextWaypoint()) {
        waypoint = waypoints_[current_waypoint_];
      } else {
        return;
      }
    }

    // if (!done_once_){
    //   //Correct heading before you plan to the waypoint
    //   faceToWaypoint(waypoint, waypoint_heading_angle);
    //   done_once_ = 1;
    //   // startPublishingCommands();
    //   toEulerAngle(current_point.orientation_W_B,ro,pitch,yw);
    // }
    //Correct heading before you plan to the waypoint
    faceToWaypoint(waypoint, waypoint_heading_angle);
    // done_once_ = 1;
    // startPublishingCommands();
    toEulerAngle(current_point.orientation_W_B,ro,pitch,yw);

    // if (false) {

    if (std::abs(yw - waypoint_heading_angle) < 10*(M_PI/180) ){// abs(robot orientation - goal orientation) <= 8degrees
      ROS_INFO("[Mav Local Planner][Plan Step] LOCO planner execution!...");
      // then allow to plan to new waypoint
      // else return and repeat process to the same waypoint

      success = loco_planner_.getTrajectoryTowardGoal(current_point, waypoint,
                                                      &trajectory);
      ROS_INFO("[Mav Local Planner][Plan Step] Planning success? %d", success);

      if (success) {
        if (trajectory.getMaxTime() <= 0.1) {
          nextWaypoint();
        } else {
          // Copy this straight into the queue.
          num_failures_ = 0;
          mav_msgs::EigenTrajectoryPointVector path;
          mav_trajectory_generation::sampleWholeTrajectory(
              trajectory, constraints_.sampling_dt, &path);
          replacePath(path);
        }
      } else {
  // <<<<<<< HEAD
        // num_failures_++;
        // if (num_failures_ > max_failures_) {
          // current_waypoint_ = -1;
        // }
        
        // ADDED PLOTTING ALSO FOR NON SUCCESFUL PATH
        // ROS_INFO("PLOTTING NON COLLISION-FREE PATH...");
        // mav_msgs::EigenTrajectoryPointVector path;
        // mav_trajectory_generation::sampleWholeTrajectory(trajectory, constraints_.sampling_dt, &path);
        // replacePath(path);
  // =======
        // fully_reached_waypoint_ = 0;
        dealWithFailure();
  // >>>>>>> master
      }
    }
  }
}

bool MavLocalPlanner::planPathThroughWaypoints(
    const mav_msgs::EigenTrajectoryPointVector& waypoints,
    mav_msgs::EigenTrajectoryPointVector* path) {
  CHECK_NOTNULL(path);
  bool success = false;
  if (smoother_name_ == "loco") {
    if (waypoints.size() == 2) {
      success = loco_smoother_.getPathBetweenTwoPoints(waypoints[0],
                                                       waypoints[1], path);
    } else {
      success = loco_smoother_.getPathBetweenWaypoints(waypoints, path);
    }
  } else if (smoother_name_ == "polynomial") {
    success = poly_smoother_.getPathBetweenWaypoints(waypoints, path);

  } else if (smoother_name_ == "ramp") {
    success = ramp_smoother_.getPathBetweenWaypoints(waypoints, path);
  } else {
    // Default case is ramp!
    ROS_ERROR(
        "[Mav Local Planner] Unknown smoother type %s, using ramp instead.",
        smoother_name_.c_str());
    success = ramp_smoother_.getPathBetweenWaypoints(waypoints, path);
  }
  return success;
}

bool MavLocalPlanner::nextWaypoint() {
  if (current_waypoint_ >= static_cast<int64_t>(waypoints_.size()) - 1) {
    // WHY THERE'S NO TYPE CASTING NEEDED FOR THE .size() BELOW???
    current_waypoint_ = waypoints_.size() - 1;
    return false;
  } else {
    current_waypoint_++;
    return true;
  }
}

void MavLocalPlanner::finishWaypoints() {
  current_waypoint_ = waypoints_.size()-1;
}

void MavLocalPlanner::replacePath(
    const mav_msgs::EigenTrajectoryPointVector& path) {
  std::lock_guard<std::recursive_mutex> guard(path_mutex_);
  path_queue_.clear();
  path_queue_ = path;
  path_queue_.front().orientation_W_B = odometry_.orientation_W_B;
  yaw_policy_.applyPolicyInPlace(&path_queue_);
  path_index_ = 0;
}

void MavLocalPlanner::startPublishingCommands() {
  // Call the service call to takeover publishing commands.
  if (position_hold_client_.exists()) {
    std_srvs::Empty empty_call;
    position_hold_client_.call(empty_call);
  }

  // Publish the first set immediately, on this thread.
  commandPublishTimerCallback(ros::TimerEvent());

  // Need advanced timer options to assign callback queue to this timer.
  ros::TimerOptions timer_options(
      ros::Duration(command_publishing_dt_),
      boost::bind(&MavLocalPlanner::commandPublishTimerCallback, this, _1),
      &command_publishing_queue_);

  command_publishing_timer_ = nh_.createTimer(timer_options);
}

void MavLocalPlanner::commandPublishTimerCallback(
    const ros::TimerEvent& event) {
  constexpr size_t kQueueBuffer = 0;
  if (path_index_ < path_queue_.size()) {
    std::lock_guard<std::recursive_mutex> guard(path_mutex_);
    size_t number_to_publish = std::min<size_t>(
        std::floor(command_publishing_dt_ / constraints_.sampling_dt),
        path_queue_.size() - path_index_);

    size_t starting_index = 0;
    if (path_index_ != 0) {
      starting_index = path_index_ + kQueueBuffer;
      if (starting_index >= path_queue_.size()) {
        starting_index = path_index_;
      }
    }

    size_t number_to_publish_with_buffer = std::min<size_t>(
        number_to_publish + mpc_prediction_horizon_ - kQueueBuffer,
        path_queue_.size() - starting_index);

    // TODO(helenol): do this without copy! Use iterators properly!
    mav_msgs::EigenTrajectoryPointVector::const_iterator first_sample =
        path_queue_.begin() + starting_index;
    mav_msgs::EigenTrajectoryPointVector::const_iterator last_sample =
        first_sample + number_to_publish_with_buffer;
    mav_msgs::EigenTrajectoryPointVector trajectory_to_publish(first_sample,
                                                               last_sample);

    trajectory_msgs::MultiDOFJointTrajectory msg;
    msg.header.frame_id = local_frame_id_;
    msg.header.stamp = ros::Time::now();

    ROS_INFO(
        "[Mav Local Planner][Command Publish] Publishing %zu samples of %zu. "
        "Start index: %zu Time: %f Start position: %f Start velocity: %f End "
        "time: %f End position: %f",
        trajectory_to_publish.size(), path_queue_.size(), starting_index,
        trajectory_to_publish.front().time_from_start_ns * 1.0e-9,
        trajectory_to_publish.front().position_W.x(),
        trajectory_to_publish.front().velocity_W.x(),
        trajectory_to_publish.back().time_from_start_ns * 1.0e-9,
        trajectory_to_publish.back().position_W.x());
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_to_publish, &msg);

    command_pub_.publish(msg);
    path_index_ += number_to_publish;
    should_replan_.notify();
  }
  // Does there need to be an else????
}

void MavLocalPlanner::abort() {
  // No need to check anything on stop, just clear all the paths.
  clearTrajectory();
  // Make sure to clear the queue in the controller as well (we send about a
  // second of trajectories ahead).
  sendCurrentPose();
  // Set abort flag
  abort_executed_ = 1;
}

void MavLocalPlanner::clearTrajectory() {
  std::lock_guard<std::recursive_mutex> guard(path_mutex_);
  command_publishing_timer_.stop();
  path_queue_.clear();
  path_index_ = 0;
}

void MavLocalPlanner::sendCurrentPose() {
  // Sends the current pose with velocity 0 to the controller to clear the
  // controller's trajectory queue.
  // More or less an abort operation.
  mav_msgs::EigenTrajectoryPoint current_point;
  current_point.position_W = odometry_.position_W;
  current_point.orientation_W_B = odometry_.orientation_W_B;

  trajectory_msgs::MultiDOFJointTrajectory msg;
  msgMultiDofJointTrajectoryFromEigen(current_point, &msg);
  command_pub_.publish(msg);
}

bool MavLocalPlanner::startCallback(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response) {
  if (path_queue_.size() <= path_index_) {
    ROS_WARN("Trying to start an empty or finished trajectory queue!");
    return false;
  }
  startPublishingCommands();
  return true;
}

bool MavLocalPlanner::pauseCallback(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response) {
  if (path_queue_.size() <= path_index_) {
    ROS_WARN("Trying to pause an empty or finished trajectory queue!");
    return false;
  }
  command_publishing_timer_.stop();
  return true;
}

bool MavLocalPlanner::stopCallback(std_srvs::Empty::Request& request,
                                   std_srvs::Empty::Response& response) {
  abort();
  return true;
}

void MavLocalPlanner::visualizePath() {
  // TODO: Split trajectory into two chunks: before and after.
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker path_marker;
  {
    std::lock_guard<std::recursive_mutex> guard(path_mutex_);

    path_marker = createMarkerForPath(path_queue_, local_frame_id_,
                                      mav_visualization::Color::Teal(),
                                      "local_path", 0.05);
  }
  marker_array.markers.push_back(path_marker);
  path_marker_pub_.publish(marker_array);
}

double MavLocalPlanner::getMapDistance(const Eigen::Vector3d& position) const {
  double distance = 0.0;
  const bool kInterpolate = false;
  if (!esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(
          position, kInterpolate, &distance)) {
    return 0.0;
  }
  return distance;
}

double MavLocalPlanner::getMapDistanceAndGradient(
    const Eigen::Vector3d& position, Eigen::Vector3d* gradient) const {
  double distance = 0.0;
  const bool kInterpolate = false;
  if (!esdf_server_.getEsdfMapPtr()->getDistanceAndGradientAtPosition(
          position, kInterpolate, &distance, gradient)) {
    return 0.0;
  }
  return distance;
}

bool MavLocalPlanner::isPathCollisionFree(
    const mav_msgs::EigenTrajectoryPointVector& path) const {
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    if (getMapDistance(point.position_W) < constraints_.robot_radius - 0.1) {
      return false;
    }
  }
  return true;
}

bool MavLocalPlanner::isPathFeasible(
    const mav_msgs::EigenTrajectoryPointVector& path) const {
  // This is easier to check in the trajectory but then we are limited in how
  // we do the smoothing.
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    if (point.acceleration_W.norm() > constraints_.a_max + 1e-2) {
      return false;
    }
    if (point.velocity_W.norm() > constraints_.v_max + 1e-2) {
      return false;
    }
  }
  return true;
}

bool MavLocalPlanner::dealWithFailure() {
  if (current_waypoint_ < 0) {
    return false;
  }

  constexpr double kCloseEnough = 0.05;
  mav_msgs::EigenTrajectoryPoint waypoint = waypoints_[current_waypoint_];
  mav_msgs::EigenTrajectoryPoint goal = waypoint;
  if (temporary_goal_ &&
      static_cast<int64_t>(waypoints_.size()) > current_waypoint_ + 1) {
    goal = waypoints_[current_waypoint_ + 1];
  }
  mav_msgs::EigenTrajectoryPoint current_point;
  current_point.position_W = odometry_.position_W;
  current_point.orientation_W_B = odometry_.orientation_W_B;

  mav_msgs::EigenTrajectoryPoint current_goal;
  if (!goal_selector_.selectNextGoal(goal, waypoint, current_point,
                                     &current_goal)) {
    num_failures_++;
    ROS_INFO("[Mav Local Planner][Dealing with Failure] CURRENT NUMBER OF FAILURES = %d",
              num_failures_);
    if (num_failures_ > max_failures_) {
      ROS_INFO("[Mav Local Planner][Dealing with Failure] Max number of failures reached!...");
      current_waypoint_ = -1;
    }
    return false;
  } else {
    if ((current_goal.position_W - waypoint.position_W).norm() < kCloseEnough) {
      // Goal is unchanged. :(
      ROS_INFO("[Mav Local Planner][Dealing with Failure] MAV tried to found a new goal "
                "but this goal is the same as the current waypoint you're tracking!...");
      temporary_goal_ = false;
      return false;
    } else if ((current_goal.position_W - goal.position_W).norm() <
               kCloseEnough) {
      // This is just the next waypoint that we're trying to go to.
      ROS_INFO("[Mav Local Planner][Dealing with Failure] MAV found a new goal but this is "
                "just the same as the next waypoint from the current one you are tracking...");
      current_waypoint_++;
      temporary_goal_ = false;
      return true;
    } else {
      // Then this is something different!
      ROS_INFO("[Mav Local Planner][Dealing with Failure] MAV found a new (intermediate) goal."
                "Appending it to the list of waypoints!...");
      temporary_goal_ = true;
      waypoints_.insert(waypoints_.begin() + current_waypoint_, current_goal);
      return true;
    }
  }
}

void MavLocalPlanner::RecoveryMode(){
  // Vector containing initial and final trajectory points (same position, different orientations)
  mav_msgs::EigenTrajectoryPointVector yaw_path_on_spot;
  // Intial and final trajectory point
  mav_msgs::EigenTrajectoryPoint initial_point;
  mav_msgs::EigenTrajectoryPoint desired_point;
  // Waypoint you were supposed to reach
  mav_msgs::EigenTrajectoryPoint waypoint = waypoints_[current_waypoint_];
  // Set waypoint as point you wish to face/look at
  recovery_yaw_policy_.setFacingPoint(waypoint.position_W);
  // Set position and orientation for initial point from odometry
  initial_point.position_W = odometry_.position_W;
  initial_point.orientation_W_B = odometry_.orientation_W_B;
  yaw_path_on_spot.push_back(initial_point);
  //Set position for final point (also from odometry as rotation in place is desired)
  desired_point.position_W = odometry_.position_W;
  yaw_path_on_spot.push_back(desired_point);

  // Apply yaw policy to the 'path_on_spot' to get yaw interpolation for the path
  recovery_yaw_policy_.applyPolicyInPlace(&yaw_path_on_spot);

  // Clear path queue just to be sure that first thing controller will do is to follow my
  // rotation trajectory assigned below
  path_queue_.clear();

  // Send 'path_on_spot' to path_queue to be followed by the controller
  path_queue_.insert(path_queue_.end(), yaw_path_on_spot.begin(),
                      yaw_path_on_spot.end());
  // TODO: ensure that after this routine is executed you replan again to the waypoint
  if (abort_executed_){
    // Restart controller timer when abort routine was executed
    command_publishing_timer_.start();
    abort_executed_ = 0;   // Reset the flag
  }
}
// TODO: Revise this function as this is the one that is giving me the problem when planning
// for multiple waypoints.
void MavLocalPlanner::faceToWaypoint(mav_msgs::EigenTrajectoryPoint& waypoint, double& my_yaw){
ROS_INFO("[Mav Local Planner][Facing to current waypoint] Routine on execution");

  mav_msgs::EigenTrajectoryPointVector rotation_to_waypoint;

  mav_msgs::EigenTrajectoryPoint initial_point;
  mav_msgs::EigenTrajectoryPoint final_point;

  // ROS_INFO_STREAM("[Mav Local Planner][Facing to current waypoint] Tracking waypoint: "
  //                   << waypoint.position_W.transpose());

  // recovery_yaw_policy_.setFacingPoint(waypoint.position_W);

  // ROS_INFO_STREAM("[Mav Local Planner][Facing to current waypoint] Waypoint stored in class: "
  //                   << recovery_yaw_policy_.getFacingPoint().transpose());

  // if (!path_queue_.empty()){
  //   // Get last element from the queue as initial position and orientation for rotation to waypoint
  //     mav_msgs::EigenTrajectoryPoint last_element_path_queue = *(path_queue_.end()-static_cast<size_t>(1));
  //     initial_point.position_W = last_element_path_queue.position_W;
  //     initial_point.orientation_W_B = last_element_path_queue.orientation_W_B;
  // } else {
  //   // Otherwise get the best thing we can, i.e. the odometry!
  //     initial_point.position_W = odometry_.position_W;
  //     initial_point.orientation_W_B = odometry_.orientation_W_B;
  // }

  initial_point.position_W = odometry_.position_W;
  initial_point.orientation_W_B = odometry_.orientation_W_B;
  rotation_to_waypoint.push_back(initial_point);

  // Compute heading/yaw to face waypoint
  my_yaw = compute_heading(waypoint);

  final_point.position_W = initial_point.position_W;
  final_point.orientation_W_B = toQuaternion(my_yaw, 0.0, 0.0);
  rotation_to_waypoint.push_back(final_point);

  recovery_yaw_policy_.applyPolicyInPlace(&rotation_to_waypoint);

  // THe clear() method below was clearing my queue in the 2nd iteration!!! so path_idx was 
  // always the at the end of the path and controller was not triggered again
  // path_queue_.clear();

  // ROS_INFO_STREAM("[Mav Local Planner][Facing to current waypoint] Path Queue Size Before: "
  //                   << path_queue_.size());

  path_queue_.insert(path_queue_.end(), rotation_to_waypoint.begin(),
                      rotation_to_waypoint.end());

  // ROS_INFO_STREAM("[Mav Local Planner][Facing to current waypoint] Path Queue Size After: "
  //                   << path_queue_.size());

  // ROS_INFO_STREAM("[Mav Local Planner][Facing to current waypoint] Path Queue Element 0 "
  //                 " Position: "
  //                   << path_queue_[0].position_W.transpose());

  // ROS_INFO_STREAM("[Mav Local Planner][Facing to current waypoint] Path Queue Element 0 "
  //                 " Orientation: "
  //                   << path_queue_[0].orientation_W_B.coeffs());

  // ROS_INFO_STREAM("[Mav Local Planner][Facing to current waypoint] Path Queue Element 1 "
  //                 " Position: "
  //                   << path_queue_[1].position_W.transpose());

  // ROS_INFO_STREAM("[Mav Local Planner][Facing to current waypoint] Path Queue Element 1 "
  //                 " Orientation: "
  //                   << path_queue_[1].orientation_W_B.coeffs());

}

Eigen::Quaterniond MavLocalPlanner::toQuaternion( double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Eigen::Quaterniond q;
    q.w() = cy * cp * cr + sy * sp * sr;
    q.x() = cy * cp * sr - sy * sp * cr;
    q.y() = sy * cp * sr + cy * sp * cr;
    q.z() = sy * cp * cr - cy * sp * sr;
    return q;
}

void MavLocalPlanner::toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());  
  yaw = atan2(siny_cosp, cosy_cosp);
}

double MavLocalPlanner::compute_heading(mav_msgs::EigenTrajectoryPoint& waypoint){
  // Compute heading/yaw to face waypoint

  mav_msgs::EigenTrajectoryPoint initial_point;
  initial_point.position_W = odometry_.position_W;
  initial_point.orientation_W_B = odometry_.orientation_W_B;

  Eigen::Vector3d direction_to_waypoint = waypoint.position_W - initial_point.position_W;
  double my_yaw = 0.0;
  if (std::abs(direction_to_waypoint.x()) > 1e-4 ||
      std::abs(direction_to_waypoint.y()) > 1e-4) {
    my_yaw = atan2(direction_to_waypoint.y(),direction_to_waypoint.x());
  }

  return my_yaw;

}

void MavLocalPlanner::increasePathQueue(){
  // Increase size of path_queue_ vector with the last element of the vector
  ROS_INFO("[Mav Local Planner][Increase Path Queue] Increasing path queue so controller thread triggers... ");
  // std::vector<mav_msgs::EigenTrajectoryPoint>::iterator it;
  // it = path_queue_.end()-static_cast<size_t>(1);
  path_queue_.insert(path_queue_.end(), 10,*(path_queue_.end()-static_cast<size_t>(1)));

  ROS_INFO_STREAM("[Mav Local Planner][Increase Path Queue] Path queue size = "
                    << path_queue_.size() << "   Current path index = " << path_index_ );
    // it);
}

// double MavLocalPlanner::computeYawDifference(double& yaw, double& desired_heading){
//   double delta_yaw = fmod(desired_heading - yaw, 2*M_PI);
//   // TODO: Check if the angles arg you receive (both computed with atan2) return values between
//   // -pi to pi, and if so adjust modulo args so that the remainder falls as you want.
//   // Maybe it is not even needed???
// }


}  // namespace mav_planning
