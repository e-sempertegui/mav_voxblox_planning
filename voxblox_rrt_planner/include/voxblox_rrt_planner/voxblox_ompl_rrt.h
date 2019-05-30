#ifndef VOXBLOX_RRT_PLANNER_VOXBLOX_OMPL_RRT_H_
#define VOXBLOX_RRT_PLANNER_VOXBLOX_OMPL_RRT_H_

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>

#include <string>

#include "voxblox_rrt_planner/ompl/mav_setup.h"

namespace mav_planning {

class VoxbloxOmplRrt {
 public:
  enum RrtPlannerType {
    kRrtConnect = 0,
    kRrtStar,
    kInformedRrtStar,
    kBitStar,
    kPrm
  };

  VoxbloxOmplRrt(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~VoxbloxOmplRrt() {}

  inline void setRobotRadius(double robot_radius) {
    robot_radius_ = robot_radius;
  }
  void setBounds(const Eigen::Vector3d& lower_bound,
                 const Eigen::Vector3d& upper_bound);

  // Both are expected to be OWNED BY ANOTHER OBJECT that shouldn't go out of
  // scope while this object exists.
  void setTsdfLayer(voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer);
  void setEsdfLayer(voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer);

  inline void setOptimistic(bool optimistic) { optimistic_ = optimistic; }
  bool getOptimistic() const { return optimistic_; }

  double getNumSecondsToPlan() const { return num_seconds_to_plan_; }
  void setNumSecondsToPlan(double num_seconds) {
    num_seconds_to_plan_ = num_seconds;
  }

  RrtPlannerType getPlanner() const { return planner_type_; }
  void setPlanner(RrtPlannerType planner) { planner_type_ = planner; }

  // Only call this once, only call this after setting all settings correctly.
  void setupProblem();
  void setupMyProblem(const mav_msgs::EigenTrajectoryPoint& start);

  // Fixed start and end locations, returns list of waypoints between.
  bool getPathBetweenWaypoints(
      const mav_msgs::EigenTrajectoryPoint& start,
      const mav_msgs::EigenTrajectoryPoint& goal,
      mav_msgs::EigenTrajectoryPoint::Vector* solution);

  void solutionPathToTrajectoryPoints(
      ompl::geometric::PathGeometric& path,
      mav_msgs::EigenTrajectoryPointVector* trajectory_points) const;

  // Even if planning fails, get the part of the tree that spans closest to
  // the original goal point. Returns true if it was actually successfully
  // able to plan to the original goal point, false otherwise.
  bool getBestPathTowardGoal(const mav_msgs::EigenTrajectoryPoint& start,
                             const mav_msgs::EigenTrajectoryPoint& goal,
                             mav_msgs::EigenTrajectoryPoint::Vector* solution);

  void constructPrmRoadmap(double roadmap_construction_sec) {
    problem_setup_.setup();
    problem_setup_.constructPrmRoadmap(roadmap_construction_sec);
  }

  RrtPlannerType string_to_planner_enum(const std::string& planner_string){
    if (planner_string == "kRrtConnect") return kRrtConnect;
    else if (planner_string == "kRrtStar") return kRrtStar;
    else if (planner_string == "kInformedRrtStar") return kInformedRrtStar;
    else if (planner_string == "kPrm") return kPrm;
    else if (planner_string == "kBitStar") return kBitStar;
    else{
      ROS_INFO("Specified planner is not currently available. Using default RRT* instead...");
      return kRrtStar;
    }
  }

 protected:
  void setupFromStartAndGoal(const mav_msgs::EigenTrajectoryPoint& start,
                             const mav_msgs::EigenTrajectoryPoint& goal);

  double getDistanceEigenToState(const Eigen::Vector3d& eigen,
                                 const ompl::base::State* state_ptr);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Setup the problem in OMPL.
  ompl::mav::MavSetup problem_setup_;
  RrtPlannerType planner_type_;
  double num_seconds_to_plan_;
  bool simplify_solution_;
  double robot_radius_;
  bool verbose_;

  // Whether the planner is optimistic (true) or pessimistic (false) about
  // how unknown space is handled.
  // Optimistic uses the TSDF for collision checking, while pessimistic uses
  // the ESDF. Be sure to set the maps accordingly.
  bool optimistic_;

  // Whether to trust an approximate solution (i.e., not necessarily reaching
  // the exact goal state).
  bool trust_approx_solution_;

  // Planning bounds, if set.
  Eigen::Vector3d lower_bound_;
  Eigen::Vector3d upper_bound_;

  // NON-OWNED pointers to the relevant layers. TSDF only used if optimistic,
  // ESDF only used if pessimistic.
  voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer_;
  voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer_;

  double voxel_size_;

  // WEIGHT PARAMETERS for multi-objective optimisation
  double alpha_;
  double beta_;

  // Max length of branches between nodes. Make sure max_branch_length > 0 as otherwise
  // OMPL default is considered (~7 m)
  double max_branch_length_;
};

}  // namespace mav_planning

#endif  // VOXBLOX_RRT_PLANNER_VOXBLOX_OMPL_RRT_H_
