#ifndef VOXBLOX_RRT_PLANNER_OMPL_ALTITUDE_OBJECTIVE_H_
#define VOXBLOX_RRT_PLANNER_OMPL_ALTITUDE_OBJECTIVE_H_ 

#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/Cost.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "voxblox_rrt_planner/ompl/ompl_types.h"
#include "voxblox_rrt_planner/ompl/ompl_voxblox.h"

#include <cmath>

namespace ompl{
namespace mav{

class AltitudeObjective : public ompl::base::StateCostIntegralObjective 
{
public:
	AltitudeObjective(const ompl::base::SpaceInformationPtr& si, const double h):
		constant_height(h),
		ompl::base::StateCostIntegralObjective(si, true){} 
	// ~AltitudeObjective();

	ompl::base::Cost stateCost(const ompl::base::State* s) const {
		return ompl::base::Cost(this->altitude_deviation(s));
	}

	double altitude_deviation(const ompl::base::State* s) const{
		const Eigen::Vector3d robot_state = omplToEigen(s);
		double z = robot_state[2];		//access the z-coordinate of current state s
		return sqrt(pow(z - constant_height,2));
	}

private:
	double constant_height;
	
};

}	//namespace mav
}	//namespace ompl


#endif