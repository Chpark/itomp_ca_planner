/*

License

ITOMP Optimization-based Planner
Copyright © and trademark ™ 2014 University of North Carolina at Chapel Hill.
All rights reserved.

Permission to use, copy, modify, and distribute this software and its documentation
for educational, research, and non-profit purposes, without fee, and without a
written agreement is hereby granted, provided that the above copyright notice,
this paragraph, and the following four paragraphs appear in all copies.

This software program and documentation are copyrighted by the University of North
Carolina at Chapel Hill. The software program and documentation are supplied "as is,"
without any accompanying services from the University of North Carolina at Chapel
Hill or the authors. The University of North Carolina at Chapel Hill and the
authors do not warrant that the operation of the program will be uninterrupted
or error-free. The end-user understands that the program was developed for research
purposes and is advised not to rely exclusively on the program for any reason.

IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE AUTHORS
BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY STATUTORY WARRANTY
OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND
THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS HAVE NO OBLIGATIONS
TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

Any questions or comments should be sent to the author chpark@cs.unc.edu

*/
#ifndef ITOMP_OPTIMIZER_H_
#define ITOMP_OPTIMIZER_H_

#include <itomp_ca_planner/common.h>
#include <itomp_ca_planner/trajectory/itomp_cio_trajectory.h>
#include <itomp_ca_planner/optimization/evaluation_manager.h>
#include <itomp_ca_planner/optimization/improvement_manager.h>
#include <itomp_ca_planner/optimization/best_cost_manager.h>

namespace itomp_ca_planner
{
class ItompRobotModel;
class ItompPlanningGroup;
class ItompOptimizer
{
public:
	ItompOptimizer() {};
	ItompOptimizer(int trajectory_index, ItompCIOTrajectory* trajectory, ItompRobotModel *robot_model,
			const ItompPlanningGroup *planning_group, double planning_start_time, double trajectory_start_time,
			const moveit_msgs::Constraints& path_constraints, BestCostManager* best_cost_manager,
			const planning_scene::PlanningSceneConstPtr& planning_scene);
	virtual ~ItompOptimizer();

	bool optimize();
	double getBestCost() const;
	bool isSucceed() const;
	int getLastIteration() const;
    int getFirstViolationPoint() const;

    ItompCIOTrajectory& getGroupTrajectory();

private:
	void initialize(ItompRobotModel *robot_model, const ItompPlanningGroup *planning_group,
			double trajectory_start_time, const moveit_msgs::Constraints& path_constraints,
			const planning_scene::PlanningSceneConstPtr& planning_scene);
	bool updateBestTrajectory(double cost);

	bool is_feasible;
	bool terminated_;
	int trajectory_index_;
	double planning_start_time_;

	int iteration_;
	int feasible_iteration_;
	int last_improvement_iteration_;

	ItompCIOTrajectory* full_trajectory_;
	ItompCIOTrajectory group_trajectory_;

	EvaluationManager evaluation_manager_;
	ImprovementManagerPtr improvement_manager_;

	Eigen::MatrixXd best_group_trajectory_;
	Eigen::MatrixXd best_group_contact_trajectory_;
	double best_group_trajectory_cost_;

	BestCostManager* best_cost_manager_;
};

typedef boost::shared_ptr<ItompOptimizer> ItompOptimizerPtr;

////////////////////////////////////////////////////////////////////////////////

inline double ItompOptimizer::getBestCost() const
{
	return best_group_trajectory_cost_;
}

inline bool ItompOptimizer::isSucceed() const
{
	return is_feasible;
}

inline int ItompOptimizer::getLastIteration() const
{
	return iteration_;
}

inline ItompCIOTrajectory& ItompOptimizer::getGroupTrajectory()
{
    return group_trajectory_;
}

}

#endif
