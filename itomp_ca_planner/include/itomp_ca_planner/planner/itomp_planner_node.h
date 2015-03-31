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
#ifndef ITOMP_PLANNER_NODE_H_
#define ITOMP_PLANNER_NODE_H_

#include <itomp_ca_planner/common.h>
#include <itomp_ca_planner/planner/planning_info.h>
#include <itomp_ca_planner/model/itomp_robot_model.h>
#include <itomp_ca_planner/trajectory/itomp_cio_trajectory.h>
#include <itomp_ca_planner/optimization/itomp_optimizer.h>
#include <itomp_ca_planner/optimization/best_cost_manager.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>

namespace itomp_ca_planner
{

class ItompPlannerNode
{
public:
	ItompPlannerNode(const robot_model::RobotModelConstPtr& model);
	virtual ~ItompPlannerNode();

	bool init();

	int run();

	bool planKinematicPath(
			const planning_scene::PlanningSceneConstPtr& planning_scene,
			const planning_interface::MotionPlanRequest &req,
			planning_interface::MotionPlanResponse &res);

private:
	bool preprocessRequest(const planning_interface::MotionPlanRequest &req);
	void getGoalState(const planning_interface::MotionPlanRequest &req,
			sensor_msgs::JointState& goalState);
	void initTrajectory(const sensor_msgs::JointState &joint_state);
	void getPlanningGroups(std::vector<std::string>& plannningGroups,
			const std::string& groupName);
	void fillGroupJointTrajectory(const std::string& groupName,
			const sensor_msgs::JointState& jointGoalState,
			const moveit_msgs::Constraints& path_constraints,
			const moveit_msgs::TrajectoryConstraints& trajectory_constraints);
	void trajectoryOptimization(const std::string& groupName,
			const sensor_msgs::JointState& jointGoalState,
			const moveit_msgs::Constraints& path_constraints,
			const moveit_msgs::TrajectoryConstraints& trajectory_constraints,
			const planning_scene::PlanningSceneConstPtr& planning_scene);

	void
	fillInResult(const std::vector<std::string>& planningGroups,
			planning_interface::MotionPlanResponse &res);

	ItompRobotModel robot_model_;

	ItompCIOTrajectoryPtr trajectory_;
	std::vector<ItompCIOTrajectoryPtr> trajectories_;
	std::vector<ItompOptimizerPtr> optimizers_;

	double trajectory_start_time_;

	double last_planning_time_;
	int last_min_cost_trajectory_;

	std::vector<std::vector<PlanningInfo> > planning_info_;
	void resetPlanningInfo(int trials, int component);
	void writePlanningInfo(int trials, int component);
	void printPlanningInfoSummary();

	double planning_start_time_;
	int planning_count_;

	Eigen::MatrixXd start_point_velocities_;
	Eigen::MatrixXd start_point_accelerations_;

	robot_state::RobotStatePtr complete_initial_robot_state_;

	BestCostManager best_cost_manager_;

	void jointConstraintsToJointState(
			const std::vector<moveit_msgs::Constraints> &constraints,
			std::vector<sensor_msgs::JointState>& joint_states)
	{
		joint_states.clear();

		for (unsigned int i = 0; i < constraints.size(); i++)
		{
			sensor_msgs::JointState state;
			state.name.clear();
			state.position.clear();

			const std::vector<moveit_msgs::JointConstraint> &joint_constraints =
					constraints[i].joint_constraints;

			for (unsigned int j = 0; j < joint_constraints.size(); j++)
			{
				state.name.push_back(joint_constraints[j].joint_name);
				state.position.push_back(joint_constraints[j].position);
			}
			joint_states.push_back(state);
		}
	}
};

}

#endif
