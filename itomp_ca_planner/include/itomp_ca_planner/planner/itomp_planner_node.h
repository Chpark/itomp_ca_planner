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

template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);
        return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

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
                                                                  const planning_interface::MotionPlanRequest &req,
                                                                  const planning_scene::PlanningSceneConstPtr& planning_scene);
	void trajectoryOptimization(const std::string& groupName,
                                                                const planning_interface::MotionPlanRequest& req,
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

        void jointConstraintsToJointState(const std::vector<moveit_msgs::Constraints> &constraints,
                                                                          std::vector<sensor_msgs::JointState>& joint_states)
	{
		joint_states.clear();

		for (unsigned int i = 0; i < constraints.size(); i++)
		{
			sensor_msgs::JointState state;
			state.name.clear();
			state.position.clear();

                        const std::vector<moveit_msgs::JointConstraint> &joint_constraints = constraints[i].joint_constraints;

			for (unsigned int j = 0; j < joint_constraints.size(); j++)
			{
				state.name.push_back(joint_constraints[j].joint_name);
				state.position.push_back(joint_constraints[j].position);
			}
			joint_states.push_back(state);
		}
	}

        // TODO: move
        bool collisionAwareIK(robot_state::RobotState& robot_state, const Eigen::Affine3d& transform,
                                                  const std::string& group_name, const std::string& link_name,
                                                  const planning_scene::PlanningSceneConstPtr& planning_scene) const;
};

}

#endif
