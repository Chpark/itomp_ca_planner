/*
 * move_itomp.h
 *
 *  Created on: Sep 23, 2014
 *      Author: chonhyon
 */

#ifndef MOVE_KUKA_TEST_H_
#define MOVE_KUKA_TEST_H_

#include <moveit_msgs/RobotTrajectory.h>

namespace move_kuka
{

class MoveKukaTest
{
public:
	MoveKukaTest(const ros::NodeHandle& node_handle);
	~MoveKukaTest();

        void run(const std::string& group_name);

protected:
	void loadStaticScene();
	void initStartGoalStates(robot_state::RobotState& start_state,
                                                         std::vector<robot_state::RobotState>& goal_states);
	bool isStateCollide(const robot_state::RobotState& state);
	bool isStateSingular(robot_state::RobotState& state);

	void plan(planning_interface::MotionPlanRequest& req,
                          planning_interface::MotionPlanResponse& res,
                          const robot_state::RobotState& start_state,
                          std::vector<robot_state::RobotState>& goal_states,
                          bool use_itomp);

	void computeIKState(robot_state::RobotState& ik_state,
                                                const Eigen::Affine3d& end_effector_state, bool rand = false);

	void renderStartGoalStates(robot_state::RobotState& start_state,
                                                           robot_state::RobotState& goal_state);
	void drawPath(int id, const Eigen::Vector3d& from,
                                  const Eigen::Vector3d& to);
	void drawEndeffectorPosition(int id, const Eigen::Vector3d& position);

	ros::NodeHandle node_handle_;
	robot_model::RobotModelPtr robot_model_;
	planning_scene::PlanningScenePtr planning_scene_;
	planning_interface::PlannerManagerPtr itomp_planner_instance_;

	ros::Publisher planning_scene_diff_publisher_;
	ros::Publisher display_publisher_;
	ros::Publisher vis_marker_array_publisher_;

	std::string group_name_;
};

}

#endif /* MOVE_KUKA_TEST_H_ */
