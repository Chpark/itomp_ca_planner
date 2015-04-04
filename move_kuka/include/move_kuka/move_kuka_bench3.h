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
        void initStartGoalStates(planning_interface::MotionPlanRequest& req, int index = 0);
	bool isStateCollide(const robot_state::RobotState& state);
	bool isStateSingular(robot_state::RobotState& state);

        void plan(planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res, bool use_itomp);

        void computeIKState(robot_state::RobotState& ik_state, const Eigen::Affine3d& end_effector_state, bool rand = false);

        void renderStartGoalStates(robot_state::RobotState& start_state, robot_state::RobotState& goal_state);
        void renderState(const robot_state::RobotState& state);
        void drawPath(int id, const Eigen::Vector3d& from, const Eigen::Vector3d& to);
	void drawEndeffectorPosition(int id, const Eigen::Vector3d& position);

	ros::NodeHandle node_handle_;
	robot_model::RobotModelPtr robot_model_;
	planning_scene::PlanningScenePtr planning_scene_;
	planning_interface::PlannerManagerPtr itomp_planner_instance_;

	ros::Publisher planning_scene_diff_publisher_;
	ros::Publisher display_publisher_;
	ros::Publisher vis_marker_array_publisher_;

        robot_state::RobotStatePtr last_goal_state_;

	std::string group_name_;

        void drawObstacle();
        void readTrajectory(moveit_msgs::RobotTrajectory& trajectory, const std::string& filename);
        void readOptimization(int index);
        void drawOptimization(int index);
        void playTrajectory(int index);
        void renderStateForOptimization(const robot_state::RobotState& state, int i);
        std::vector<std::pair<double, Eigen::MatrixXd> > optimization_data_[4];
        ros::Publisher opt_state_display_publisher_[11];
};

}

#endif /* MOVE_KUKA_TEST_H_ */
