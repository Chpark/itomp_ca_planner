/*
 * move_itomp.h
 *
 *  Created on: Sep 23, 2014
 *      Author: chonhyon
 */

#ifndef MOVE_KUKA_TEST_H_
#define MOVE_KUKA_TEST_H_

#include <moveit_msgs/RobotTrajectory.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/astar_search.hpp>
#include <flann/flann.hpp>
#include <queue>

namespace move_kuka
{

struct found_goal {}; // exception for termination
// visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
  astar_goal_visitor(Vertex goal) : m_goal(goal) {}
  template <class Graph>
  void examine_vertex(Vertex u, Graph& g) {
    if(u == m_goal)
      throw found_goal();
  }
private:
  Vertex m_goal;
};

inline bool pathCompare(const std::pair<std::vector<const robot_state::RobotState*>, double>& p1,
		const std::pair<std::vector<const robot_state::RobotState*>, double>& p2)
{
	return p1.second < p2.second;
}

class MoveKukaTest
{
public:
	struct vertex_state_t
	{
		typedef boost::vertex_property_tag kind;
	};
	struct vertex_total_connection_attempts_t
	{
		typedef boost::vertex_property_tag kind;
	};
	struct vertex_successful_connection_attempts_t
	{
		typedef boost::vertex_property_tag kind;
	};
	struct edge_scaled_weight_t
	{
		typedef boost::edge_property_tag kind;
	};
	typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
			boost::property<vertex_state_t, const robot_state::RobotState*,
			boost::property<vertex_total_connection_attempts_t,	unsigned int,
			boost::property<vertex_successful_connection_attempts_t, unsigned int> > >,
			boost::property<boost::edge_weight_t, double,
			boost::property<edge_scaled_weight_t, double> > > Graph;
	typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
	typedef boost::graph_traits<Graph>::edge_descriptor Edge;

	MoveKukaTest(const ros::NodeHandle& node_handle);
	~MoveKukaTest();

	void run(const std::string& group_name);

protected:
	void loadStaticScene();
	bool isStateCollide(const robot_state::RobotState& state);
	bool isStateSingular(robot_state::RobotState& state);

	void plan(planning_interface::MotionPlanRequest& req,
			planning_interface::MotionPlanResponse& res,
			robot_state::RobotState& start_state,
			robot_state::RobotState& goal_state);
	void plan(planning_interface::MotionPlanRequest& req,
			planning_interface::MotionPlanResponse& res,
			robot_state::RobotState& start_state,
			geometry_msgs::PoseStamped& goal_pose,
			const std::string& endeffector_link);

	void displayState(robot_state::RobotState& state);
	void displayStates(robot_state::RobotState& start_state,
			robot_state::RobotState& goal_state);

	void computeIKState(robot_state::RobotState& ik_state,
			const Eigen::Affine3d& end_effector_state, bool rand = false);

	void printTrajectory(const moveit_msgs::RobotTrajectory &traj);

	void drawPath(int id, const Eigen::Vector3d& from,
			const Eigen::Vector3d& to);
	void drawEndeffectorPosition(int id, const Eigen::Vector3d& position);

	ros::NodeHandle node_handle_;
	robot_model::RobotModelPtr robot_model_;
	planning_scene::PlanningScenePtr planning_scene_;
	planning_interface::PlannerManagerPtr ompl_planner_instance_;
	planning_interface::PlannerManagerPtr itomp_planner_instance_;

	ros::Publisher planning_scene_diff_publisher_;
	ros::Publisher display_publisher_;
	ros::Publisher vis_marker_array_publisher_;

	std::string group_name_;

	void createRoadmap(int milestones);
	void addStartState(const robot_state::RobotState& from);
	void addGoalStates(const std::vector<robot_state::RobotState>& to);
	bool localPlanning(const robot_state::RobotState& from,
			const robot_state::RobotState& to, double distance);
	bool extractPaths(int num_paths);

	void growRoadmap(int new_milestones);
	void expandRoadmap(int new_milestones);

	Graph g_;
	Vertex start_vertex_;
	std::vector<Vertex> goal_vertices_;
	std::vector<const robot_state::RobotState*> states_;
	std::vector<std::pair<std::vector<const robot_state::RobotState*>, double> > paths_;
	boost::property_map<Graph, vertex_state_t>::type stateProperty_;
	boost::property_map<Graph, vertex_total_connection_attempts_t>::type totalConnectionAttemptsProperty_;
	boost::property_map<Graph, vertex_successful_connection_attempts_t>::type successfulConnectionAttemptsProperty_;
	boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;
	boost::property_map<Graph, edge_scaled_weight_t>::type copiedWeightProperty_;

	double costHeuristic(Vertex u, Vertex v) const;
	double distance(const robot_state::RobotState* s1, const robot_state::RobotState* s2) const;
	void renderPRMGraph();
	void renderPaths();
};

}

#endif /* MOVE_KUKA_TEST_H_ */
