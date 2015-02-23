/*
 * Precomputation.h
 *
 *  Created on: Dec 8, 2014
 *      Author: chonhyon
 */

#ifndef PRECOMPUTATION_H_
#define PRECOMPUTATION_H_

#include <itomp_ca_planner/common.h>
#include <itomp_ca_planner/util/singleton.h>
#include <itomp_ca_planner/model/itomp_robot_model.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/astar_search.hpp>
#include <flann/flann.hpp>
#include <queue>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/TrajectoryConstraints.h>

namespace itomp_ca_planner
{

struct found_goal
{
};
// exception for termination
// visitor that terminates when we find the goal
template<class Vertex>
class astar_goal_visitor: public boost::default_astar_visitor
{
public:
	astar_goal_visitor(Vertex goal) :
                m_goal(goal)
	{
	}
	template<class Graph>
	void examine_vertex(Vertex u, Graph& g)
	{
		if (u == m_goal)
			throw found_goal();
	}
private:
	Vertex m_goal;
};

inline bool pathCompare(
        const std::pair<std::vector<const robot_state::RobotState*>, double>& p1,
        const std::pair<std::vector<const robot_state::RobotState*>, double>& p2)
{
	return p1.second < p2.second;
}

class Precomputation : public Singleton<Precomputation>
{
public:
	Precomputation();
	virtual ~Precomputation();

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
                        boost::property<vertex_total_connection_attempts_t,
                        unsigned int,
                        boost::property<
                        vertex_successful_connection_attempts_t,
                        unsigned int> > >,
			boost::property<boost::edge_weight_t, double,
                        boost::property<edge_scaled_weight_t, double> > > Graph;
	typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
	typedef boost::graph_traits<Graph>::edge_descriptor Edge;

	void initialize(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                        const ItompRobotModel& robot_model, const std::string& group_name);
	void createRoadmap();
	void createRoadmap(int milestones);
	void addStartState(const robot_state::RobotState& from);
	void addGoalStates(const std::vector<robot_state::RobotState>& to);
        bool localPlanning(const robot_state::RobotState& from, const robot_state::RobotState& to, double distance) const;
	bool extractPaths(int num_paths);

	void growRoadmap(int new_milestones);
	void expandRoadmap(int new_milestones);

	void renderPRMGraph();
	void renderPaths();

	int getNumMilestones() const;

	void extractInitialTrajectories(moveit_msgs::TrajectoryConstraints& trajectory_constraints);

protected:
	double costHeuristic(Vertex u, Vertex v) const;
        double costWorkspace(Vertex u, Vertex v) const;
	double distance(const robot_state::RobotState* s1,
                                        const robot_state::RobotState* s2) const;
        double workspaceDistance(const robot_state::RobotState* s1,
                                                         const robot_state::RobotState* s2) const;

        std::vector<const robot_state::RobotState*> smoothPath(const std::vector<const robot_state::RobotState*>& path) const;

	planning_scene::PlanningSceneConstPtr planning_scene_;
	std::string group_name_;
	const ItompRobotModel* robot_model_;

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
};

inline int Precomputation::getNumMilestones() const
{
	return states_.size();
}

}

#endif /* PRECOMPUTATION_H_ */
