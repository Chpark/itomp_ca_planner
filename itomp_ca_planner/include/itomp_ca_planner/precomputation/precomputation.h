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
