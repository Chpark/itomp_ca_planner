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

template<class Vertex>
class GraphPath
{
public:
    GraphPath();
    GraphPath(Vertex v);

    const std::vector<Vertex>& getPath() const;
    double getCost() const;

    bool operator< (const GraphPath& p) const;
    bool operator> (const GraphPath& p) const;

    void addVertex(Vertex v, double cost);

private:
    std::vector<Vertex> path_;
    double cost_;
};

template<class Vertex>
inline GraphPath<Vertex>::GraphPath()
    : cost_(0.0)
{
}

template<class Vertex>
inline GraphPath<Vertex>::GraphPath(Vertex v)
    : cost_(0.0)
{
    path_.push_back(v);
}

template<class Vertex>
inline const std::vector<Vertex>& GraphPath<Vertex>::getPath() const
{
    return path_;
}

template<class Vertex>
inline double GraphPath<Vertex>::getCost() const
{
    return cost_;
}

template<class Vertex>
inline bool GraphPath<Vertex>::operator <(const GraphPath& p) const
{
    return cost_ < p.cost_;
}

template<class Vertex>
inline bool GraphPath<Vertex>::operator >(const GraphPath& p) const
{
    return cost_ > p.cost_;
}

template<class Vertex>
inline void GraphPath<Vertex>::addVertex(Vertex v, double cost)
{
    path_.push_back(v);
    cost_ += cost;
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
	typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
            boost::property<vertex_state_t, const robot_state::RobotState*>,
            boost::property<boost::edge_weight_t, double> > Graph;
	typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
	typedef boost::graph_traits<Graph>::edge_descriptor Edge;
    typedef GraphPath<Vertex> Path;

	void initialize(const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const ItompRobotModel& robot_model, const std::string& group_name);
	void createRoadmap();
    void addStartState(const robot_state::RobotState& from);
	void addGoalStates(const std::vector<robot_state::RobotState>& to);

    void extractInitialTrajectories(moveit_msgs::TrajectoryConstraints& trajectory_constraints);

    int getNumMilestones() const;

    void reset();

protected:
    void renderPRMGraph();
    void renderPaths();

    void computeVizPRM();
    void computePDR();
    robot_state::RobotState* getNewFeasibleSample() const;

    bool localPlanning(const robot_state::RobotState& from, const robot_state::RobotState& to) const;
    bool localPlanning(const robot_state::RobotState& from, const robot_state::RobotState& to, double distance) const;
    bool localEdgePlanning(const robot_state::RobotState& from, const robot_state::RobotState& edge_start, const robot_state::RobotState& edge_end) const;

    bool extractPaths(int num_paths);

    void getKShortestPaths(Vertex start, std::vector<Vertex>& goals, int k, std::vector<Path>& paths) const;
    bool isDeformable(const Path& from, const Path& to) const;

	double costHeuristic(Vertex u, Vertex v) const;
    double costWorkspace(Vertex u, Vertex v) const;
    double distance(const robot_state::RobotState* s1, const robot_state::RobotState* s2) const;
    double workspaceDistance(const robot_state::RobotState* s1, const robot_state::RobotState* s2) const;

	planning_scene::PlanningSceneConstPtr planning_scene_;
	std::string group_name_;
	const ItompRobotModel* robot_model_;

	Graph g_;
	Vertex start_vertex_;
	std::vector<Vertex> goal_vertices_;
	std::vector<const robot_state::RobotState*> states_;

	boost::property_map<Graph, vertex_state_t>::type stateProperty_;
	boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;

    std::vector<Path> paths_;
};

inline int Precomputation::getNumMilestones() const
{
	return states_.size();
}

inline bool Precomputation::localPlanning(const robot_state::RobotState& from, const robot_state::RobotState& to) const
{
    return localPlanning(from, to, distance(&from, &to));
}


}

#endif /* PRECOMPUTATION_H_ */
