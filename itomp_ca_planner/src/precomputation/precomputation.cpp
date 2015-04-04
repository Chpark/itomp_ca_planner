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

#include <itomp_ca_planner/precomputation/precomputation.h>
#include <moveit/robot_model/robot_model.h>
#include <itomp_ca_planner/visualization/visualization_manager.h>
#include <itomp_ca_planner/util/planning_parameters.h>
#include <omp.h>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <algorithm>
#include <queue>

using namespace std;

namespace itomp_ca_planner
{
flann::SearchParams FLANN_PARAMS;

Precomputation::Precomputation() :
    stateProperty_(boost::get(vertex_state_t(), g_)),
    weightProperty_(boost::get(boost::edge_weight, g_))
{
    FLANN_PARAMS.checks = 128;
    FLANN_PARAMS.cores = 8;
}

Precomputation::~Precomputation()
{
    reset();
}

void Precomputation::initialize(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                const ItompRobotModel& robot_model, const std::string& group_name)
{
	planning_scene_ = planning_scene;
	group_name_ = group_name;
	robot_model_ = &robot_model;

    size_t num_threads_ = omp_get_max_threads();
    omp_set_num_threads(num_threads_);
    std::cout << "Use " << num_threads_ << " threads on " << omp_get_num_procs() << " processors\n";
}

void Precomputation::createRoadmap()
{
    if (PlanningParameters::getInstance()->getUsePrecomputation() == false)
        return;

    computeVizPRM();
    renderPRMGraph();
    std::cout << "# of milestones : " << states_.size() << std::endl;

    std::vector<int> component(num_vertices(g_));
    int num = connected_components(g_, &component[0]);
    std::cout << "Total number of components: " << num << std::endl;

    computePDR();
    renderPRMGraph();
    std::cout << "# of milestones : " << states_.size() << std::endl;
    component.resize(num_vertices(g_));
    num = connected_components(g_, &component[0]);
    std::cout << "Total number of components: " << num << std::endl;

}

robot_state::RobotState* Precomputation::getNewFeasibleSample() const
{
    const robot_state::RobotState& current_state = planning_scene_->getCurrentState();
    robot_state::RobotState* new_state = new robot_state::RobotState(current_state);
    while (true)
    {
        new_state->setToRandomPositions();
        new_state->update(true);
        if (planning_scene_->isStateValid(*new_state))
        {
            return new_state;
        }
    }
}
void Precomputation::computePDR()
{
    const int ntry_max = 100;

    int last_max = 0;
    int ntry = 0;

    while (ntry < ntry_max)
    {
        robot_state::RobotState* new_state = getNewFeasibleSample();
        bool added = false;

        std::vector<int> component(num_vertices(g_));
        int num_components = connected_components(g_, &component[0]);

        Graph g;


        std::set<Vertex> visible_set;
        std::map<Vertex, Vertex> graph_index_backward_mapping;

        // Iterate through the vertices and add visible nodes to the new graph
        BOOST_FOREACH (Vertex v, boost::vertices(g_))
        {
            if (localPlanning(*new_state, *states_[v]))
            {
                visible_set.insert(v);

                // add vertex to the new graph
                Vertex m = boost::add_vertex(g);

                graph_index_backward_mapping[m] = v;
            }
        }
        // Iterate through the edges and add edges if two end nodes are visible
        BOOST_FOREACH (const Edge e, boost::edges(g_))
        {
            const Vertex u = boost::source(e, g_);
            const Vertex v = boost::target(e, g_);

            if (visible_set.find(u) != visible_set.end() && visible_set.find(v) != visible_set.end())
            {
                // add edge
                boost::add_edge(u, v, weightProperty_[e], g);
            }
        }

        num_components = connected_components(g, &component[0]);

        if (num_components == 1)
        {
            // edge visibility test
            std::set<std::pair<int,int> > invisible_edge_set;
            BOOST_FOREACH (const Edge e, boost::edges(g))
            {
                const Vertex u = boost::source(e, g);
                const Vertex v = boost::target(e, g);

                if (localEdgePlanning(*new_state, *stateProperty_[graph_index_backward_mapping[u]], *stateProperty_[graph_index_backward_mapping[v]]) == false)
                {
                    invisible_edge_set.insert(std::make_pair(u,v));
                }
            }

            for (std::set<std::pair<int,int> >::iterator it = invisible_edge_set.begin(); it != invisible_edge_set.end(); ++it)
            {
                remove_edge(it->first, it->second, g);
            }
            num_components = connected_components(g, &component[0]);
        }

        if (num_components == 0)
        {
            // add vertex
            Vertex m = boost::add_vertex(g_);
            states_.push_back(new_state);
            added = true;
            stateProperty_[m] = states_[states_.size() - 1];

            cout << "New vertex " << states_.size() - 1 << " : ";
            for (int k = 0; k < 7; ++k)
                cout << new_state->getVariablePositions()[k] << " ";
            cout << endl;

            renderPRMGraph();
        }
        else if (num_components == 1)
        {
            // if # of connected components = 1, new sample does not add a new loop
        }
        else
        {
            // if # of connected components > 1, new sample can add a new loop

            int last_vertex = -1;
            int last_vertex_component = -1;
            for (int i = 0; i < num_vertices(g_); ++i)
            {
                // discard vertex in the same component
                if (last_vertex_component == component[i])
                    continue;

                // TODO: remove
                if (degree(graph_index_backward_mapping[i], g_) > 1)
                    continue;

                if (last_vertex == -1)
                {
                    last_vertex = i;
                    last_vertex_component = component[i];
                }
                else
                {                    
                    Vertex ou = graph_index_backward_mapping[last_vertex];
                    Vertex ov = graph_index_backward_mapping[i];

                    // redundancy check
                    std::vector<Path> paths;
                    std::vector<Vertex> goal;
                    goal.push_back(ov);
                    getKShortestPaths(ou, goal, 10, paths);

                    // add new path
                    Vertex m = boost::add_vertex(g_);
                    states_.push_back(new_state);
                    added = true;
                    int new_vertex_index = states_.size() - 1;
                    stateProperty_[m] = states_[new_vertex_index];
                    const Graph::edge_property_type properties(distance(new_state, states_[ou]));
                    Edge e1 = boost::add_edge(new_vertex_index, ou, properties, g_).first;
                    const Graph::edge_property_type properties2(distance(new_state, states_[ov]));
                    Edge e2 = boost::add_edge(new_vertex_index, ov, properties2, g_).first;
                    Path new_path(ou);
                    new_path.addVertex(m, 0.0);
                    new_path.addVertex(ov, 0.0);

                    bool is_deformable = false;
                    for (int j = 0; j < paths.size(); ++j)
                    {
                        if (isDeformable(new_path, paths[j]))
                        {
                            is_deformable = true;
                            break;
                        }
                    }

                    // delete if redundant
                    if (is_deformable)
                    {
                        boost::remove_edge(e1, g_);
                        boost::remove_edge(e2, g_);
                        boost::remove_vertex(m, g_);

                        states_.resize(states_.size() - 1);

                        added = false;
                    }
                    else
                    {
                        cout << "New vertex " << new_vertex_index << " : ";
                        for (int k = 0; k < 7; ++k)
                            cout << new_state->getVariablePositions()[k] << " ";
                        cout << endl;
                    }

                    renderPRMGraph();

                    break;
                }
            }


        }

        if (!added)
        {
            delete new_state;
            ++ntry;
        }
        else
        {
            if (ntry > last_max)
            {
                last_max = ntry;
                ROS_INFO("Max ntry : %d", ntry);
            }
            ntry = 0;
        }

    }
}

void Precomputation::computeVizPRM()
{
    const int ntry_max = 1000;
    int last_max = 0;
    int ntry = 0;

    int num_components = 0;

    while (ntry < ntry_max)
    {
        robot_state::RobotState* new_state = getNewFeasibleSample();
        bool added = false;

        std::vector<int> component(num_vertices(g_));
        num_components = connected_components(g_, &component[0]);

        int last_vertex = -1;
        int last_vertex_component = -1;
        for (int i = 0; i < num_vertices(g_); ++i)
        {
            if (last_vertex_component == component[i])
                continue;

            if (localPlanning(*new_state, *states_[i]))
            {
                if (last_vertex == -1)
                {
                    last_vertex = i;
                    last_vertex_component = component[i];
                }
                else
                {
                    // add vertex
                    Vertex m = boost::add_vertex(g_);
                    states_.push_back(new_state);
                    added = true;
                    int new_vertex_index = states_.size() - 1;
                    stateProperty_[m] = states_[new_vertex_index];

                    // add connector
                    const Graph::edge_property_type properties(distance(new_state, states_[last_vertex]));
                    boost::add_edge(new_vertex_index, last_vertex, properties, g_);

                    const Graph::edge_property_type properties2(distance(new_state, states_[i]));
                    boost::add_edge(new_vertex_index, i, properties2, g_);

                    cout << "New vertex " << new_vertex_index << " : ";
                    for (int k = 0; k < 7; ++k)
                        cout << new_state->getVariablePositions()[k] << " ";
                    cout << endl;


                    renderPRMGraph();

                    break;
                }
            }
        }
        if (last_vertex == -1)
        {
            // add vertex
            Vertex m = boost::add_vertex(g_);
            states_.push_back(new_state);
            added = true;
            stateProperty_[m] = states_[states_.size() - 1];

            cout << "New vertex " << states_.size() - 1 << " : ";
            for (int k = 0; k < 7; ++k)
                cout << new_state->getVariablePositions()[k] << " ";
            cout << endl;

            //ntry = 0;

            renderPRMGraph();
        }
        else
        {
            //++ntry;
        }

        if (!added)
        {
            delete new_state;
            ++ntry;
        }
        else
        {
            if (ntry > last_max)
            {
                last_max = ntry;
                ROS_INFO("Max ntry : %d cc : %d", ntry, num_components);
            }
            ntry = 0;
        }
    }
}

bool Precomputation::localPlanning(const robot_state::RobotState& from, const robot_state::RobotState& to, double distance) const
{
    const double LONGEST_VALID_SEGMENT_LENGTH = PlanningParameters::getInstance()->getPrecomputationMaxValidSegmentDist();

	bool result = true;
	int nd = ceil(distance / LONGEST_VALID_SEGMENT_LENGTH);

	/* initialize the queue of test positions */
	std::queue<std::pair<int, int> > pos;

    if (nd >= 2)
    {
        pos.push(std::make_pair(1, nd - 1));

        int num_threads_ = omp_get_max_threads();
        std::vector<robot_state::RobotState> test(num_threads_, robot_state::RobotState(from));

        std::vector<int> evaluation_index(num_threads_);
        while (!pos.empty())
        {
            int i = 0;
            for ( ; i < num_threads_; ++i)
            {
                if (pos.empty())
                    break;

                std::pair<int, int> x = pos.front();

                int mid = (x.first + x.second) / 2;

                evaluation_index[i] = mid;

                pos.pop();

                if (x.first < mid)
                    pos.push(std::make_pair(x.first, mid - 1));
                if (x.second > mid)
                    pos.push(std::make_pair(mid + 1, x.second));
            }
            int num_evaluations = i;

            #pragma omp parallel for
            for (int i = 0; i < num_evaluations; ++i)
            {
                int mid = evaluation_index[i];
                from.interpolate(to, (double) mid / (double) nd, test[i]);
                test[i].updateCollisionBodyTransforms();

                if (!planning_scene_->isStateValid(test[i]))
                {
                    result = false;
                }
            }
            if (!result)
                break;
        }
    }

	return result;
}

bool Precomputation::localEdgePlanning(const robot_state::RobotState& from, const robot_state::RobotState& edge_start,
                                       const robot_state::RobotState& edge_end) const
{
    const double LONGEST_VALID_SEGMENT_LENGTH = PlanningParameters::getInstance()->getPrecomputationMaxValidSegmentDist();

    double edge_distance = distance(&edge_start, &edge_end);

    int nd = ceil(edge_distance / LONGEST_VALID_SEGMENT_LENGTH);

    robot_state::RobotState test(edge_start);
    for (int i = 0; i <= nd; ++i)
    {
        edge_start.interpolate(edge_end, (double)i / nd, test);

        if (localPlanning(from, test) == false)
            return false;
    }
    return true;
}

void Precomputation::addStartState(const robot_state::RobotState& from)
{
    if (PlanningParameters::getInstance()->getUsePrecomputation() == false)
        return;

	int dim = from.getVariableCount();

    const int NN = PlanningParameters::getInstance()->getPrecomputationNn();

    int data_size = states_.size();
    int safe_nn = std::min(data_size, NN);

	states_.resize(states_.size() + 1);
	int milestones = states_.size();
	states_[milestones - 1] = new robot_state::RobotState(from);

	// find nearest neighbors
    flann::Matrix<double> dataset(new double[data_size * dim], data_size, dim);
	const flann::Matrix<double> query(new double[1 * dim], 1, dim);
    flann::Matrix<int> indices(new int[query.rows * safe_nn], query.rows, safe_nn);
    flann::Matrix<double> dists(new double[query.rows * safe_nn], query.rows, safe_nn);
	{
		double* data_ptr = dataset.ptr();
        for (int i = 0; i < data_size; ++i)
		{
            memcpy(data_ptr, states_[i]->getVariablePositions(), sizeof(double) * dim);
			data_ptr += dim;
		}

		memcpy(query.ptr(), from.getVariablePositions(), sizeof(double) * dim);

		// do a knn search, using flann libarary
        flann::Index<flann::L2<double> > index(dataset, flann::KDTreeIndexParams(4));
        if (safe_nn > 0)
        {
            index.buildIndex();
            index.knnSearch(query, indices, dists, safe_nn, FLANN_PARAMS);
        }
	}

	// add to graph
	std::vector<Vertex> graph_vertices(milestones);

	// TODO:
	for (int i = 0; i < milestones - 1; ++i)
		graph_vertices[i] = i;
	// add vertices
	for (int i = milestones - 1; i < milestones; ++i)
	{
		Vertex m = boost::add_vertex(g_);
		stateProperty_[m] = states_[i];
		graph_vertices[i] = m;
	}
	start_vertex_ = graph_vertices[milestones - 1];

	// add edges
	for (int i = milestones - 1; i < milestones; ++i)
	{
        for (int j = 0; j < safe_nn; ++j)
		{
            int index = indices.ptr()[(i - (milestones - 1)) * safe_nn + j];
            double weight = sqrt(dists.ptr()[(i - (milestones - 1)) * safe_nn + j]);

			bool result = true;

			result = localPlanning(*states_[i], *states_[index], weight);

			if (result)
			{
				const Graph::edge_property_type properties(weight);
                boost::add_edge(graph_vertices[i], graph_vertices[index], properties, g_);
			}

            /*
            printf("Start NN %d (%f) : ", j, weight);
            for (int k = 0; k < dim; ++k)
            {
                printf("%f ", states_[index]->getVariablePositions()[k]);
            }
            printf("\n");
            */
		}
	}

	delete[] dataset.ptr();
	delete[] query.ptr();
	delete[] indices.ptr();
	delete[] dists.ptr();
}

void Precomputation::addGoalStates(const std::vector<robot_state::RobotState>& to)
{
    if (PlanningParameters::getInstance()->getUsePrecomputation() == false)
        return;

    const int NN = PlanningParameters::getInstance()->getPrecomputationNn();

	goal_vertices_.clear();
	int dim = to[0].getVariableCount();
	int num_goal_states = to.size();

    int data_size = states_.size();
    int safe_nn = std::min(data_size, NN);

	states_.resize(states_.size() + num_goal_states);
	int milestones = states_.size();
	for (int i = 0; i < num_goal_states; ++i)
        states_[milestones - num_goal_states + i] = new robot_state::RobotState(to[i]);

	// find nearest neighbors
    flann::Matrix<double> dataset(new double[data_size * dim], data_size, dim);
    const flann::Matrix<double> query(new double[num_goal_states * dim], num_goal_states, dim);
    flann::Matrix<int> indices(new int[query.rows * safe_nn], query.rows, safe_nn);
    flann::Matrix<double> dists(new double[query.rows * safe_nn], query.rows, safe_nn);
	{
		double* data_ptr = dataset.ptr();
        for (int i = 0; i < data_size; ++i)
		{
            memcpy(data_ptr, states_[i]->getVariablePositions(), sizeof(double) * dim);
			data_ptr += dim;
		}
		double* query_ptr = query.ptr();
		for (int i = 0; i < num_goal_states; ++i)
		{
            memcpy(query_ptr, to[i].getVariablePositions(), sizeof(double) * dim);
			query_ptr += dim;
		}

		// do a knn search, using flann libarary
        flann::Index<flann::L2<double> > index(dataset, flann::KDTreeIndexParams(4));
		index.buildIndex();
        index.knnSearch(query, indices, dists, safe_nn, FLANN_PARAMS);
	}

	// add to graph
	std::vector<Vertex> graph_vertices(milestones);

	// TODO:
	for (int i = 0; i < milestones - num_goal_states; ++i)
		graph_vertices[i] = i;
	// add vertices
	for (int i = milestones - num_goal_states; i < milestones; ++i)
	{
		Vertex m = boost::add_vertex(g_);
		stateProperty_[m] = states_[i];
		graph_vertices[i] = m;
		goal_vertices_.push_back(m);
	}

	// add edges
	for (int i = milestones - num_goal_states; i < milestones; ++i)
	{
        for (int j = 0; j < safe_nn; ++j)
		{
            int index = indices.ptr()[(i - (milestones - num_goal_states)) * safe_nn + j];
            double weight = sqrt(dists.ptr()[(i - (milestones - num_goal_states)) * safe_nn + j]);

			bool result = true;

			result = localPlanning(*states_[i], *states_[index], weight);

			if (result)
			{
				const Graph::edge_property_type properties(weight);
                boost::add_edge(graph_vertices[i], graph_vertices[index], properties, g_);
			}

            /*
            printf("Goal %d NN %d (%f) : ", i, j, weight);
            for (int k = 0; k < dim; ++k)
            {
                printf("%f ", states_[index]->getVariablePositions()[k]);
            }
            printf("\n");
            */
		}
	}

	delete[] dataset.ptr();
	delete[] query.ptr();
	delete[] indices.ptr();
	delete[] dists.ptr();
}

bool Precomputation::extractPaths(int num_paths)
{
    paths_.clear();
    getKShortestPaths(start_vertex_, goal_vertices_, num_paths, paths_);

	renderPaths();

	return true;
}

double Precomputation::distance(const robot_state::RobotState* s1,
                                const robot_state::RobotState* s2) const
{
	double cost = 0.0;
	int dim = s1->getVariableCount();
	for (int i = 0; i < dim; ++i)
	{
		double c = s1->getVariablePosition(i) - s2->getVariablePosition(i);
		cost += c * c;
	}
	return sqrt(cost);
}

double Precomputation::workspaceDistance(const robot_state::RobotState* s1, const robot_state::RobotState* s2) const
{
    double cost = 0.0;

    const std::string end_effector_name = robot_model_->getGroupEndeffectorLinkName(group_name_);

    const Eigen::Affine3d& transform1 = s1->getGlobalLinkTransform(end_effector_name);
    const Eigen::Affine3d& transform2 = s2->getGlobalLinkTransform(end_effector_name);

    const Eigen::Vector3d& translation1 = transform1.translation();
    const Eigen::Vector3d& translation2 = transform2.translation();
    cost += (translation1 - translation2).norm();

    return cost;
}

double Precomputation::costHeuristic(Vertex u, Vertex v) const
{
	const robot_state::RobotState* s1 = stateProperty_[u];
	const robot_state::RobotState* s2 = stateProperty_[v];
	return distance(s1, s2);
}

double Precomputation::costWorkspace(Vertex u, Vertex v) const
{
    const robot_state::RobotState* s1 = stateProperty_[u];
    const robot_state::RobotState* s2 = stateProperty_[v];
    return workspaceDistance(s1, s2);
}

void Precomputation::renderPaths()
{
	if (PlanningParameters::getInstance()->getDrawPrecomputation() == false)
		return;

	const double trajectory_color_diff = 0.33;
	const double scale = 0.005, scale2 = 0.001;
	const int marker_step = 1;

	const std::string end_effector_name =
        robot_model_->getGroupEndeffectorLinkName(group_name_);

	visualization_msgs::MarkerArray ma;
	visualization_msgs::Marker::_color_type RED;
    visualization_msgs::Marker::_color_type BLUE;
	RED.a = 1.0;
	RED.r = 1.0;
	RED.g = 0.0;
	RED.b = 0.0;
    BLUE.a = 1.0;
    BLUE.r = 0.0;
    BLUE.g = 0.0;
    BLUE.b = 1.0;

	visualization_msgs::Marker msg;
	msg.header.frame_id = robot_model_->getRobotModel()->getModelFrame();
	msg.header.stamp = ros::Time::now();
	msg.ns = "prm_results";
	msg.action = visualization_msgs::Marker::ADD;

	msg.points.resize(0);
	geometry_msgs::Point point;

	msg.id = 2;
	msg.type = visualization_msgs::Marker::LINE_STRIP;
	msg.scale.x = scale2;
	msg.scale.y = scale2;
	msg.scale.z = scale2;
	msg.color = RED;

    const double LONGEST_VALID_SEGMENT_LENGTH = PlanningParameters::getInstance()->getPrecomputationMaxValidSegmentDist();
	for (int j = 0; j < paths_.size(); ++j)
	{
		msg.points.resize(0);
        for (int i = 0; i < paths_[j].getPath().size() - 1; ++i)
		{
            const robot_state::RobotState* from = stateProperty_[paths_[j].getPath()[i]];
            const robot_state::RobotState* to = stateProperty_[paths_[j].getPath()[i + 1]];
			double dist = distance(from, to);
			int nd = ceil(dist / LONGEST_VALID_SEGMENT_LENGTH);

			for (int k = 0; k <= nd; ++k)
			{
				robot_state::RobotState test(*from);
				from->interpolate(*to, (double) k / (double) nd, test);
				test.updateLinkTransforms();

                const Eigen::Affine3d& transform = test.getGlobalLinkTransform(end_effector_name);

				point.x = transform.translation()(0);
				point.y = transform.translation()(1);
				point.z = transform.translation()(2);
				msg.points.push_back(point);

                k = nd;
			}

            const Eigen::Affine3d& transform = to->getGlobalLinkTransform(end_effector_name);

            point.x = transform.translation()(0);
            point.y = transform.translation()(1);
            point.z = transform.translation()(2);
            msg.points.push_back(point);

		}
        msg.color = BLUE;
		ma.markers.push_back(msg);
		++msg.id;

        //

        msg.points.resize(0);
        for (int i = 0; i < paths_[j].getPath().size() - 1; ++i)
        {
            const robot_state::RobotState* from = stateProperty_[paths_[j].getPath()[i]];
            const robot_state::RobotState* to = stateProperty_[paths_[j].getPath()[i + 1]];
            double dist = distance(from, to);
            int nd = ceil(dist / LONGEST_VALID_SEGMENT_LENGTH);

            for (int k = 0; k <= nd; ++k)
            {
                robot_state::RobotState test(*from);
                from->interpolate(*to, (double) k / (double) nd, test);
                test.updateLinkTransforms();

                const Eigen::Affine3d& transform = test.getGlobalLinkTransform(end_effector_name);

                point.x = transform.translation()(0);
                point.y = transform.translation()(1);
                point.z = transform.translation()(2);
                msg.points.push_back(point);
            }
        }
        msg.color = RED;
        ma.markers.push_back(msg);
        ++msg.id;
	}

	VisualizationManager::getInstance()->getVisualizationMarkerArrayPublisher().publish(
        ma);
}

void Precomputation::renderPRMGraph()
{
	if (PlanningParameters::getInstance()->getDrawPrecomputation() == false)
		return;

	const double trajectory_color_diff = 0.33;
    const double scale = 0.01, scale2 = 0.0025;
	const int marker_step = 1;

	const std::string end_effector_name =
        robot_model_->getGroupEndeffectorLinkName(group_name_);

	visualization_msgs::MarkerArray ma;
	visualization_msgs::Marker::_color_type BLUE, GREEN, LIGHT_YELLOW;
	BLUE.a = 1.0;
	BLUE.r = 1.0;
	BLUE.g = 1.0;
	BLUE.b = 1.0;
	LIGHT_YELLOW = BLUE;
    LIGHT_YELLOW.b = 0.0;
    GREEN.a = 1.0;
	GREEN.r = 0.5;
	GREEN.b = 0.5;
	GREEN.g = 1.0;

	visualization_msgs::Marker msg;
	msg.header.frame_id = robot_model_->getRobotModel()->getModelFrame();
	msg.header.stamp = ros::Time::now();
	msg.ns = "prm_vertices";
	msg.type = visualization_msgs::Marker::SPHERE_LIST;
	msg.action = visualization_msgs::Marker::ADD;

	msg.scale.x = scale;
	msg.scale.y = scale;
	msg.scale.z = scale;

	msg.id = 0;
	msg.color = LIGHT_YELLOW;

	msg.points.resize(0);
	geometry_msgs::Point point;

	BOOST_FOREACH (Vertex v, boost::vertices(g_))
	{
        msg.points.resize(0);

		const Eigen::Affine3d& transform =
            stateProperty_[v]->getGlobalLinkTransform(end_effector_name);
		point.x = transform.translation()(0);
		point.y = transform.translation()(1);
		point.z = transform.translation()(2);

        msg.color.g = 1.0;

		msg.points.push_back(point);

        ma.markers.push_back(msg);
        ++msg.id;
	}
    //ma.markers.push_back(msg);

    ++msg.id;
	msg.points.resize(0);
	msg.type = visualization_msgs::Marker::LINE_LIST;
	msg.scale.x = scale2;
	msg.scale.y = scale2;
	msg.scale.z = scale2;
	msg.color = GREEN;

	BOOST_FOREACH (const Edge e, boost::edges(g_))
	{
		const Vertex u = boost::source(e, g_);
		const Vertex v = boost::target(e, g_);

		const Eigen::Affine3d& transform =
            stateProperty_[u]->getGlobalLinkTransform(end_effector_name);

		point.x = transform.translation()(0);
		point.y = transform.translation()(1);
		point.z = transform.translation()(2);
		msg.points.push_back(point);

		const Eigen::Affine3d& transform2 =
            stateProperty_[v]->getGlobalLinkTransform(end_effector_name);
		point.x = transform2.translation()(0);
		point.y = transform2.translation()(1);
		point.z = transform2.translation()(2);
		msg.points.push_back(point);
	}

    ma.markers.push_back(msg);

    VisualizationManager::getInstance()->getVisualizationMarkerArrayPublisher().publish(ma);

	ros::WallDuration sleep_time(0.01);
	sleep_time.sleep();
}

void Precomputation::extractInitialTrajectories(moveit_msgs::TrajectoryConstraints& trajectory_constraints)
{
    if (PlanningParameters::getInstance()->getUsePrecomputation() == false)
        return;

    renderPRMGraph();

    int num_trajectories = PlanningParameters::getInstance()->getNumTrajectories();
	while (extractPaths(num_trajectories) == false)
	{
        // TODO: add more vertices
        ROS_ASSERT(false);
	}
	trajectory_constraints.constraints.clear();
	int traj_constraint_begin = 0;

    if (paths_.size() < num_trajectories)
        num_trajectories = paths_.size();

	for (int c = 0; c < num_trajectories; ++c)
	{
		int num_joints = planning_scene_->getCurrentState().getVariableCount();

		moveit_msgs::JointConstraint jc;
        int num_points = paths_[c].getPath().size();
        trajectory_constraints.constraints.resize(traj_constraint_begin + num_points);
        std::string trajectory_index_string = boost::lexical_cast<std::string>(c);
		for (int j = 0; j < num_points; ++j)
		{
			int point = j + traj_constraint_begin;
			if (j == 0)
                trajectory_constraints.constraints[point].name = trajectory_index_string;
			if (j == num_points - 1)
				trajectory_constraints.constraints[point].name = "end";

            trajectory_constraints.constraints[point].joint_constraints.resize(num_joints);
			for (int k = 0; k < num_joints; ++k)
			{
                jc.joint_name = planning_scene_->getCurrentState().getVariableNames()[k];
                jc.position = stateProperty_[paths_[c].getPath()[j]]->getVariablePosition(k);
                trajectory_constraints.constraints[point].joint_constraints[k] = jc;
			}
		}
		traj_constraint_begin += num_points;
	}
}

void Precomputation::reset()
{
    for (int i = 0; i < states_.size(); ++i)
        delete states_[i];

    g_.clear();
    paths_.clear();
    states_.clear();
}

void Precomputation::getKShortestPaths(Vertex start, std::vector<Vertex>& goals, int k, std::vector<Path>& paths) const
{
    // Dijkstra
    paths.clear();

    boost::vector_property_map<int> num_paths(boost::num_vertices(g_));
    BOOST_FOREACH (Vertex v, boost::vertices(g_))
    {
        num_paths[v] = 0;
    }

    std::priority_queue<Path, std::vector<Path>, std::greater<Path> > current_paths;
    current_paths.push(Path(start));

    int num_shortest_paths_to_goals = 0;
    while (!current_paths.empty() && num_shortest_paths_to_goals < k)
    {
        Path p = current_paths.top();
        current_paths.pop();
        Vertex v = p.getPath().back();
        ++num_paths[v];

        if (std::find(goals.begin(), goals.end(), v) != goals.end())
        {
            cout << num_shortest_paths_to_goals << " : ";
            for (int i = 0; i < p.getPath().size(); ++i)
                cout << p.getPath()[i] << " ";
            cout << " : " << p.getCost() << endl;
            paths.push_back(p);
            ++num_shortest_paths_to_goals;
        }
        if (num_paths[v] <= k)
        {
            BOOST_FOREACH (Vertex u, boost::adjacent_vertices(v, g_))
            {
                if (std::find(p.getPath().begin(), p.getPath().end(), u) == p.getPath().end())
                {
                    Path new_p = p;
                    Edge e = boost::edge(u, v, g_).first;
                    double cost = weightProperty_[e];
                    new_p.addVertex(u, cost);
                    current_paths.push(new_p);
                }
            }
        }
    }

    return;
}

typedef boost::grid_graph<2> GridGraph;
typedef boost::graph_traits<GridGraph>::vertex_descriptor GridVertex;
typedef boost::graph_traits<GridGraph>::edge_descriptor GridEdge;
// A hash function for vertices.
struct GridVertexHasher {
  std::size_t operator()(GridVertex const& u) const {
    std::size_t seed = 0;
    boost::hash_combine(seed, u[0]);
    boost::hash_combine(seed, u[1]);
    return seed;
  }
};


double gridHeuristicCostEstimate(const GridVertex& from, const GridVertex& to)
{
    return (double)(std::abs(to[0] - from[0]) + std::abs(to[1] - from[1]));
}

bool greaterFScore(const GridVertex& v1, const GridVertex& v2,
                   const boost::unordered_map<GridVertex, double, GridVertexHasher>* f_score,
                   const boost::unordered_map<GridVertex, double, GridVertexHasher>* g_score)
{
    double f1 = f_score->at(v1);
    double f2 = f_score->at(v2);
    if (f1 == f2)
    {
        double g1 = g_score->at(v1);
        double g2 = g_score->at(v2);
        return g1 < g2;
    }
    else
        return f1 > f2;
}


bool Precomputation::isDeformable(const Path& from, const Path& to) const
{
    const double LONGEST_VALID_SEGMENT_LENGTH = PlanningParameters::getInstance()->getPrecomputationMaxValidSegmentDist();

    // get n waypoints on 'from' path
    std::vector<robot_state::RobotState> from_waypoints;
    from_waypoints.push_back(*stateProperty_[from.getPath()[0]]);
    robot_state::RobotState state(*stateProperty_[from.getPath()[0]]);

    for (int i = 1; i < from.getPath().size(); ++i)
    {
        const robot_state::RobotState* s1 = stateProperty_[from.getPath()[i - 1]];
        const robot_state::RobotState* s2 = stateProperty_[from.getPath()[i]];

        double dist = distance(s1, s2);

        int nd = ceil(dist / LONGEST_VALID_SEGMENT_LENGTH);

        for (int j = 1; j <= nd; ++j)
        {
            s1->interpolate(*s2, (double) j / (double) nd, state);
            from_waypoints.push_back(state);
        }
    }
    int n = from_waypoints.size();

    // get m waypoints on 'to' path
    std::vector<robot_state::RobotState> to_waypoints;
    to_waypoints.push_back(*stateProperty_[to.getPath()[0]]);
    for (int i = 1; i < from.getPath().size(); ++i)
    {
        const robot_state::RobotState* s1 = stateProperty_[to.getPath()[i - 1]];
        const robot_state::RobotState* s2 = stateProperty_[to.getPath()[i]];

        double dist = distance(s1, s2);

        int nd = ceil(dist / LONGEST_VALID_SEGMENT_LENGTH);

        for (int j = 1; j <= nd; ++j)
        {
            s1->interpolate(*s2, (double) j / (double) nd, state);
            to_waypoints.push_back(state);
        }
    }
    int m = to_waypoints.size();

    if (from_waypoints.size() <= 2 || to_waypoints.size() <= 2)
        return true;

    // on n X m grid, A* search path from (0,0) to (1,1)
    // (x,y) is enabled if local planning from x to y succeeds
    // Implement A* search with lazy evaluation

    boost::array<std::size_t, 2> lengths = { {n, m} };
    GridGraph grid(lengths);
    GridVertex start = boost::vertex(0, grid);
    GridVertex goal = boost::vertex(boost::num_vertices(grid) - 1, grid);

    boost::unordered_map<GridVertex, double, GridVertexHasher > g_score;
    boost::unordered_map<GridVertex, double, GridVertexHasher > f_score;

    g_score[start] = 0.0;
    f_score[start] = g_score[start] + gridHeuristicCostEstimate(start, goal);

    boost::unordered_set<GridVertex, GridVertexHasher> closed_set;
    boost::unordered_set<GridVertex, GridVertexHasher> open_set;
    std::priority_queue<GridVertex, std::vector<GridVertex>, boost::function<bool(GridVertex, GridVertex)> >
            open_priority_queue(boost::bind(greaterFScore, _1, _2, &f_score, &g_score));

    open_set.insert(start);
    open_priority_queue.push(start);

    while (!open_set.empty())
    {
        GridVertex current = open_priority_queue.top();
        if (current == goal)
            return true;

        open_set.erase(current);
        open_priority_queue.pop();
        closed_set.insert(current);

        BOOST_FOREACH (GridVertex u, boost::adjacent_vertices(current, grid))
        {
            if (closed_set.find(u) != closed_set.end())
                continue;
            double tentative_g_score = g_score[current] + 1.0;

            if (open_set.find(u) == open_set.end())
            {
                const robot_state::RobotState& s1 = from_waypoints[u[0]];
                const robot_state::RobotState& s2 = to_waypoints[u[1]];
                if (localPlanning(s1, s2) == false)
                {
                        closed_set.insert(u);
                        continue;
                }

                g_score[u] = tentative_g_score;
                f_score[u] = g_score[u] + gridHeuristicCostEstimate(u, goal);

                open_set.insert(u);
                open_priority_queue.push(u);
            }
            else if (tentative_g_score < g_score[u])
            {
                g_score[u] = tentative_g_score;
                f_score[u] = g_score[u] + gridHeuristicCostEstimate(u, goal);
            }
        }
    }

    return false;
}

}

