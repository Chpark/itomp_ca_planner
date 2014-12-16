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

namespace itomp_ca_planner
{

Precomputation::Precomputation() :
		stateProperty_(boost::get(vertex_state_t(), g_)), totalConnectionAttemptsProperty_(
				boost::get(vertex_total_connection_attempts_t(), g_)), successfulConnectionAttemptsProperty_(
				boost::get(vertex_successful_connection_attempts_t(), g_)), weightProperty_(
				boost::get(boost::edge_weight, g_)), copiedWeightProperty_(
				boost::get(edge_scaled_weight_t(), g_))
{

}

Precomputation::~Precomputation()
{

}

void Precomputation::initialize(
		const planning_scene::PlanningSceneConstPtr& planning_scene,
		const ItompRobotModel& robot_model, const std::string& group_name)
{
	planning_scene_ = planning_scene;
	group_name_ = group_name;
	robot_model_ = &robot_model;
}

void Precomputation::growRoadmap(int new_milestones)
{
	int old_milestones = states_.size();

	const robot_state::RobotState& current_state =
			planning_scene_->getCurrentState();

	int dim = current_state.getVariableCount();

	states_.resize(states_.size() + new_milestones);
	int milestones = states_.size();

	// sample
	for (int i = old_milestones; i < milestones; ++i)
	{
		robot_state::RobotState* state = new robot_state::RobotState(
				current_state);
		while (true)
		{
			state->setToRandomPositions();
			state->updateCollisionBodyTransforms();
			if (planning_scene_->isStateValid(*state))
			{
				states_[i] = state;
				break;
			}
		}
	}

	const int NN = PlanningParameters::getInstance()->getPrecomputationNn() + 1;

	// find nearest neighbors
	flann::Matrix<double> dataset(new double[milestones * dim], milestones,
			dim);
	flann::Matrix<double> query(new double[new_milestones * dim],
			new_milestones, dim);
	flann::Matrix<int> indices(new int[query.rows * NN], query.rows, NN);
	flann::Matrix<double> dists(new double[query.rows * NN], query.rows, NN);
	{
		double* data_ptr = dataset.ptr();
		for (int i = 0; i < milestones; ++i)
		{
			memcpy(data_ptr, states_[i]->getVariablePositions(),
					sizeof(double) * dim);
			data_ptr += dim;
		}
		double* query_ptr = query.ptr();
		for (int i = 0; i < new_milestones; ++i)
		{
			memcpy(query_ptr,
					states_[old_milestones + i]->getVariablePositions(),
					sizeof(double) * dim);
			query_ptr += dim;
		}

		// do a knn search, using flann libarary
		flann::Index<flann::L2<double> > index(dataset,
				flann::KDTreeIndexParams(4));
		index.buildIndex();
		index.knnSearch(query, indices, dists, NN, flann::SearchParams(128));
	}

	// create graph
	std::map<std::pair<int, int>, bool> local_planning_result;
	std::vector<Vertex> graph_vertices(milestones);

	// TODO:
	for (int i = 0; i < old_milestones; ++i)
		graph_vertices[i] = i;
	// add vertices
	for (int i = old_milestones; i < milestones; ++i)
	{
		Vertex m = boost::add_vertex(g_);
		stateProperty_[m] = states_[i];
		totalConnectionAttemptsProperty_[m] = 1;
		successfulConnectionAttemptsProperty_[m] = 0;
		graph_vertices[i] = m;
	}
	// add edges
	for (int i = old_milestones; i < milestones; ++i)
	{
		// ignore j = 0 (vertex itself)
		for (int j = 1; j < NN; ++j)
		{
			int index = indices.ptr()[(i - old_milestones) * NN + j];
			double weight = sqrt(dists.ptr()[(i - old_milestones) * NN + j]);

			bool result = true;
			if (index < i && index >= old_milestones)
			{
				std::map<std::pair<int, int>, bool>::const_iterator it =
						local_planning_result.find(
								std::make_pair<int, int>(index, i));
				if (it != local_planning_result.end())
					continue;
				else
					result = localPlanning(*states_[i], *states_[index],
							weight);
			}
			else
			{
				result = localPlanning(*states_[i], *states_[index], weight);
				local_planning_result.insert(
						std::make_pair<std::pair<int, int>, bool>(
								std::make_pair<int, int>(i, index), result));
			}

			totalConnectionAttemptsProperty_[graph_vertices[i]]++;
			totalConnectionAttemptsProperty_[graph_vertices[index]]++;
			if (result)
			{
				const Graph::edge_property_type properties(weight);
				boost::add_edge(graph_vertices[i], graph_vertices[index],
						properties, g_);
				successfulConnectionAttemptsProperty_[graph_vertices[i]]++;
				successfulConnectionAttemptsProperty_[graph_vertices[index]]++;
			}
		}
	}

	delete[] dataset.ptr();
	delete[] indices.ptr();
	delete[] dists.ptr();
}
void Precomputation::expandRoadmap(int new_milestones)
{
	int old_milestones = states_.size();

	const int NN = PlanningParameters::getInstance()->getPrecomputationNn() + 1;

	const robot_state::RobotState& current_state =
			planning_scene_->getCurrentState();

	const robot_state::JointModelGroup* joint_model_group =
			current_state.getJointModelGroup(group_name_);

	int dim = current_state.getVariableCount();

	states_.resize(states_.size() + new_milestones);
	int milestones = states_.size();

	// PDF
	std::map<double, Vertex> pdf;
	double prob_acc = 0.0;
	BOOST_FOREACH (Vertex v, boost::vertices(g_))
	{
		const unsigned int t = totalConnectionAttemptsProperty_[v];
		const unsigned int s = successfulConnectionAttemptsProperty_[v];
		prob_acc += (double) (t - s) / t;

		pdf[prob_acc] = v;
	}

	// sample
	for (int i = old_milestones; i < milestones; ++i)
	{
		robot_state::RobotState* state = new robot_state::RobotState(
				current_state);

		double r = (double) rand() / RAND_MAX * prob_acc;
		std::map<double, Vertex>::iterator it = pdf.lower_bound(r);
		if (it == pdf.end())
			continue;
		Vertex v = it->second;
		const robot_state::RobotState* s = stateProperty_[v];

		const double LONGEST_VALID_SEGMENT_LENGTH =
				PlanningParameters::getInstance()->getPrecomputationMaxValidSegmentDist();
		while (true)
		{

			state->setToRandomPositionsNearBy(joint_model_group, *s,
					LONGEST_VALID_SEGMENT_LENGTH * 0.5);
			state->updateCollisionBodyTransforms();

			if (planning_scene_->isStateValid(*state))
			{
				states_[i] = state;
				break;
			}
		}
	}

	// find nearest neighbors
	flann::Matrix<double> dataset(new double[milestones * dim], milestones,
			dim);
	flann::Matrix<double> query(new double[new_milestones * dim],
			new_milestones, dim);
	flann::Matrix<int> indices(new int[query.rows * NN], query.rows, NN);
	flann::Matrix<double> dists(new double[query.rows * NN], query.rows, NN);
	{
		double* data_ptr = dataset.ptr();
		for (int i = 0; i < milestones; ++i)
		{
			memcpy(data_ptr, states_[i]->getVariablePositions(),
					sizeof(double) * dim);
			data_ptr += dim;
		}
		double* query_ptr = query.ptr();
		for (int i = 0; i < new_milestones; ++i)
		{
			memcpy(query_ptr,
					states_[old_milestones + i]->getVariablePositions(),
					sizeof(double) * dim);
			query_ptr += dim;
		}

		// do a knn search, using flann libarary
		flann::Index<flann::L2<double> > index(dataset,
				flann::KDTreeIndexParams(4));
		index.buildIndex();
		index.knnSearch(query, indices, dists, NN, flann::SearchParams(128));
	}

	// create graph
	std::map<std::pair<int, int>, bool> local_planning_result;
	std::vector<Vertex> graph_vertices(milestones);

	// TODO:
	for (int i = 0; i < old_milestones; ++i)
		graph_vertices[i] = i;
	// add vertices
	for (int i = old_milestones; i < milestones; ++i)
	{
		Vertex m = boost::add_vertex(g_);
		stateProperty_[m] = states_[i];
		totalConnectionAttemptsProperty_[m] = 1;
		successfulConnectionAttemptsProperty_[m] = 0;
		graph_vertices[i] = m;
	}
	// add edges
	for (int i = old_milestones; i < milestones; ++i)
	{
		// ignore j = 0 (vertex itself)
		for (int j = 1; j < NN; ++j)
		{
			int index = indices.ptr()[(i - old_milestones) * NN + j];
			double weight = sqrt(dists.ptr()[(i - old_milestones) * NN + j]);

			bool result = true;
			if (index < i && index >= old_milestones)
			{
				std::map<std::pair<int, int>, bool>::const_iterator it =
						local_planning_result.find(
								std::make_pair<int, int>(index, i));
				if (it != local_planning_result.end())
					continue;
				else
					result = localPlanning(*states_[i], *states_[index],
							weight);
			}
			else
			{
				result = localPlanning(*states_[i], *states_[index], weight);
				local_planning_result.insert(
						std::make_pair<std::pair<int, int>, bool>(
								std::make_pair<int, int>(i, index), result));
			}

			totalConnectionAttemptsProperty_[graph_vertices[i]]++;
			totalConnectionAttemptsProperty_[graph_vertices[index]]++;
			if (result)
			{
				const Graph::edge_property_type properties(weight);
				boost::add_edge(graph_vertices[i], graph_vertices[index],
						properties, g_);
				successfulConnectionAttemptsProperty_[graph_vertices[i]]++;
				successfulConnectionAttemptsProperty_[graph_vertices[index]]++;
			}
		}
	}

	delete[] dataset.ptr();
	delete[] indices.ptr();
	delete[] dists.ptr();
}

void Precomputation::createRoadmap()
{
	createRoadmap(
			PlanningParameters::getInstance()->getPrecomputationInitMilestones());
}

void Precomputation::createRoadmap(int milestones)
{
	ROS_INFO("Create %d milestones", milestones);
	while (states_.size() < milestones)
	{
		growRoadmap(
				PlanningParameters::getInstance()->getPrecomputationGrowMilestones());
		expandRoadmap(
				PlanningParameters::getInstance()->getPrecomputationExpandMilestones());

		renderPRMGraph();
	}
}

bool Precomputation::localPlanning(const robot_state::RobotState& from,
		const robot_state::RobotState& to, double distance)
{
	const double LONGEST_VALID_SEGMENT_LENGTH =
			PlanningParameters::getInstance()->getPrecomputationMaxValidSegmentDist();

	bool result = true;
	int nd = ceil(distance / LONGEST_VALID_SEGMENT_LENGTH);

	/* initialize the queue of test positions */
	std::queue<std::pair<int, int> > pos;
	if (nd >= 2)
	{
		pos.push(std::make_pair(1, nd - 1));

		/* temporary storage for the checked state */
		robot_state::RobotState test(from);

		/* repeatedly subdivide the path segment in the middle (and check the middle) */
		while (!pos.empty())
		{
			std::pair<int, int> x = pos.front();

			int mid = (x.first + x.second) / 2;
			from.interpolate(to, (double) mid / (double) nd, test);
			test.updateCollisionBodyTransforms();

			if (!planning_scene_->isStateValid(test))
			{
				result = false;
				break;
			}

			pos.pop();

			if (x.first < mid)
				pos.push(std::make_pair(x.first, mid - 1));
			if (x.second > mid)
				pos.push(std::make_pair(mid + 1, x.second));
		}
	}

	return result;
}

void Precomputation::addStartState(const robot_state::RobotState& from)
{
	int dim = from.getVariableCount();

	const int NN = PlanningParameters::getInstance()->getPrecomputationNn() + 1;

	states_.resize(states_.size() + 1);
	int milestones = states_.size();
	states_[milestones - 1] = new robot_state::RobotState(from);

	// find nearest neighbors
	flann::Matrix<double> dataset(new double[milestones * dim], milestones,
			dim);
	const flann::Matrix<double> query(new double[1 * dim], 1, dim);
	flann::Matrix<int> indices(new int[query.rows * NN], query.rows, NN);
	flann::Matrix<double> dists(new double[query.rows * NN], query.rows, NN);
	{
		double* data_ptr = dataset.ptr();
		for (int i = 0; i < milestones; ++i)
		{
			memcpy(data_ptr, states_[i]->getVariablePositions(),
					sizeof(double) * dim);
			data_ptr += dim;
		}

		memcpy(query.ptr(), from.getVariablePositions(), sizeof(double) * dim);

		// do a knn search, using flann libarary
		flann::Index<flann::L2<double> > index(dataset,
				flann::KDTreeIndexParams(4));
		index.buildIndex();
		index.knnSearch(query, indices, dists, NN, flann::SearchParams(128));
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
		totalConnectionAttemptsProperty_[m] = 1;
		successfulConnectionAttemptsProperty_[m] = 0;
		graph_vertices[i] = m;
	}
	start_vertex_ = graph_vertices[milestones - 1];

	// add edges
	for (int i = milestones - 1; i < milestones; ++i)
	{
		for (int j = 1; j < NN; ++j)
		{
			int index = indices.ptr()[(i - (milestones - 1)) * NN + j];
			double weight = sqrt(dists.ptr()[(i - (milestones - 1)) * NN + j]);

			bool result = true;

			result = localPlanning(*states_[i], *states_[index], weight);

			totalConnectionAttemptsProperty_[graph_vertices[i]]++;
			if (result)
			{
				const Graph::edge_property_type properties(weight);
				boost::add_edge(graph_vertices[i], graph_vertices[index],
						properties, g_);
			}
		}
	}

	delete[] dataset.ptr();
	delete[] query.ptr();
	delete[] indices.ptr();
	delete[] dists.ptr();
}

void Precomputation::addGoalStates(
		const std::vector<robot_state::RobotState>& to)
{
	const int NN = PlanningParameters::getInstance()->getPrecomputationNn() + 1;

	goal_vertices_.clear();
	int dim = to[0].getVariableCount();
	int num_goal_states = to.size();

	states_.resize(states_.size() + num_goal_states);
	int milestones = states_.size();
	for (int i = 0; i < num_goal_states; ++i)
		states_[milestones - num_goal_states + i] = new robot_state::RobotState(
				to[i]);

	// find nearest neighbors
	flann::Matrix<double> dataset(new double[milestones * dim], milestones,
			dim);
	const flann::Matrix<double> query(new double[num_goal_states * dim],
			num_goal_states, dim);
	flann::Matrix<int> indices(new int[query.rows * NN], query.rows, NN);
	flann::Matrix<double> dists(new double[query.rows * NN], query.rows, NN);
	{
		double* data_ptr = dataset.ptr();
		for (int i = 0; i < milestones; ++i)
		{
			memcpy(data_ptr, states_[i]->getVariablePositions(),
					sizeof(double) * dim);
			data_ptr += dim;
		}
		double* query_ptr = query.ptr();
		for (int i = 0; i < num_goal_states; ++i)
		{
			memcpy(query_ptr, to[i].getVariablePositions(),
					sizeof(double) * dim);
			query_ptr += dim;
		}

		// do a knn search, using flann libarary
		flann::Index<flann::L2<double> > index(dataset,
				flann::KDTreeIndexParams(4));
		index.buildIndex();
		index.knnSearch(query, indices, dists, NN, flann::SearchParams(128));
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
		totalConnectionAttemptsProperty_[m] = 1;
		successfulConnectionAttemptsProperty_[m] = 0;
		graph_vertices[i] = m;
		goal_vertices_.push_back(m);
	}

	// add edges
	for (int i = milestones - num_goal_states; i < milestones; ++i)
	{
		for (int j = 1; j < NN; ++j)
		{
			int index = indices.ptr()[(i - (milestones - num_goal_states)) * NN
					+ j];
			double weight = sqrt(
					dists.ptr()[(i - (milestones - num_goal_states)) * NN + j]);

			bool result = true;

			result = localPlanning(*states_[i], *states_[index], weight);

			totalConnectionAttemptsProperty_[graph_vertices[i]]++;
			if (result)
			{
				const Graph::edge_property_type properties(weight);
				boost::add_edge(graph_vertices[i], graph_vertices[index],
						properties, g_);
			}
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

	double best_cost = std::numeric_limits<double>::max();
	for (int j = 0; j < goal_vertices_.size(); ++j)
	{
		Vertex goal_vertex = goal_vertices_[j];

		// backup
		BOOST_FOREACH (const Edge e, boost::edges(g_))
		{
			copiedWeightProperty_[e] = weightProperty_[e];
		}

		for (int i = 0; i < num_paths; ++i)
		{
			// astar search
			boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));
			try
			{
				boost::astar_search(g_, start_vertex_,
						boost::bind(&Precomputation::costHeuristic, this, _1,
								goal_vertex),
						boost::predecessor_map(prev).visitor(
								astar_goal_visitor<Vertex>(goal_vertex)));
			} catch (found_goal&)
			{
			}
			if (prev[goal_vertex] == goal_vertex)
			{
				break;
			}

			std::vector<const robot_state::RobotState*> path;
			double path_cost = 0.0;
			for (Vertex pos = goal_vertex; prev[pos] != pos; pos = prev[pos])
			{
				path.push_back(stateProperty_[pos]);

				const std::pair<Edge, bool>& ed = boost::edge(pos, prev[pos],
						g_);
				path_cost += weightProperty_[ed.first];
				weightProperty_[ed.first] *= 2.0;
			}
			path.push_back(stateProperty_[start_vertex_]);
			std::reverse(path.begin(), path.end());

			if (i == 0)
			{
				if (path_cost > best_cost)
					break;

				best_cost = path_cost;
				paths_.clear();
			}

			paths_.push_back(
					std::make_pair<std::vector<const robot_state::RobotState*>,
							double>(path, path_cost));
		}

		// restore
		BOOST_FOREACH (const Edge e, boost::edges(g_))
		{
			weightProperty_[e] = copiedWeightProperty_[e];
		}
	}

	if (paths_.size() == 0)
	{
		ROS_INFO("Could not find a solution path\n");
		return false;
	}

	sort(paths_.begin(), paths_.end(), pathCompare);
	if (paths_.size() > num_paths)
		paths_.resize(num_paths);

	renderPaths();
	ros::WallDuration sleep_time(0.01);
	sleep_time.sleep();

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

double Precomputation::costHeuristic(Vertex u, Vertex v) const
{
	const robot_state::RobotState* s1 = stateProperty_[u];
	const robot_state::RobotState* s2 = stateProperty_[v];
	return distance(s1, s2);
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
	RED.a = 1.0;
	RED.r = 1.0;
	RED.g = 0.0;
	RED.b = 0.0;

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

	const double LONGEST_VALID_SEGMENT_LENGTH =
			PlanningParameters::getInstance()->getPrecomputationMaxValidSegmentDist();
	for (int j = 0; j < paths_.size(); ++j)
	{
		msg.points.resize(0);
		for (int i = 0; i < paths_[j].first.size() - 1; ++i)
		{
			const robot_state::RobotState* from = paths_[j].first[i];
			const robot_state::RobotState* to = paths_[j].first[i + 1];
			double dist = distance(from, to);
			int nd = ceil(dist / LONGEST_VALID_SEGMENT_LENGTH);

			for (int k = 0; k <= nd; ++k)
			{
				robot_state::RobotState test(*from);
				from->interpolate(*to, (double) k / (double) nd, test);
				test.updateLinkTransforms();

				const Eigen::Affine3d& transform = test.getGlobalLinkTransform(
					end_effector_name);

				point.x = transform.translation()(0);
				point.y = transform.translation()(1);
				point.z = transform.translation()(2);
				msg.points.push_back(point);
			}
		}
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
	const double scale = 0.005, scale2 = 0.001;
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
	LIGHT_YELLOW.b = 0.5;
	GREEN.a = 0.1;
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
		const Eigen::Affine3d& transform =
				stateProperty_[v]->getGlobalLinkTransform(end_effector_name);
		point.x = transform.translation()(0);
		point.y = transform.translation()(1);
		point.z = transform.translation()(2);

		//if (v == 500 || v == 501)
		msg.points.push_back(point);
	}
	ma.markers.push_back(msg);

	msg.id = 1;
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

		//if (!(u == 500 || u == 501 || v == 500 || v == 501))
		//continue;

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

	VisualizationManager::getInstance()->getVisualizationMarkerArrayPublisher().publish(
			ma);

	ros::WallDuration sleep_time(0.01);
	sleep_time.sleep();
}

void Precomputation::extractInitialTrajectories(
		moveit_msgs::TrajectoryConstraints& trajectory_constraints)
{
	int num_trajectories =
			PlanningParameters::getInstance()->getNumTrajectories();
	while (extractPaths(num_trajectories) == false)
	{
		createRoadmap(
				getNumMilestones()
						+ PlanningParameters::getInstance()->getPrecomputationAddMilestones());
	}
	trajectory_constraints.constraints.clear();
	int traj_constraint_begin = 0;
	const robot_state::RobotState& goal_state = *paths_[0].first.back();
	for (int c = 0; c < num_trajectories; ++c)
	{
		int num_joints = planning_scene_->getCurrentState().getVariableCount();
		std::vector<double> positions(num_joints);

		moveit_msgs::JointConstraint jc;
		int num_points = paths_[c].first.size();
		trajectory_constraints.constraints.resize(
				traj_constraint_begin + num_points);
		std::string trajectory_index_string = boost::lexical_cast<std::string>(
				c);
		for (int j = 0; j < num_points; ++j)
		{
			int point = j + traj_constraint_begin;
			if (j == 0)
				trajectory_constraints.constraints[point].name =
						trajectory_index_string;
			if (j == num_points - 1)
				trajectory_constraints.constraints[point].name = "end";

			trajectory_constraints.constraints[point].joint_constraints.resize(
					num_joints);
			for (int k = 0; k < num_joints; ++k)
			{
				jc.joint_name =
						planning_scene_->getCurrentState().getVariableNames()[k];
				jc.position = paths_[c].first[j]->getVariablePosition(k);
				trajectory_constraints.constraints[point].joint_constraints[k] =
						jc;
			}
		}
		traj_constraint_begin += num_points;
	}
}

}

