// Original code from pr2_moveit_tutorials::motion_planning_api_tutorial.cpp
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PositionConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <boost/variant/get.hpp>
#include <boost/lexical_cast.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <move_kuka/move_kuka_test.h>
#include <fstream>

using namespace std;

const int M = 8;

namespace move_kuka
{

MoveKukaTest::MoveKukaTest(const ros::NodeHandle& node_handle) :
		node_handle_(node_handle), stateProperty_(
				boost::get(vertex_state_t(), g_)), totalConnectionAttemptsProperty_(
				boost::get(vertex_total_connection_attempts_t(), g_)), successfulConnectionAttemptsProperty_(
				boost::get(vertex_successful_connection_attempts_t(), g_)), weightProperty_(
				boost::get(boost::edge_weight, g_)), copiedWeightProperty_(
				boost::get(edge_scaled_weight_t(), g_))
{

}

MoveKukaTest::~MoveKukaTest()
{
}

void MoveKukaTest::run(const std::string& group_name)
{
	// scene initialization

	robot_model_loader::RobotModelLoader robot_model_loader(
			"robot_description");
	robot_model_ = robot_model_loader.getModel();
	planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
	planning_scene_diff_publisher_ = node_handle_.advertise<
			moveit_msgs::PlanningScene>("/planning_scene", 1);
	while (planning_scene_diff_publisher_.getNumSubscribers() < 1)
	{
		ros::WallDuration sleep_t(0.5);
		sleep_t.sleep();
		ROS_INFO("Waiting planning_scene subscribers");
	}

	loadStaticScene();

	collision_detection::AllowedCollisionMatrix& acm =
			planning_scene_->getAllowedCollisionMatrixNonConst();
	acm.setEntry("environment", "segment_00", true);
	acm.setEntry("environment", "segment_0", true);
	acm.setEntry("environment", "segment_1", true);

	// planner initialization

	group_name_ = group_name;

	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
	std::string planner_plugin_name;
	if (!node_handle_.getParam("planning_plugin", planner_plugin_name))
		ROS_FATAL_STREAM("Could not find planner plugin name");
	try
	{
		planner_plugin_loader.reset(
				new pluginlib::ClassLoader<planning_interface::PlannerManager>(
						"moveit_core", "planning_interface::PlannerManager"));
	} catch (pluginlib::PluginlibException& ex)
	{
		ROS_FATAL_STREAM(
				"Exception while creating planning plugin loader " << ex.what());
	}

	try
	{
		itomp_planner_instance_.reset(
				planner_plugin_loader->createUnmanagedInstance(
						planner_plugin_name));
		if (!itomp_planner_instance_->initialize(robot_model_,
				node_handle_.getNamespace()))
			ROS_FATAL_STREAM("Could not initialize planner instance");
		ROS_INFO_STREAM(
				"Using planning interface '" << itomp_planner_instance_->getDescription() << "'");
	} catch (pluginlib::PluginlibException& ex)
	{
		const std::vector<std::string> &classes =
				planner_plugin_loader->getDeclaredClasses();
		std::stringstream ss;
		for (std::size_t i = 0; i < classes.size(); ++i)
			ss << classes[i] << " ";
		ROS_ERROR_STREAM(
				"Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl << "Available plugins: " << ss.str());
	}

	display_publisher_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>(
			"/move_group/display_planned_path", 1, true);
	vis_marker_array_publisher_ = node_handle_.advertise<
			visualization_msgs::MarkerArray>("visualization_marker_array", 100,
			true);

	ros::WallDuration sleep_time(0.01);
	sleep_time.sleep();

	///////////////////////////////////////////////////////

	moveit_msgs::DisplayTrajectory display_trajectory;
	moveit_msgs::MotionPlanResponse response;
	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;

	createRoadmap(1000);

	// Set start / goal states
	robot_state::RobotState& start_state =
			planning_scene_->getCurrentStateNonConst();
	std::vector<robot_state::RobotState> goal_states;
	goal_states.resize(10, planning_scene_->getCurrentStateNonConst());
	initStartGoalStates(start_state, goal_states);

	addStartState(start_state);
	addGoalStates(goal_states);
	renderPRMGraph();

	while (extractPaths(M) == false)
	{
		createRoadmap(states_.size() + 1000);
	}

	// generate trajectories for optimization
	req.trajectory_constraints.constraints.clear();
	int traj_constraint_begin = 0;
	const robot_state::RobotState& goal_state = *paths_[0].first.back();
	for (int c = 0; c < M; ++c)
	{
		int num_joints = start_state.getVariableCount();
		std::vector<double> positions(num_joints);

		moveit_msgs::JointConstraint jc;
		int num_points = paths_[c].first.size();
		req.trajectory_constraints.constraints.resize(
				traj_constraint_begin + num_points);
		std::string trajectory_index_string = boost::lexical_cast < std::string
				> (c);
		for (int j = 0; j < num_points; ++j)
		{
			int point = j + traj_constraint_begin;
			if (j == 0)
				req.trajectory_constraints.constraints[point].name =
						trajectory_index_string;
			if (j == num_points - 1)
				req.trajectory_constraints.constraints[point].name = "end";

			req.trajectory_constraints.constraints[point].joint_constraints.resize(
					num_joints);
			for (int k = 0; k < num_joints; ++k)
			{
				jc.joint_name = start_state.getVariableNames()[k];
				jc.position = paths_[c].first[j]->getVariablePosition(k);
				req.trajectory_constraints.constraints[point].joint_constraints[k] =
						jc;
			}
		}
		traj_constraint_begin += num_points;
	}

	// trajectory optimization using ITOMP
	plan(req, res, start_state, goal_state);
	res.getMessage(response);

	// display trajectories
	display_trajectory.trajectory_start = response.trajectory_start;
	display_trajectory.trajectory.push_back(response.trajectory);
	display_publisher_.publish(display_trajectory);

	// clean up
	itomp_planner_instance_.reset();
	planning_scene_.reset();
	robot_model_.reset();

	sleep_time.sleep();
	ROS_INFO("Done");
}

void MoveKukaTest::initStartGoalStates(robot_state::RobotState& start_state,
		std::vector<robot_state::RobotState>& goal_states)
{
	std::map<std::string, double> values;
	const robot_state::JointModelGroup* joint_model_group =
			start_state.getJointModelGroup(group_name_);

	std::vector<robot_state::RobotState> states(2, start_state);
	const double INV_SQRT_2 = 1.0 / sqrt(2.0);
	double EE_CONSTRAINTS[][7] =
	{
	{ .2, .10, 1.2, -0.5, 0.5, -0.5, 0.5 },
	{ .15, .2, .85 + .1, -INV_SQRT_2, 0, 0, INV_SQRT_2 }, };

	Eigen::Affine3d goal_transform[2];
	Eigen::Affine3d transform_1_inv =
			robot_model_->getLinkModel("tcp_1_link")->getJointOriginTransform().inverse();

	for (int i = 0; i < 2; ++i)
	{
		EE_CONSTRAINTS[i][0] -= 5.4 * 0.1;
		EE_CONSTRAINTS[i][1] -= 1.9 * 0.1;
		EE_CONSTRAINTS[i][2] -= 4.16 * 0.1;

		EE_CONSTRAINTS[i][0] = -EE_CONSTRAINTS[i][0];
		EE_CONSTRAINTS[i][1] = -EE_CONSTRAINTS[i][1];

		EE_CONSTRAINTS[i][0] += 0.3;

		Eigen::Vector3d pos(EE_CONSTRAINTS[i][0], EE_CONSTRAINTS[i][1],
				EE_CONSTRAINTS[i][2]);
		drawEndeffectorPosition(i, pos);

		Eigen::Vector3d trans = Eigen::Vector3d(EE_CONSTRAINTS[i][0],
				EE_CONSTRAINTS[i][1], EE_CONSTRAINTS[i][2]);
		Eigen::Quaterniond rot = Eigen::Quaterniond(EE_CONSTRAINTS[i][6],
				EE_CONSTRAINTS[i][3], EE_CONSTRAINTS[i][4],
				EE_CONSTRAINTS[i][5]);

		goal_transform[i].linear() = rot.toRotationMatrix();
		goal_transform[i].translation() = trans;

		goal_transform[i] = goal_transform[i] * transform_1_inv;

		states[i].update();
		computeIKState(states[i], goal_transform[i]);
	}

	start_state = states[0];
	robot_state::RobotState& goal_state = states[1];

	renderStartGoalStates(start_state, goal_state);

	for (int j = 0; j < goal_states.size(); ++j)
	{
		computeIKState(goal_states[j], goal_transform[1], true);
	}
}

bool MoveKukaTest::isStateSingular(robot_state::RobotState& state)
{
	// check singularity
	Eigen::MatrixXd jacobianFull = (state.getJacobian(
			planning_scene_->getRobotModel()->getJointModelGroup(group_name_)));
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobianFull);
	int rows = svd.singularValues().rows();
	double min_value = svd.singularValues()(rows - 1);

	const double threshold = 1e-3;
	if (min_value < threshold)
		return true;
	else
		return false;
}

void MoveKukaTest::plan(planning_interface::MotionPlanRequest& req,
		planning_interface::MotionPlanResponse& res,
		const robot_state::RobotState& start_state,
		const robot_state::RobotState& goal_state)
{
	const robot_state::JointModelGroup* joint_model_group =
			start_state.getJointModelGroup(group_name_);
	req.group_name = group_name_;
	req.allowed_planning_time = 3000.0;

	// Copy from start_state to req.start_state
	unsigned int num_joints = start_state.getVariableCount();
	req.start_state.joint_state.name = start_state.getVariableNames();
	req.start_state.joint_state.position.resize(num_joints);
	req.start_state.joint_state.velocity.resize(num_joints);
	req.start_state.joint_state.effort.resize(num_joints);
	memcpy(&req.start_state.joint_state.position[0],
			start_state.getVariablePositions(), sizeof(double) * num_joints);
	if (start_state.hasVelocities())
		memcpy(&req.start_state.joint_state.velocity[0],
				start_state.getVariableVelocities(),
				sizeof(double) * num_joints);
	else
		memset(&req.start_state.joint_state.velocity[0], 0,
				sizeof(double) * num_joints);
	if (start_state.hasAccelerations())
		memcpy(&req.start_state.joint_state.effort[0],
				start_state.getVariableAccelerations(),
				sizeof(double) * num_joints);
	else
		memset(&req.start_state.joint_state.effort[0], 0,
				sizeof(double) * num_joints);

	// goal state
	moveit_msgs::Constraints joint_goal =
			kinematic_constraints::constructGoalConstraints(goal_state,
					joint_model_group);
	req.goal_constraints.clear();
	req.goal_constraints.push_back(joint_goal);

	planning_interface::PlanningContextPtr context =
			itomp_planner_instance_->getPlanningContext(planning_scene_, req,
					res.error_code_);
	context->solve(res);
	if (res.error_code_.val != res.error_code_.SUCCESS)
	{
		ROS_ERROR("Could not compute plan successfully");
		return;
	}
}

void MoveKukaTest::loadStaticScene()
{
	moveit_msgs::PlanningScene planning_scene_msg;
	std::string environment_file;
	std::vector<double> environment_position;

	node_handle_.param<std::string>("/itomp_planner/environment_model",
			environment_file, "");

	if (!environment_file.empty())
	{
		double scale;
		node_handle_.param("/itomp_planner/environment_model_scale", scale,
				1.0);
		environment_position.resize(3, 0);
		if (node_handle_.hasParam("/itomp_planner/environment_model_position"))
		{
			XmlRpc::XmlRpcValue segment;
			node_handle_.getParam("/itomp_planner/environment_model_position",
					segment);
			if (segment.getType() == XmlRpc::XmlRpcValue::TypeArray)
			{
				int size = segment.size();
				for (int i = 0; i < size; ++i)
				{
					double value = segment[i];
					environment_position[i] = value;
				}
			}
		}

		// Collision object
		moveit_msgs::CollisionObject collision_object;
		collision_object.header.frame_id = robot_model_->getModelFrame();
		collision_object.id = "environment";
		geometry_msgs::Pose pose;
		pose.position.x = environment_position[0];
		pose.position.y = environment_position[1];
		pose.position.z = environment_position[2];
		pose.orientation.x = 0.0;
		pose.orientation.y = 0.0;
		pose.orientation.z = 0.0;
		pose.orientation.w = 1.0;

		shapes::Mesh* shape = shapes::createMeshFromResource(environment_file,
				Eigen::Vector3d(scale, scale, scale));
		shapes::ShapeMsg mesh_msg;
		shapes::constructMsgFromShape(shape, mesh_msg);
		shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

		collision_object.meshes.push_back(mesh);
		collision_object.mesh_poses.push_back(pose);

		collision_object.operation = collision_object.ADD;
		//moveit_msgs::PlanningScene planning_scene_msg;
		planning_scene_msg.world.collision_objects.push_back(collision_object);
		planning_scene_msg.is_diff = true;
		planning_scene_->setPlanningSceneDiffMsg(planning_scene_msg);
	}

	planning_scene_diff_publisher_.publish(planning_scene_msg);
}

void MoveKukaTest::renderStartGoalStates(robot_state::RobotState& start_state,
		robot_state::RobotState& goal_state)
{
	// display start / goal states
	int num_variables = start_state.getVariableNames().size();
	static ros::Publisher start_state_display_publisher =
			node_handle_.advertise<moveit_msgs::DisplayRobotState>(
					"/move_itomp/display_start_state", 1, true);
	moveit_msgs::DisplayRobotState disp_start_state;
	disp_start_state.state.joint_state.header.frame_id =
			robot_model_->getModelFrame();
	disp_start_state.state.joint_state.name = start_state.getVariableNames();
	disp_start_state.state.joint_state.position.resize(num_variables);
	memcpy(&disp_start_state.state.joint_state.position[0],
			start_state.getVariablePositions(), sizeof(double) * num_variables);
	disp_start_state.highlight_links.clear();
	const std::vector<std::string>& link_model_names =
			robot_model_->getLinkModelNames();
	for (unsigned int i = 0; i < link_model_names.size(); ++i)
	{
		std_msgs::ColorRGBA color;

		color.a = 0.5;
		color.r = 0.0;
		color.g = 1.0;
		color.b = 0.5;

		moveit_msgs::ObjectColor obj_color;
		obj_color.id = link_model_names[i];
		obj_color.color = color;
		disp_start_state.highlight_links.push_back(obj_color);
	}
	start_state_display_publisher.publish(disp_start_state);

	static ros::Publisher goal_state_display_publisher = node_handle_.advertise<
			moveit_msgs::DisplayRobotState>("/move_itomp/display_goal_state", 1,
			true);
	moveit_msgs::DisplayRobotState disp_goal_state;
	disp_goal_state.state.joint_state.header.frame_id =
			robot_model_->getModelFrame();
	disp_goal_state.state.joint_state.name = goal_state.getVariableNames();
	disp_goal_state.state.joint_state.position.resize(num_variables);
	memcpy(&disp_goal_state.state.joint_state.position[0],
			goal_state.getVariablePositions(), sizeof(double) * num_variables);
	disp_goal_state.highlight_links.clear();
	for (int i = 0; i < link_model_names.size(); ++i)
	{
		std_msgs::ColorRGBA color;
		color.a = 0.5;
		color.r = 0.0;
		color.g = 0.5;
		color.b = 1.0;
		moveit_msgs::ObjectColor obj_color;
		obj_color.id = link_model_names[i];
		obj_color.color = color;
		disp_goal_state.highlight_links.push_back(obj_color);
	}
	goal_state_display_publisher.publish(disp_goal_state);
}

bool MoveKukaTest::isStateCollide(const robot_state::RobotState& state)
{
	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;
	collision_request.verbose = false;
	collision_request.contacts = false;

	planning_scene_->checkCollisionUnpadded(collision_request, collision_result,
			state);

	return collision_result.collision;
}

void MoveKukaTest::computeIKState(robot_state::RobotState& ik_state,
		const Eigen::Affine3d& end_effector_state, bool rand)
{
	// compute waypoint ik solutions

	const robot_state::JointModelGroup* joint_model_group =
			ik_state.getJointModelGroup(group_name_);

	kinematics::KinematicsQueryOptions options;
	options.return_approximate_solution = false;
	bool found_ik = false;

	robot_state::RobotState org_start(ik_state);
	int i = 0;

	if (rand)
		ik_state.setToRandomPositionsNearBy(joint_model_group, org_start,
				log(-3) / log(10));

	while (true)
	{
		found_ik = ik_state.setFromIK(joint_model_group, end_effector_state, 10,
				0.1, moveit::core::GroupStateValidityCallbackFn(), options);
		ik_state.update();

		found_ik &= !isStateCollide(ik_state);

		if (found_ik && isStateSingular(ik_state))
			found_ik = false;

		if (found_ik)
			break;

		++i;

		double dist = log(-3 + 0.001 * i) / log(10);

		ik_state.setToRandomPositionsNearBy(joint_model_group, org_start, dist);
	}

	if (found_ik)
	{
		//ROS_INFO("IK solution found after %d trials", i + 1);
	}
	else
	{
		ROS_INFO("Could not find IK solution");
	}
}

void MoveKukaTest::drawEndeffectorPosition(int id,
		const Eigen::Vector3d& position)
{
	const double trajectory_color_diff = 0.33;
	const double scale = 0.02;
	const int marker_step = 1;

	visualization_msgs::Marker::_color_type BLUE, LIGHT_YELLOW;
	visualization_msgs::Marker::_color_type RED, LIGHT_RED;
	RED.a = 1.0;
	RED.r = 1.0;
	RED.g = 0.0;
	RED.b = 0.0;
	BLUE.a = 1.0;
	BLUE.r = 0.5;
	BLUE.g = 0.5;
	BLUE.b = 1.0;
	LIGHT_RED = RED;
	LIGHT_RED.g = 0.5;
	LIGHT_RED.b = 0.5;
	LIGHT_YELLOW = BLUE;
	LIGHT_YELLOW.b = 0.5;

	visualization_msgs::Marker msg;
	msg.header.frame_id = robot_model_->getModelFrame();
	msg.header.stamp = ros::Time::now();
	msg.ns = "cartesian_traj";
	msg.type = visualization_msgs::Marker::CUBE_LIST;
	msg.action = visualization_msgs::Marker::ADD;

	msg.scale.x = scale;
	msg.scale.y = scale;
	msg.scale.z = scale;

	msg.id = id;
	msg.color = BLUE;

	msg.points.resize(0);
	geometry_msgs::Point point;
	point.x = position(0);
	point.y = position(1);
	point.z = position(2);
	msg.points.push_back(point);

	visualization_msgs::MarkerArray ma;
	ma.markers.push_back(msg);
	vis_marker_array_publisher_.publish(ma);
}

void MoveKukaTest::drawPath(int id, const Eigen::Vector3d& from,
		const Eigen::Vector3d& to)
{
	const double trajectory_color_diff = 0.33;
	const double scale = 0.005;
	const int marker_step = 1;

	visualization_msgs::Marker::_color_type BLUE, LIGHT_YELLOW;
	visualization_msgs::Marker::_color_type RED, LIGHT_RED;
	RED.a = 1.0;
	RED.r = 1.0;
	RED.g = 0.0;
	RED.b = 0.0;
	BLUE.a = 1.0;
	BLUE.r = 0.5;
	BLUE.g = 0.5;
	BLUE.b = 1.0;
	LIGHT_RED = RED;
	LIGHT_RED.g = 0.5;
	LIGHT_RED.b = 0.5;
	LIGHT_YELLOW = BLUE;
	LIGHT_YELLOW.b = 0.5;

	visualization_msgs::Marker msg;
	msg.header.frame_id = robot_model_->getModelFrame();
	msg.header.stamp = ros::Time::now();
	msg.ns = "cartesian_traj";
	msg.type = visualization_msgs::Marker::LINE_LIST;
	msg.action = visualization_msgs::Marker::ADD;

	msg.scale.x = scale;
	msg.scale.y = scale;
	msg.scale.z = scale;

	msg.id = id;
	msg.color = BLUE;

	msg.points.resize(0);
	geometry_msgs::Point point;
	point.x = from(0) - 0.001;
	point.y = from(1);
	point.z = from(2);
	msg.points.push_back(point);
	point.x = to(0) - 0.001;
	point.y = to(1);
	point.z = to(2);
	msg.points.push_back(point);

	visualization_msgs::MarkerArray ma;
	ma.markers.push_back(msg);
	vis_marker_array_publisher_.publish(ma);
}

////////////////////////////////////////////////////////////////////////////////
// 10 neighbor + 1 vertex itself
const int NN = 10 + 1;
const double LONGEST_VALID_SEGMENT_LENGTH = 0.3;

static int edge_index = 0;

void MoveKukaTest::growRoadmap(int new_milestones)
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
			if (isStateCollide(*state) == false)
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
void MoveKukaTest::expandRoadmap(int new_milestones)
{
	int old_milestones = states_.size();

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

		while (true)
		{

			state->setToRandomPositionsNearBy(joint_model_group, *s,
					LONGEST_VALID_SEGMENT_LENGTH * 0.5);
			state->updateCollisionBodyTransforms();

			if (isStateCollide(*state) == false)
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

void MoveKukaTest::createRoadmap(int milestones)
{
	ROS_INFO("Create roadmap of %d milestones", milestones);
	while (states_.size() < milestones)
	{
		growRoadmap(100);
		expandRoadmap(400);

		renderPRMGraph();
	}
}

bool MoveKukaTest::localPlanning(const robot_state::RobotState& from,
		const robot_state::RobotState& to, double distance)
{
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

			if (isStateCollide(test))
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

void MoveKukaTest::addStartState(const robot_state::RobotState& from)
{
	int dim = from.getVariableCount();

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

void MoveKukaTest::addGoalStates(const std::vector<robot_state::RobotState>& to)
{
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

bool MoveKukaTest::extractPaths(int num_paths)
{
	paths_.clear();

	double best_cost = numeric_limits<double>::max();
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
						boost::bind(&MoveKukaTest::costHeuristic, this, _1,
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
		ROS_INFO("Could not find solution path\n");
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

double MoveKukaTest::distance(const robot_state::RobotState* s1,
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

double MoveKukaTest::costHeuristic(Vertex u, Vertex v) const
{
	const robot_state::RobotState* s1 = stateProperty_[u];
	const robot_state::RobotState* s2 = stateProperty_[v];
	return distance(s1, s2);
}

void MoveKukaTest::renderPaths()
{
	const double trajectory_color_diff = 0.33;
	const double scale = 0.005, scale2 = 0.001;
	;
	const int marker_step = 1;

	visualization_msgs::MarkerArray ma;
	visualization_msgs::Marker::_color_type RED;
	RED.a = 1.0;
	RED.r = 1.0;
	RED.g = 0.0;
	RED.b = 0.0;

	visualization_msgs::Marker msg;
	msg.header.frame_id = robot_model_->getModelFrame();
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
						"tcp_1_link");

				point.x = transform.translation()(0);
				point.y = transform.translation()(1);
				point.z = transform.translation()(2);
				msg.points.push_back(point);
			}
		}
		ma.markers.push_back(msg);
		++msg.id;
	}

	vis_marker_array_publisher_.publish(ma);
}

void MoveKukaTest::renderPRMGraph()
{
	const double trajectory_color_diff = 0.33;
	const double scale = 0.005, scale2 = 0.001;
	;
	const int marker_step = 1;

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
	msg.header.frame_id = robot_model_->getModelFrame();
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
				stateProperty_[v]->getGlobalLinkTransform("tcp_1_link");
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
				stateProperty_[u]->getGlobalLinkTransform("tcp_1_link");

		point.x = transform.translation()(0);
		point.y = transform.translation()(1);
		point.z = transform.translation()(2);
		msg.points.push_back(point);

		const Eigen::Affine3d& transform2 =
				stateProperty_[v]->getGlobalLinkTransform("tcp_1_link");
		point.x = transform2.translation()(0);
		point.y = transform2.translation()(1);
		point.z = transform2.translation()(2);
		msg.points.push_back(point);
	}

	ma.markers.push_back(msg);

	vis_marker_array_publisher_.publish(ma);

	ros::WallDuration sleep_time(0.01);
	sleep_time.sleep();
}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_itomp");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle("~");

	move_kuka::MoveKukaTest* move_kuka = new move_kuka::MoveKukaTest(
			node_handle);
	move_kuka->run("lower_body");
	delete move_kuka;

	return 0;
}
