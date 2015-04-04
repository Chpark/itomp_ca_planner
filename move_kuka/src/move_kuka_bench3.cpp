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
#include <move_kuka/move_kuka_bench3.h>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <sched.h>
#include <limits>

using namespace std;

const int M = 8;
static int PLANNER_INDEX = -1;

namespace move_kuka
{

MoveKukaTest::MoveKukaTest(const ros::NodeHandle& node_handle) :
    node_handle_(node_handle)
{

}

MoveKukaTest::~MoveKukaTest()
{
}

void MoveKukaTest::run(const std::string& group_name)
{
	// scene initialization
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model_ = robot_model_loader.getModel();
	planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    planning_scene_diff_publisher_ = node_handle_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
	while (planning_scene_diff_publisher_.getNumSubscribers() < 1)
	{
		ros::WallDuration sleep_t(0.5);
		sleep_t.sleep();
		ROS_INFO("Waiting planning_scene subscribers");
	}

	loadStaticScene();

    collision_detection::AllowedCollisionMatrix& acm = planning_scene_->getAllowedCollisionMatrixNonConst();
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
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
                                        "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
	{
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
	}

	try
	{
        cpu_set_t mask;
        if (sched_getaffinity(0, sizeof(cpu_set_t), &mask) != 0)
            ROS_ERROR("sched_getaffinity failed");
        itomp_planner_instance_.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (sched_setaffinity(0, sizeof(cpu_set_t), &mask) != 0)
            ROS_ERROR("sched_setaffinity failed");

        if (!itomp_planner_instance_->initialize(robot_model_, node_handle_.getNamespace()))
			ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << itomp_planner_instance_->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
	{
        const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
		std::stringstream ss;
		for (std::size_t i = 0; i < classes.size(); ++i)
			ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl << "Available plugins: " << ss.str());
	}

    display_publisher_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    vis_marker_array_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100, true);

	ros::WallDuration sleep_time(0.01);
	sleep_time.sleep();

	///////////////////////////////////////////////////////
    for (int i = 0; i < 11; ++i)
    {
        std::stringstream ss;
        ss << "/move_itomp/display_state" << i;
        opt_state_display_publisher_[i] = node_handle_.advertise<moveit_msgs::DisplayRobotState>(ss.str().c_str(), 10, true);
    }

    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory.resize(1);
    ros::WallDuration sleep_t(5.0);

    readOptimization(1);
    readOptimization(2);
    readOptimization(3);

    playTrajectory(0);
    drawOptimization(1);

    //sleep_t.sleep();

    playTrajectory(1);
    drawOptimization(2);

    //sleep_t.sleep();
    playTrajectory(2);
    drawOptimization(3);

    //sleep_t.sleep();

    playTrajectory(3);
    drawOptimization(4);



    for (int i = 0; i < 100; ++i)
        sleep_t.sleep();


    /*
    for (int i = 0; i < 4; ++i)
    {
        std::stringstream ss;
        ss << i << ".txt";
        readTrajectory(display_trajectory.trajectory[0], ss.str());
        display_publisher_.publish(display_trajectory);
        sleep_t.sleep();
    }
    display_publisher_.publish(display_trajectory);
    sleep_t.sleep();
    drawObstacle();
    */
    display_trajectory.trajectory.clear();





    for (int index = 3; index < 4; ++index)
    {
        moveit_msgs::MotionPlanResponse response;
        planning_interface::MotionPlanRequest req;
        planning_interface::MotionPlanResponse res;

        // Set start / goal states
        initStartGoalStates(req, index);

        // trajectory optimization using ITOMP
        bool use_itomp = (planner_plugin_name.find("Itomp") != string::npos);
        plan(req, res, use_itomp);
        res.getMessage(response);

        last_goal_state_.reset(new robot_state::RobotState(res.trajectory_->getLastWayPoint()));

        // display trajectories
        //if (index == 0)
        //  display_trajectory.trajectory_start = response.trajectory_start;
        display_trajectory.trajectory.push_back(response.trajectory);

        // print trajectory
        std::stringstream ss;
        ss.precision(std::numeric_limits<double>::digits10);
        for (unsigned int i = 0; i < response.trajectory.joint_trajectory.points.size(); ++i)
        {
            const trajectory_msgs::JointTrajectoryPoint& point = response.trajectory.joint_trajectory.points[i];
            ss << i << " : " << point.time_from_start.toSec() << " : ";
            for (unsigned int j = 0; j < point.positions.size(); ++j)
                ss << std::fixed << point.positions[j] << " ";
            ss << endl;
        }
        ROS_INFO(ss.str().c_str());

    }
    display_publisher_.publish(display_trajectory);





	// clean up
	itomp_planner_instance_.reset();
	planning_scene_.reset();
	robot_model_.reset();

	sleep_time.sleep();
	ROS_INFO("Done");
}

void MoveKukaTest::initStartGoalStates(planning_interface::MotionPlanRequest& req, int index)
{
    const int num_waypoints = 4;
    double EE_CONSTRAINTS[][7] =
    {
        { 6.25, 3.5, 7.0, 0.5, 0.5, 0.5, 0.5 },
        { 6.25, 3.5, 10.0, 0.5, 0.5, 0.5, 0.5 },
        { 6.25, 0.5, 10.0, 0.5, 0.5, 0.5, 0.5 },
        { 6.25, 0.5, 7, 0.5, 0.5, 0.5, 0.5 },
    };

    for (int i = 0; i < num_waypoints; ++i)
    {
        EE_CONSTRAINTS[i][0] *= 0.1;
        EE_CONSTRAINTS[i][1] *= 0.1;
        EE_CONSTRAINTS[i][2] *= 0.1;
    }

    robot_state::RobotState& start_state = (!last_goal_state_) ? planning_scene_->getCurrentStateNonConst() : *last_goal_state_;

    Eigen::Affine3d goal_transform[num_waypoints];
    for (int i = 0; i < num_waypoints; ++i)
    {
        Eigen::Vector3d trans = Eigen::Vector3d(EE_CONSTRAINTS[i][0], EE_CONSTRAINTS[i][1], EE_CONSTRAINTS[i][2]);
        Eigen::Quaterniond rot = Eigen::Quaterniond(EE_CONSTRAINTS[i][6], EE_CONSTRAINTS[i][3], EE_CONSTRAINTS[i][4], EE_CONSTRAINTS[i][5]);

        goal_transform[i].linear() = rot.toRotationMatrix();
        goal_transform[i].translation() = trans;
    }

    for (int i = 0; i < num_waypoints; ++i)
        drawPath(i, goal_transform[i].translation(), goal_transform[(i + 1) % num_waypoints].translation());
    ros::WallDuration sleep_t(0.001);
    sleep_t.sleep();


    std::vector<robot_state::RobotState> states(num_waypoints, start_state);
    for (int i = 0; i < num_waypoints; ++i)
    {
        states[i].update();
        computeIKState(states[i], goal_transform[i]);
    }

    robot_state::RobotState& from_state = (!last_goal_state_) ? states[index] : *last_goal_state_;
    robot_state::RobotState& to_state = states[(index + 1) % num_waypoints];

    renderStartGoalStates(from_state, to_state);

    // set start state
    // Copy from start_state to req.start_state
    unsigned int num_joints = from_state.getVariableCount();
    req.start_state.joint_state.name = from_state.getVariableNames();
    req.start_state.joint_state.position.resize(num_joints);
    req.start_state.joint_state.velocity.resize(num_joints);
    req.start_state.joint_state.effort.resize(num_joints);
    memcpy(&req.start_state.joint_state.position[0], from_state.getVariablePositions(), sizeof(double) * num_joints);
    if (start_state.hasVelocities())
        memcpy(&req.start_state.joint_state.velocity[0], from_state.getVariableVelocities(), sizeof(double) * num_joints);
    else
        memset(&req.start_state.joint_state.velocity[0], 0, sizeof(double) * num_joints);
    if (start_state.hasAccelerations())
        memcpy(&req.start_state.joint_state.effort[0], from_state.getVariableAccelerations(), sizeof(double) * num_joints);
    else
        memset(&req.start_state.joint_state.effort[0], 0, sizeof(double) * num_joints);

    // set goal state
    req.goal_constraints.clear();

    const Eigen::Affine3d& transform = goal_transform[(index + 1) % num_waypoints];
    Eigen::Vector3d trans = transform.translation();
    Eigen::Quaterniond rot = Eigen::Quaterniond(transform.linear());

    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = robot_model_->getModelFrame();
    goal_pose.pose.position.x = trans(0);
    goal_pose.pose.position.y = trans(1);
    goal_pose.pose.position.z = trans(2);
    goal_pose.pose.orientation.x = rot.x();
    goal_pose.pose.orientation.y = rot.y();
    goal_pose.pose.orientation.z = rot.z();
    goal_pose.pose.orientation.w = rot.w();
    std::string endeffector_name = "tcp_2_link";
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);
    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(endeffector_name, goal_pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);

    // set path constraint
    moveit_msgs::PositionConstraint pc;
    pc.link_name = endeffector_name;
    pc.header.frame_id = robot_model_->getModelFrame();
    pc.weight = 1.0;
    pc.target_point_offset.x = EE_CONSTRAINTS[index % num_waypoints][0];
    pc.target_point_offset.y = EE_CONSTRAINTS[index % num_waypoints][1];
    pc.target_point_offset.z = EE_CONSTRAINTS[index % num_waypoints][2];
    req.path_constraints.position_constraints.push_back(pc);
    pc.target_point_offset.x = EE_CONSTRAINTS[(index + 1) % num_waypoints][0];
    pc.target_point_offset.y = EE_CONSTRAINTS[(index + 1) % num_waypoints][1];
    pc.target_point_offset.z = EE_CONSTRAINTS[(index + 1) % num_waypoints][2];
    req.path_constraints.position_constraints.push_back(pc);

    // set
    const double start[] = {0.2239734837,	-0.9555846276,	-0.230254601,	-2.0531476191,	1.3632906805,	1.5732336713,	0.4518983512};
    for (int i = 0; i < 7; ++i)
        req.start_state.joint_state.position[i] = start[i];

    std::stringstream ss;
    ss << "Start state : ";
    ss.precision(std::numeric_limits<double>::digits10);
    for (int i = 0; i < from_state.getVariableCount(); ++i)
        ss << std::fixed << from_state.getVariablePositions()[i] << " ";
    ROS_INFO(ss.str().c_str());
}

bool MoveKukaTest::isStateSingular(robot_state::RobotState& state)
{
	// check singularity
    Eigen::MatrixXd jacobianFull = (state.getJacobian(planning_scene_->getRobotModel()->getJointModelGroup(group_name_)));
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
                        bool use_itomp)
{
	req.group_name = group_name_;
    req.allowed_planning_time = 300.0;
    req.num_planning_attempts = 1;

    req.workspace_parameters.min_corner.x = -1.0;
    req.workspace_parameters.min_corner.y = -1.25;
    req.workspace_parameters.min_corner.z = -0.25;
    req.workspace_parameters.max_corner.x = 1.0;
    req.workspace_parameters.max_corner.y = 0.75;
    req.workspace_parameters.max_corner.z = 1.75;

    ROS_INFO("Available planners :");
    std::vector<std::string> algorithms;
    itomp_planner_instance_->getPlanningAlgorithms(algorithms);
    for (unsigned int i = 0; i < algorithms.size(); ++i)
    {
        if (algorithms[i].find(group_name_) != std::string::npos)
            ROS_INFO("%d : %s", i, algorithms[i].c_str());
    }

    if (PLANNER_INDEX != -1)
        req.planner_id = algorithms[PLANNER_INDEX];

	planning_interface::PlanningContextPtr context =
        itomp_planner_instance_->getPlanningContext(planning_scene_, req, res.error_code_);

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

    node_handle_.param<std::string>("/itomp_planner/environment_model", environment_file, "");

	if (!environment_file.empty())
	{
		double scale;
        node_handle_.param("/itomp_planner/environment_model_scale", scale, 1.0);
		environment_position.resize(3, 0);
		if (node_handle_.hasParam("/itomp_planner/environment_model_position"))
		{
			XmlRpc::XmlRpcValue segment;
            node_handle_.getParam("/itomp_planner/environment_model_position", segment);
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
        node_handle_.advertise<moveit_msgs::DisplayRobotState>("/move_itomp/display_start_state", 1, true);
	moveit_msgs::DisplayRobotState disp_start_state;
    disp_start_state.state.joint_state.header.frame_id = robot_model_->getModelFrame();
	disp_start_state.state.joint_state.name = start_state.getVariableNames();
	disp_start_state.state.joint_state.position.resize(num_variables);
    memcpy(&disp_start_state.state.joint_state.position[0], start_state.getVariablePositions(), sizeof(double) * num_variables);
	disp_start_state.highlight_links.clear();
    const std::vector<std::string>& link_model_names = robot_model_->getLinkModelNames();
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
            moveit_msgs::DisplayRobotState>("/move_itomp/display_goal_state", 1, true);
	moveit_msgs::DisplayRobotState disp_goal_state;
    disp_goal_state.state.joint_state.header.frame_id = robot_model_->getModelFrame();
	disp_goal_state.state.joint_state.name = goal_state.getVariableNames();
	disp_goal_state.state.joint_state.position.resize(num_variables);
    memcpy(&disp_goal_state.state.joint_state.position[0], goal_state.getVariablePositions(), sizeof(double) * num_variables);
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

    planning_scene_->checkCollisionUnpadded(collision_request, collision_result, state);

	return collision_result.collision;
}

void MoveKukaTest::renderState(const robot_state::RobotState& state)
{
    std_msgs::ColorRGBA color;
    color.a = 0.5;
    color.r = 1.0;
    color.g = 0.5;
    color.b = 0.5;

    int num_variables = state.getVariableNames().size();
    static ros::Publisher state_display_publisher = node_handle_.advertise<
            moveit_msgs::DisplayRobotState>("/move_itomp/display_state", 1,
                                            true);
    moveit_msgs::DisplayRobotState disp_state;
    disp_state.state.joint_state.header.frame_id =
        robot_model_->getModelFrame();
    disp_state.state.joint_state.name = state.getVariableNames();
    disp_state.state.joint_state.position.resize(num_variables);
    memcpy(&disp_state.state.joint_state.position[0],
           state.getVariablePositions(), sizeof(double) * num_variables);
    disp_state.highlight_links.clear();
    const std::vector<std::string>& link_model_names =
        robot_model_->getLinkModelNames();
    for (unsigned int i = 0; i < link_model_names.size(); ++i)
    {
        moveit_msgs::ObjectColor obj_color;
        obj_color.id = link_model_names[i];
        obj_color.color = color;
        disp_state.highlight_links.push_back(obj_color);
    }
    state_display_publisher.publish(disp_state);
}

void MoveKukaTest::computeIKState(robot_state::RobotState& ik_state,
                                  const Eigen::Affine3d& end_effector_state, bool rand)
{
	// compute waypoint ik solutions

    const robot_state::JointModelGroup* joint_model_group = ik_state.getJointModelGroup(group_name_);

	kinematics::KinematicsQueryOptions options;
	options.return_approximate_solution = false;
	bool found_ik = false;

	robot_state::RobotState org_start(ik_state);
	int i = 0;

	if (rand)
        ik_state.setToRandomPositionsNearBy(joint_model_group, org_start, log(-3) / log(10));

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

        renderState(ik_state);

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

void MoveKukaTest::drawEndeffectorPosition(int id, const Eigen::Vector3d& position)
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

void MoveKukaTest::drawPath(int id, const Eigen::Vector3d& from, const Eigen::Vector3d& to)
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

void MoveKukaTest::drawObstacle()
{
    visualization_msgs::Marker::_color_type GRAY;
    GRAY.a = 1.0;
    GRAY.r = 0.5;
    GRAY.g = 0.5;
    GRAY.b = 0.5;

    for (int i = 0; i < 300; ++i)
    {
        visualization_msgs::Marker msg;
        msg.header.frame_id = robot_model_->getModelFrame();
        msg.header.stamp = ros::Time::now();
        msg.ns = "obstacle";
        msg.type = visualization_msgs::Marker::CUBE;
        msg.action = (i==0) ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::MODIFY;
        msg.scale.x = 0.2;
        msg.scale.y = 1.0;
        msg.scale.z = 0.15;
        msg.id = 0;
        msg.color = GRAY;
        msg.pose.position.x = 2.5 - i * 0.01;
        msg.pose.position.y = -0.75;
        msg.pose.position.z = 0.5;
        msg.lifetime = ros::Duration(5.0);

        visualization_msgs::MarkerArray ma;
        ma.markers.push_back(msg);
        vis_marker_array_publisher_.publish(ma);

        ros::WallDuration sleep_t(0.05);
        sleep_t.sleep();
    }
}

void MoveKukaTest::readTrajectory(moveit_msgs::RobotTrajectory& trajectory, const std::string& filename)
{
    std::ifstream myfile (filename.c_str());
    if (myfile.is_open())
    {
        double v;

        trajectory.joint_trajectory.header.frame_id = robot_model_->getModelFrame();
        trajectory.joint_trajectory.header.seq = 0;
        trajectory.joint_trajectory.joint_names.clear();
        for (int i = 0; i < 7; ++i)
            trajectory.joint_trajectory.joint_names.push_back(robot_model_->getJointOfVariable(i)->getName());

        trajectory.joint_trajectory.points.resize(101);
        for (int i = 0; i < 101; ++i)
        {
            trajectory.joint_trajectory.points[i].positions.resize(7);
            trajectory.joint_trajectory.points[i].velocities.resize(7);
            for (int j = 0; j < 7; ++j)
            {
                myfile >> v;
                trajectory.joint_trajectory.points[i].positions[j] = v;
            }
            trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(0.05 * (i));
        }

        myfile.close();
    }
}

void MoveKukaTest::readOptimization(int index)
{
    Eigen::MatrixXd mat(111, 7);

    std::stringstream ss;
    ss << "best" << index << ".txt";
    std::ifstream myfile (ss.str().c_str());
    if (myfile.is_open())
    {
        double v, temp;

        while (true)
        {
            myfile >> v;

            if (myfile.eof())
                break;

            for (int i = 0; i < 111; ++i)
            {
                for (int j = 0; j < 7; ++j)
                {
                    myfile >> temp;
                    mat(i, j) = temp;
                }
            }

            optimization_data_[index].push_back(std::make_pair(v, mat));
        }

        myfile.close();
    }
}

void MoveKukaTest::drawOptimization(int index)
{
    double start_time = ros::WallTime::now().toSec();

    visualization_msgs::Marker::_color_type GRAY;
    GRAY.a = 1.0;
    GRAY.r = 0.5;
    GRAY.g = 0.5;
    GRAY.b = 0.5;

    visualization_msgs::Marker msg;
    msg.header.frame_id = robot_model_->getModelFrame();
    msg.ns = "obstacle";
    msg.type = visualization_msgs::Marker::CUBE;
    msg.scale.x = 0.2;
    msg.scale.y = 1.0;
    msg.scale.z = 0.15;
    msg.id = 0;
    msg.color = GRAY;
    msg.pose.position.x = 2.5;
    msg.pose.position.y = -0.75;
    msg.pose.position.z = 0.5;
    msg.lifetime = ros::Duration(5.0);
    msg.action = visualization_msgs::Marker::ADD;

    int pos = 0;
    while (true)
    {
        ros::Time cur_time = ros::Time::now();
        double elapsed = cur_time.toSec() - start_time;
        if (elapsed > 7.0)
            break;

        msg.header.stamp = cur_time;
        msg.pose.position.x = 4 - (double)index - elapsed / 7.0;

        if (index < 4 && pos < optimization_data_[index].size() && elapsed > optimization_data_[index][pos].first)
        {
            Eigen::MatrixXd& mat = optimization_data_[index][pos].second;

            robot_state::RobotState state = planning_scene_->getCurrentStateNonConst();
            double* p = state.getVariablePositions();
            for (int k = 5; k <= 105; k += 20)
            {
                for (int i = 0; i < 7; ++i)
                    p[i] = mat(k, i);
                state.update(true);
                renderStateForOptimization(state, (k-5)/20);
            }

            ++pos;
        }

        visualization_msgs::MarkerArray ma;
        ma.markers.push_back(msg);
        if (index > 1 || elapsed > 4.0)
            vis_marker_array_publisher_.publish(ma);

        ros::WallDuration sleep_t(0.01);
        sleep_t.sleep();

        msg.action = visualization_msgs::Marker::MODIFY;
    }
}

void MoveKukaTest::playTrajectory(int index)
{
    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory.resize(1);
    std::stringstream ss;
    ss << index << ".txt";
    readTrajectory(display_trajectory.trajectory[0], ss.str().c_str());
    display_publisher_.publish(display_trajectory);
}

void MoveKukaTest::renderStateForOptimization(const robot_state::RobotState& state, int i)
{
    std_msgs::ColorRGBA color;
    color.a = 0.25;
    color.r = 0.5;
    color.g = 1.0;
    color.b = 0.5;

    int num_variables = state.getVariableNames().size();
    moveit_msgs::DisplayRobotState disp_state;
    disp_state.state.joint_state.header.frame_id = robot_model_->getModelFrame();
    disp_state.state.joint_state.name = state.getVariableNames();
    disp_state.state.joint_state.position.resize(num_variables);
    memcpy(&disp_state.state.joint_state.position[0], state.getVariablePositions(), sizeof(double) * num_variables);
    disp_state.highlight_links.clear();
    const std::vector<std::string>& link_model_names = robot_model_->getLinkModelNames();
    for (unsigned int k = 0; k < link_model_names.size(); ++k)
    {
        moveit_msgs::ObjectColor obj_color;
        obj_color.id = link_model_names[k];
        obj_color.color = color;
        disp_state.highlight_links.push_back(obj_color);
    }
    opt_state_display_publisher_[i].publish(disp_state);
}

}

int main(int argc, char **argv)
{
    // for debug
    setbuf(stdout, NULL);

	ros::init(argc, argv, "move_itomp");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle("~");

    if (argc > 1)
    {
        PLANNER_INDEX = boost::lexical_cast<int>(argv[1]);
    }

    move_kuka::MoveKukaTest* move_kuka = new move_kuka::MoveKukaTest(node_handle);
    move_kuka->run("lower_body_tcp2");
	delete move_kuka;

	return 0;
}
