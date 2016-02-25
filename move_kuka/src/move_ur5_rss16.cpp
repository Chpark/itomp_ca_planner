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
#include <move_kuka/move_ur5_rss16.h>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <sched.h>
#include <limits>
#include <resource_retriever/retriever.h>
#include <pcpred/prediction/kinect_predictor.h>
#include <pcpred/visualization/marker_array_visualizer.h>

using namespace std;

const int M = 8;
static int PLANNER_INDEX = -1;




void readStartGoalStates(const char* filename, std::vector<robot_state::RobotState>& robot_states)
{
    FILE* fp = fopen(filename, "r");

    for (int i=0; i<robot_states.size(); i++)
    {
        for (int j=0; j<robot_states[i].getVariableCount(); j++)
        {
            double v;
            fscanf(fp, "%lf", &v);
            robot_states[i].setVariablePosition(j, v);
        }
        robot_states[i].update();
    }

    fclose(fp);
}

void writeStartGoalStates(const char* filename, const std::vector<robot_state::RobotState>& robot_states)
{
    FILE* fp = fopen(filename, "w");

    for (int i=0; i<robot_states.size(); i++)
    {
        for (int j=0; j<robot_states[i].getVariableCount(); j++)
            fprintf(fp, "%lf ", robot_states[i].getVariablePosition(j));
        fprintf(fp, "\n");
    }

    fclose(fp);
}



namespace move_ur5
{

MoveUR5::MoveUR5(const ros::NodeHandle& node_handle) :
    node_handle_(node_handle), mat_task_frames_0_(10, std::vector<Eigen::Affine3d>(65)), mat_task_frames_78_(10, std::vector<Eigen::Affine3d>(65))
{

}

MoveUR5::~MoveUR5()
{
}

void MoveUR5::run(const std::string& group_name)
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

    //loadStaticScene();

    // planner initialization
    group_name_ = group_name;

    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
    std::string planner_plugin_name;
    if (!node_handle_.getParam("planning_plugin", planner_plugin_name))
        ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
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

    //drawFrames();

    ros::WallDuration sleep_time(0.01);
    sleep_time.sleep();

    ///////////////////////////////////////////////////////

    double last_trajectory_start_time = 0.0;
    double last_trajectory_duration = 0.0;

    //while (true)
    moveit_msgs::DisplayTrajectory display_trajectory;
    for (int i = 0; i < 10; ++i)
    {

        std::vector<Eigen::Affine3d> end_effector_poses;
        std::vector<robot_state::RobotState> robot_states;
        if (initTask(end_effector_poses, robot_states) == false)
            continue;

        for (int index = 0; index < end_effector_poses.size() - 1; ++index)
        {
            moveit_msgs::MotionPlanResponse response;
            planning_interface::MotionPlanRequest req;
            planning_interface::MotionPlanResponse res;

            // Set start / goal states
            initStartGoalStates(req, end_effector_poses, robot_states, index);

            double ct_cost_weight = 0.0;
            node_handle_.getParam("/itomp_planner/cartesian_trajectory_cost_weight", ct_cost_weight);
            if (index % 2 == 0)
                node_handle_.setParam("/itomp_planner/cartesian_trajectory_cost_weight", 0);

            // trajectory optimization using ITOMP
            bool use_itomp = (planner_plugin_name.find("Itomp") != string::npos);
            plan(req, res, use_itomp);
            res.getMessage(response);

            node_handle_.setParam("/itomp_planner/cartesian_trajectory_cost_weight", ct_cost_weight);

            last_goal_state_.reset(new robot_state::RobotState(res.trajectory_->getLastWayPoint()));

            // display trajectories
            if (index == 0)
                display_trajectory.trajectory_start = response.trajectory_start;
            display_trajectory.trajectory.push_back(response.trajectory);

            display_publisher_.publish(display_trajectory);

            double duration;
            //node_handle_.getParam("/itomp_planner/trajectory_duration", duration);
            //node_handle_.setParam("/itomp_planner/trajectory_duration", duration - 1.0);

            break;
        }

        double current_time = ros::Time::now().toSec();
        double time_to_sleep = last_trajectory_duration - (current_time - last_trajectory_start_time);
        if (time_to_sleep > 0.0)
        {
            ros::WallDuration sleep_t(time_to_sleep);
            sleep_t.sleep();
        }

        last_trajectory_start_time = ros::Time::now().toSec();
        last_trajectory_duration = (end_effector_poses.size() - 1) * 2.0;

        break;
    }

    drawResults(display_trajectory);
    animateResults(display_trajectory);

    // clean up
    itomp_planner_instance_.reset();
    planning_scene_.reset();
    robot_model_.reset();

    sleep_time.sleep();
    ROS_INFO("Done");
}

bool MoveUR5::initTask(std::vector<Eigen::Affine3d>& end_effector_poses, std::vector<robot_state::RobotState>& robot_states)
{
    const std::string end_effector_name = "power_grasp_link";

    end_effector_poses.resize(0);

    robot_state::RobotState& start_state = planning_scene_->getCurrentStateNonConst();
    if (last_goal_state_)
    {
        start_state = *last_goal_state_;
    }
    const Eigen::Affine3d& start_frame = start_state.getGlobalLinkTransform(end_effector_name);


    {
        Eigen::Affine3d target_frame;
        Eigen::Quaterniond orientation(0.500, -0.500, 0.500, 0.500);
        Eigen::Vector3d translation(-0.35,0.15,0.25);
        target_frame.linear() = orientation.matrix();
        target_frame.translation() = translation;
        end_effector_poses.push_back(target_frame);
    }
    {
        Eigen::Affine3d target_frame;
        Eigen::Quaterniond orientation(0.692, 0.687, 0.145, -0.170);
        Eigen::Vector3d translation(0.606, 0.6, 0.798);
        target_frame.linear() = orientation.matrix();
        target_frame.translation() = translation;
        end_effector_poses.push_back(target_frame);
    }



    const bool read_from_file = true;
    const char filename[] = "910.txt";

    robot_states.resize(end_effector_poses.size(), start_state);

    if (read_from_file)
    {
        readStartGoalStates(filename, robot_states);
    }

    else
    {
        for (int i = 0; i < robot_states.size(); ++i)
        {
            robot_states[i].update();
            if (computeIKState(robot_states[i], end_effector_poses[i]) == false)
            {
                return false;
            }
        }

        writeStartGoalStates("tmpu.txt", robot_states);
    }
    return true;
}

void MoveUR5::initStartGoalStates(planning_interface::MotionPlanRequest& req, const std::vector<Eigen::Affine3d>& end_effector_poses,
                                       std::vector<robot_state::RobotState>& robot_states, int index)
{
    const std::string end_effector_name = "power_grasp_link";

    drawPath(index, end_effector_poses[index].translation(), end_effector_poses[index + 1].translation());
    ros::WallDuration sleep_t(0.001);
    sleep_t.sleep();

    robot_state::RobotState& start_state = last_goal_state_ ? *last_goal_state_ : robot_states[index];
    robot_state::RobotState& goal_state = robot_states[index + 1];
    renderStartGoalStates(start_state, goal_state);

    // set start state
    // Copy from start_state to req.start_state
    unsigned int num_joints = start_state.getVariableCount();
    req.start_state.joint_state.name = start_state.getVariableNames();
    req.start_state.joint_state.position.resize(num_joints);
    req.start_state.joint_state.velocity.resize(num_joints);
    req.start_state.joint_state.effort.resize(num_joints);
    memcpy(&req.start_state.joint_state.position[0], start_state.getVariablePositions(), sizeof(double) * num_joints);
    if (start_state.hasVelocities())
        memcpy(&req.start_state.joint_state.velocity[0], start_state.getVariableVelocities(), sizeof(double) * num_joints);
    else
        memset(&req.start_state.joint_state.velocity[0], 0, sizeof(double) * num_joints);
    if (start_state.hasAccelerations())
        memcpy(&req.start_state.joint_state.effort[0], start_state.getVariableAccelerations(), sizeof(double) * num_joints);
    else
        memset(&req.start_state.joint_state.effort[0], 0, sizeof(double) * num_joints);

    // set goal state
    req.goal_constraints.clear();

    const Eigen::Affine3d& transform = end_effector_poses[index + 1];
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
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);
    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(end_effector_name, goal_pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);

    std::stringstream ss;
    ss << "Start state : ";
    ss.precision(std::numeric_limits<double>::digits10);
    for (int i = 0; i < start_state.getVariableCount(); ++i)
        ss << std::fixed << start_state.getVariablePositions()[i] << " ";
    ROS_INFO(ss.str().c_str());
}

bool MoveUR5::isStateSingular(robot_state::RobotState& state)
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

void MoveUR5::plan(planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res, bool use_itomp)
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

    req.planner_id = "ITOMP_replanning";

    planning_interface::PlanningContextPtr context =
        itomp_planner_instance_->getPlanningContext(planning_scene_, req, res.error_code_);

    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        return;
    }
}

void MoveUR5::loadStaticScene()
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

        Eigen::Affine3d& mat = mat_shelf_frame_;
        Eigen::Quaterniond q(mat.linear());
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        pose.position.x = mat.translation().x() * 0.001;
        pose.position.y = mat.translation().y() * 0.001;
        pose.position.z = mat.translation().z() * 0.001;

        shapes::Mesh* shape = shapes::createMeshFromResource(environment_file, Eigen::Vector3d(scale, scale, scale));
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(shape, mesh_msg);
        shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

        collision_object.meshes.push_back(mesh);
        collision_object.mesh_poses.push_back(pose);

        // shelf mesh
        shape = shapes::createMeshFromResource("package://move_kuka/env/SME_Aufnahmetisch_shelf_s.dae", Eigen::Vector3d(scale, scale, scale));
        shapes::constructMsgFromShape(shape, mesh_msg);
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
        const double distLeft2Middle = 11 * 30; // mm
        const double distMiddle2Right = 10 * 30; // mm
        pose.position.y += (distMiddle2Right) * 0.001;
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

void MoveUR5::renderStartGoalStates(robot_state::RobotState& start_state, robot_state::RobotState& goal_state)
{
    // display start / goal states
    int num_variables = start_state.getVariableNames().size();
    static ros::Publisher start_state_display_publisher = node_handle_.advertise<moveit_msgs::DisplayRobotState>("/move_itomp/display_start_state", 1, true);
    moveit_msgs::DisplayRobotState disp_start_state;
    disp_start_state.state.joint_state.header.frame_id = robot_model_->getModelFrame();
    disp_start_state.state.joint_state.name = start_state.getVariableNames();
    disp_start_state.state.joint_state.position.resize(num_variables);
    memcpy(&disp_start_state.state.joint_state.position[0],
           start_state.getVariablePositions(), sizeof(double) * num_variables);
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

    static ros::Publisher goal_state_display_publisher = node_handle_.advertise<moveit_msgs::DisplayRobotState>("/move_itomp/display_goal_state", 1, true);
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

bool MoveUR5::isStateCollide(const robot_state::RobotState& state)
{
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.verbose = false;
    collision_request.contacts = false;

    planning_scene_->checkCollisionUnpadded(collision_request, collision_result, state);

    return collision_result.collision;
}

bool MoveUR5::computeIKState(robot_state::RobotState& ik_state, const Eigen::Affine3d& end_effector_state, bool rand)
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
        found_ik = ik_state.setFromIK(joint_model_group, end_effector_state, 10, 0.1, moveit::core::GroupStateValidityCallbackFn(), options);
        ik_state.update();

        found_ik &= !isStateCollide(ik_state);

        if (found_ik && isStateSingular(ik_state))
            found_ik = false;

        if (found_ik)
            break;

        ++i;

        double dist = log(-3 + 0.001 * i) / log(10);

        ik_state.setToRandomPositionsNearBy(joint_model_group, org_start, dist);

        break;
    }

    if (found_ik)
    {
        //ROS_INFO("IK solution found after %d trials", i + 1);
    }
    else
    {
        ROS_INFO("Could not find IK solution");
    }
    return found_ik;
}

void MoveUR5::drawEndeffectorPosition(int id, const Eigen::Vector3d& position)
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

void MoveUR5::drawPath(int id, const Eigen::Vector3d& from, const Eigen::Vector3d& to)
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
    point.x = from(0);
    point.y = from(1);
    point.z = from(2);
    msg.points.push_back(point);
    point.x = to(0);
    point.y = to(1);
    point.z = to(2);
    msg.points.push_back(point);

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(msg);
    vis_marker_array_publisher_.publish(ma);
}

void MoveUR5::drawFrames()
{
    const double scale = 0.002;

    int id = 0;

    visualization_msgs::Marker::_color_type RED, GREEN, BLUE;
    RED.a = 1.0;
    RED.r = 1.0;
    RED.g = 0.0;
    RED.b = 0.0;
    GREEN.a = 1.0;
    GREEN.r = 0.5;
    GREEN.g = 1.0;
    GREEN.b = 0.0;
    BLUE.a = 1.0;
    BLUE.r = 0.0;
    BLUE.g = 0.0;
    BLUE.b = 1.0;

    visualization_msgs::MarkerArray ma;

    visualization_msgs::Marker msg;
    msg.header.frame_id = robot_model_->getModelFrame();
    msg.header.stamp = ros::Time::now();
    msg.type = visualization_msgs::Marker::LINE_LIST;
    msg.action = visualization_msgs::Marker::ADD;

    msg.scale.x = scale;
    msg.scale.y = scale;
    msg.scale.z = scale;

    msg.ns = "shelf";

    geometry_msgs::Point point_origin;
    point_origin.x = mat_shelf_frame_(0, 3) * 0.001;
    point_origin.y = mat_shelf_frame_(1, 3) * 0.001;
    point_origin.z = mat_shelf_frame_(2, 3) * 0.001;

    geometry_msgs::Point point_dir;
    const double scale_dir = 0.05;

    msg.id = ++id;
    msg.color = RED;
    msg.points.resize(0);
    point_dir.x = point_origin.x + mat_shelf_frame_(0, 0) * scale_dir;
    point_dir.y = point_origin.y + mat_shelf_frame_(1, 0) * scale_dir;
    point_dir.z = point_origin.z + mat_shelf_frame_(2, 0) * scale_dir;
    msg.points.push_back(point_origin);
    msg.points.push_back(point_dir);
    ma.markers.push_back(msg);

    msg.id = ++id;
    msg.color = GREEN;
    msg.points.resize(0);
    point_dir.x = point_origin.x + mat_shelf_frame_(0, 1) * scale_dir;
    point_dir.y = point_origin.y + mat_shelf_frame_(1, 1) * scale_dir;
    point_dir.z = point_origin.z + mat_shelf_frame_(2, 1) * scale_dir;
    msg.points.push_back(point_origin);
    msg.points.push_back(point_dir);
    ma.markers.push_back(msg);

    msg.id = ++id;
    msg.color = BLUE;
    msg.points.resize(0);
    point_dir.x = point_origin.x + mat_shelf_frame_(0, 2) * scale_dir;
    point_dir.y = point_origin.y + mat_shelf_frame_(1, 2) * scale_dir;
    point_dir.z = point_origin.z + mat_shelf_frame_(2, 2) * scale_dir;
    msg.points.push_back(point_origin);
    msg.points.push_back(point_dir);
    ma.markers.push_back(msg);

    msg.ns = "task_0";
    for (int i = 0; i < mat_task_frames_0_.size(); ++i)
    {

        for (int j = 0; j < mat_task_frames_0_[i].size(); ++j)
        {
            //if (j != 0 && i != 0)
            //  continue;
            const Eigen::Affine3d& mat = mat_task_frames_0_[i][j];
            geometry_msgs::Point point_origin;
            point_origin.x = mat.translation()(0) * 0.001;
            point_origin.y = mat.translation()(1) * 0.001;
            point_origin.z = mat.translation()(2) * 0.001;

            msg.id = ++id;
            msg.color = RED;
            msg.points.resize(0);
            point_dir.x = point_origin.x + mat(0, 0) * scale_dir;
            point_dir.y = point_origin.y + mat(1, 0) * scale_dir;
            point_dir.z = point_origin.z + mat(2, 0) * scale_dir;
            msg.points.push_back(point_origin);
            msg.points.push_back(point_dir);
            ma.markers.push_back(msg);

            msg.id = ++id;
            msg.color = GREEN;
            msg.points.resize(0);
            point_dir.x = point_origin.x + mat(0, 1) * scale_dir;
            point_dir.y = point_origin.y + mat(1, 1) * scale_dir;
            point_dir.z = point_origin.z + mat(2, 1) * scale_dir;
            msg.points.push_back(point_origin);
            msg.points.push_back(point_dir);
            ma.markers.push_back(msg);

            msg.id = ++id;
            msg.color = BLUE;
            msg.points.resize(0);
            point_dir.x = point_origin.x + mat(0, 2) * scale_dir;
            point_dir.y = point_origin.y + mat(1, 2) * scale_dir;
            point_dir.z = point_origin.z + mat(2, 2) * scale_dir;
            msg.points.push_back(point_origin);
            msg.points.push_back(point_dir);
            ma.markers.push_back(msg);
        }
    }

    msg.ns = "task_78";
    for (int i = 0; i < mat_task_frames_78_.size(); ++i)
    {
        for (int j = 0; j < mat_task_frames_78_[i].size(); ++j)
        {
            const Eigen::Affine3d& mat = mat_task_frames_78_[i][j];
            geometry_msgs::Point point_origin;
            point_origin.x = mat.translation()(0) * 0.001;
            point_origin.y = mat.translation()(1) * 0.001;
            point_origin.z = mat.translation()(2) * 0.001;

            msg.id = ++id;
            msg.color = RED;
            msg.points.resize(0);
            point_dir.x = point_origin.x + mat(0, 0) * scale_dir;
            point_dir.y = point_origin.y + mat(1, 0) * scale_dir;
            point_dir.z = point_origin.z + mat(2, 0) * scale_dir;
            msg.points.push_back(point_origin);
            msg.points.push_back(point_dir);
            ma.markers.push_back(msg);

            msg.id = ++id;
            msg.color = GREEN;
            msg.points.resize(0);
            point_dir.x = point_origin.x + mat(0, 1) * scale_dir;
            point_dir.y = point_origin.y + mat(1, 1) * scale_dir;
            point_dir.z = point_origin.z + mat(2, 1) * scale_dir;
            msg.points.push_back(point_origin);
            msg.points.push_back(point_dir);
            ma.markers.push_back(msg);

            msg.id = ++id;
            msg.color = BLUE;
            msg.points.resize(0);
            point_dir.x = point_origin.x + mat(0, 2) * scale_dir;
            point_dir.y = point_origin.y + mat(1, 2) * scale_dir;
            point_dir.z = point_origin.z + mat(2, 2) * scale_dir;
            msg.points.push_back(point_origin);
            msg.points.push_back(point_dir);
            ma.markers.push_back(msg);
        }
    }

    msg.ns = "rivet_magazine";
    const Eigen::Affine3d& mat = mat_rivet_magazine_;
    point_origin.x = mat.translation()(0) * 0.001;
    point_origin.y = mat.translation()(1) * 0.001;
    point_origin.z = mat.translation()(2) * 0.001;

    msg.id = ++id;
    msg.color = RED;
    msg.points.resize(0);
    point_dir.x = point_origin.x + mat(0, 0) * scale_dir;
    point_dir.y = point_origin.y + mat(1, 0) * scale_dir;
    point_dir.z = point_origin.z + mat(2, 0) * scale_dir;
    msg.points.push_back(point_origin);
    msg.points.push_back(point_dir);
    ma.markers.push_back(msg);

    msg.id = ++id;
    msg.color = GREEN;
    msg.points.resize(0);
    point_dir.x = point_origin.x + mat(0, 1) * scale_dir;
    point_dir.y = point_origin.y + mat(1, 1) * scale_dir;
    point_dir.z = point_origin.z + mat(2, 1) * scale_dir;
    msg.points.push_back(point_origin);
    msg.points.push_back(point_dir);
    ma.markers.push_back(msg);

    msg.id = ++id;
    msg.color = BLUE;
    msg.points.resize(0);
    point_dir.x = point_origin.x + mat(0, 2) * scale_dir;
    point_dir.y = point_origin.y + mat(1, 2) * scale_dir;
    point_dir.z = point_origin.z + mat(2, 2) * scale_dir;
    msg.points.push_back(point_origin);
    msg.points.push_back(point_dir);
    ma.markers.push_back(msg);

    vis_marker_array_publisher_.publish(ma);

    ros::WallDuration sleep_time(0.01);
    sleep_time.sleep();
}

void MoveUR5::drawResults(moveit_msgs::DisplayTrajectory& display_trajectory)
{
    ros::NodeHandle node_handle;
    ros::Publisher publisher = node_handle.advertise<visualization_msgs::MarkerArray>("result", 10);

    ros::Duration(3.0).sleep();

    const int step = 10;

    std_msgs::ColorRGBA ROBOT;
    ROBOT.r = 0.5; ROBOT.g = 0.5; ROBOT.b = 0.5;
    ROBOT.a = 0.5;

    std_msgs::ColorRGBA ORANGE;
    ORANGE.r = 1.0; ORANGE.g = 0.6; ORANGE.b = 0.0;
    ORANGE.a = 1.0;

    std_msgs::ColorRGBA GRAY;
    GRAY.r = GRAY.g = GRAY.b = 0.5;
    GRAY.a = 0.25;

    std::vector<std::string> link_names = robot_model_->getJointModelGroup("ur5")->getLinkModelNames();

    const int num_points = display_trajectory.trajectory[0].joint_trajectory.points.size();

    visualization_msgs::MarkerArray ma;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.ns = "path";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.lifetime = ros::Duration();

    for (int point=0; point<num_points; point++)
    {
        geometry_msgs::Point p;
        std_msgs::ColorRGBA c;

        robot_state::RobotState robot_state(robot_model_);
        robot_state.setVariablePositions( display_trajectory.trajectory[0].joint_trajectory.points[point].positions );
        robot_state.updateLinkTransforms();

        const Eigen::Affine3d& t = robot_state.getFrameTransform("power_grasp_link");
        p.x = t.translation()(0);
        p.y = t.translation()(1);
        p.z = t.translation()(2);

        const double s = (double)point / (num_points - 1);
        c.r = (1-s) * 1.0 + s * 0.0;
        c.g = (1-s) * 0.6 + s * 0.0;
        c.b = (1-s) * 0.0 + s * 1.0;
        c.a = 1.0;

        marker.points.push_back(p);
        marker.colors.push_back(c);
    }

    ma.markers.push_back(marker);

    for (int point=0; point<num_points; point += step)
    {
        visualization_msgs::MarkerArray ma_point;

        robot_state::RobotState robot_state(robot_model_);
        robot_state.setVariablePositions( display_trajectory.trajectory[0].joint_trajectory.points[point].positions );
        robot_state.updateLinkTransforms();
        std::string ns = "path_" + boost::lexical_cast<std::string>(point);
        robot_state.getRobotMarkers(ma_point, link_names, ROBOT, ns, ros::Duration());

        for (int i=0; i<ma_point.markers.size(); i++)
        {
            ma_point.markers[i].mesh_use_embedded_materials = true;
        }

        ma.markers.insert(ma.markers.end(), ma_point.markers.begin(), ma_point.markers.end());
    }

    publisher.publish(ma);
}






void MoveUR5::animateResults(moveit_msgs::DisplayTrajectory& display_trajectory)
{
    ros::NodeHandle node_handle;
    ros::Publisher publisher = node_handle.advertise<visualization_msgs::MarkerArray>("result_video", 10);

    ros::Duration(3.0).sleep();

    const int step = 10;

    std_msgs::ColorRGBA ROBOT;
    ROBOT.r = 0.5; ROBOT.g = 0.5; ROBOT.b = 0.5;
    ROBOT.a = 1.0;

    std_msgs::ColorRGBA ORANGE;
    ORANGE.r = 1.0; ORANGE.g = 0.6; ORANGE.b = 0.0;
    ORANGE.a = 1.0;

    std_msgs::ColorRGBA GRAY;
    GRAY.r = GRAY.g = GRAY.b = 0.5;
    GRAY.a = 0.25;

    std::vector<std::string> link_names = robot_model_->getJointModelGroup("ur5")->getLinkModelNames();

    const int num_points = display_trajectory.trajectory[0].joint_trajectory.points.size();












    pcpred::KinectPredictor pc_predictor_;

    const double timestep = 0.05;               // 0.05 s
    double sensor_error = 0.005;          // 1 mm
    double collision_probability = 0.95;  // 95%
    const int acceleration_inference_window_size = 5;

    node_handle.param("/itomp_planner/sensor_error", sensor_error, 0.005);
    node_handle.param("/itomp_planner/collision_probability", collision_probability, 0.95);

    /* Sequence 1: left hand, slow, forth
     * Sequence 2: right hand, slow, forth
     * Sequence 3: left hand, fast, forth
     * Sequence 4: right hand, fast, forth
     * Sequence 5: left hand, left up
     * Sequence 6: left hand, left down
     */
    const int sequence_number = 9;
    const Eigen::Vector3d pointcloud_translates[] =
    {
        Eigen::Vector3d(0.1, -0.7, -0.9),
        Eigen::Vector3d(0.18, 1, -0.9),
        Eigen::Vector3d(0, -0.5, -0.9),
        Eigen::Vector3d(-0.2, 0.7, -0.9),
        Eigen::Vector3d(0.75, -0.9, -0.9),
        Eigen::Vector3d(0.75, -0.8, -0.9),
        Eigen::Vector3d(5.0, -0.0, -0.9),
        Eigen::Vector3d(5.0, -0.0, -0.9),
        Eigen::Vector3d(0.8, 2.5, -0.5),
        Eigen::Vector3d(0.7, 1.7, -0.6),
    };
    const double z_rotations[] =
    {
        0.0,
        0.0,
        0.0,
        0.0,
        -30.0 / 180.0 * M_PI,
        -15.0 / 180.0 * M_PI,
        0.0,
        0.0,
        -90.0 / 180.0 * M_PI,
        -90.0 / 180.0 * M_PI,
    };

    // initialize predictor
    pc_predictor_.setSequence( sequence_number );
    pc_predictor_.setTimestep(timestep);
    pc_predictor_.setSensorDiagonalCovariance(sensor_error * sensor_error);   // variance is proportional to square of sensing error
    pc_predictor_.setCollisionProbability(collision_probability);
    pc_predictor_.setAccelerationInferenceWindowSize(acceleration_inference_window_size);
    pc_predictor_.setVisualizerTopic("bvh_prediction_test");

    pc_predictor_.setMaximumIterations(5);
    pc_predictor_.setGradientDescentMaximumIterations(5);
    pc_predictor_.setGradientDescentAlpha(0.005);
    pc_predictor_.setHumanShapeLengthConstraintEpsilon(0.01);

    // transform
    pc_predictor_.translate( Eigen::Vector3d(0.4, 0.0, 0.0) );
    pc_predictor_.rotate( z_rotations[sequence_number - 1], Eigen::Vector3d(0, 0, 1) );
    pc_predictor_.translate( Eigen::Vector3d(-0.4, 0.0, 0.0) );
    pc_predictor_.translate( pointcloud_translates[sequence_number - 1] );

    const double sequence_duration = 7.0;
    const int sequence_end = sequence_duration / timestep;
    const int end = num_points;

    // skip 1 sec
    /*
    for (int i=0; i<30; i++)
        pc_predictor_.getInstance()->moveToNextFrame();
        */

    std::vector<std::vector<std::vector<Eigen::Vector3d> > > mu;
    std::vector<std::vector<std::vector<Eigen::Matrix3d> > > sigma;
    std::vector<double> radius;

    mu.resize(end);
    sigma.resize(end);
    for (int i = 0; i < end; ++i)
    {
        pc_predictor_.moveToNextFrame();

        mu[i].resize(20);
        sigma[i].resize(20);

        std::vector<Eigen::Vector3d> mus;
        std::vector<Eigen::Matrix3d> sigmas;

        for (int j = 0; j < 20; ++j)
        {
            pc_predictor_.getPredictedGaussianDistribution(j * timestep, mus, sigmas, radius);
            mu[i][j] = mus;
            sigma[i][j] = sigmas;
        }
    }

    pcpred::MarkerArrayVisualizer visualizer("result2");


    ros::Rate rate(20);
    while (true)
    {
        visualization_msgs::MarkerArray ma;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base_link";
        marker.header.stamp = ros::Time::now();
        marker.id = 0;
        marker.ns = "path_ee";
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.lifetime = ros::Duration();

        for (int point=0; point<num_points; point++)
        {
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            robot_state::RobotState robot_state(robot_model_);
            robot_state.setVariablePositions( display_trajectory.trajectory[0].joint_trajectory.points[point].positions );
            robot_state.updateLinkTransforms();

            const Eigen::Affine3d& t = robot_state.getFrameTransform("power_grasp_link");
            p.x = t.translation()(0);
            p.y = t.translation()(1);
            p.z = t.translation()(2);

            const double s = (double)point / (num_points - 1);
            c.r = (1-s) * 1.0 + s * 0.0;
            c.g = (1-s) * 0.6 + s * 0.0;
            c.b = (1-s) * 0.0 + s * 1.0;
            c.a = 1.0;

            marker.points.push_back(p);
            marker.colors.push_back(c);
        }

        ma.markers.push_back(marker);
        publisher.publish(ma);



        for (int point=0; point<num_points; point ++)
        {
            visualization_msgs::MarkerArray ma_point;

            robot_state::RobotState robot_state(robot_model_);
            robot_state.setVariablePositions( display_trajectory.trajectory[0].joint_trajectory.points[point].positions );
            robot_state.updateLinkTransforms();
            std::string ns = "path";
            robot_state.getRobotMarkers(ma_point, link_names, ROBOT, ns, ros::Duration());

            for (int i=0; i<ma_point.markers.size(); i++)
            {
                ma_point.markers[i].mesh_use_embedded_materials = true;
            }

            int j=10;
            visualizer.drawGaussianDistributions("obstacles1", mu[point][j], sigma[point][j], 0.95, radius);
            j=19;
            visualizer.drawGaussianDistributions("obstacles2", mu[point][j], sigma[point][j], 0.95, radius);
            j=0;
            visualizer.drawGaussianDistributions("obstacles3", mu[point][j], sigma[point][j], 0.95, radius);

            publisher.publish(ma_point);
            rate.sleep();
        }
    }
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

    move_ur5::MoveUR5* move_ur5 = new move_ur5::MoveUR5(node_handle);
    move_ur5->run("ur5_m2");
    delete move_ur5;

    return 0;
}
