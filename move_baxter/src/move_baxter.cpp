// Original code from pr2_moveit_tutorials::motion_planning_api_tutorial.cpp
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_state/conversions.h>
int main(int argc, char **argv)
{
    ros::init (argc, argv, "move_baxter");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    // Load planner plugin
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;
    if (!node_handle.getParam("planning_plugin", planner_plugin_name))
        planner_plugin_name = "itomp_ca_planner/ItompPlanner";
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    }
    catch(pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
            ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch(pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0 ; i < classes.size() ; ++i)
            ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                         << "Available plugins: " << ss.str());
    }

    ros::WallDuration sleep_time(15.0);
    sleep_time.sleep();

    // Set planning request
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    req.group_name = "right_arm";
    req.allowed_planning_time = 300.0;

    // set start configuration
    robot_state::RobotState start_state = planning_scene->getCurrentStateNonConst();
    const robot_state::JointModelGroup* joint_model_group = start_state.getJointModelGroup(req.group_name);
    robot_state::robotStateToRobotStateMsg(start_state, req.start_state);

    // set goal configuration
    robot_state::RobotState goal_state(robot_model);
    std::vector<double> joint_values(7, 0.0);
    joint_values[0] = -1.0;
    joint_values[3] = 0.2;
    joint_values[5] = -0.15;
    goal_state.setJointGroupPositions(joint_model_group, joint_values);

    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);

    // Call the planner
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    if(res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
    }

    // Visualize the trajectory
    ROS_INFO("Visualizing the trajectory");
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit_msgs::MotionPlanResponse response;

    res.getMessage(response);
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);

    // Exit
    sleep_time.sleep();
    ROS_INFO("Done");
    planner_instance.reset();
    return 0;
}

