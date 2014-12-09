itomp_ca_planner
================

ITOMP_CA_Planner is a planning library which uses a two-step planning approach (PRM-based precomputation step and trajectory optimization step).

Installation
===========
ITOMP_CA_Planner is a planning plug-in for Moveit framework and tested on ROS Hydro.
ITOMP_CA_Planner uses Flann libarary(http://www.cs.ubc.ca/research/flann/) for k-NN search and it should be installed on the system before ITOMP_CA_Planner.
With ROS, ITOMP_CA_Planner can be easily build using rosmake command.

```
itomp_ca_planner$ rosmake
```

Interfaces
===========
When the build is done, there are two shared library files are generated in ITOMP_CA_Planner/lib directory.
(libitomp_ca.so for the planning library and libitomp_ca_planner_plugin.so for Moveit framework plug-in).
ITOMP_CA_Planner can be loaded using below commands.
```
try
{
  planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
					"moveit_core", "planning_interface::PlannerManager"));
} catch (pluginlib::PluginlibException& ex)
{
  ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
}
try
{
  itomp_planner_instance_.reset(planner_plugin_loader->createUnmanagedInstance("itomp_ca_planner/ItompPlanner"));
  if (!itomp_planner_instance_->initialize(robot_model_, node_handle_.getNamespace()))
	  ROS_FATAL_STREAM("Could not initialize planner instance");
  ROS_INFO_STREAM("Using planning interface '" << itomp_planner_instance_->getDescription() << "'");
} catch (pluginlib::PluginlibException& ex)
{
}
```
A example file (move_kuka_test.cpp) shows how to use the planner.

Planning Parameters
===========
Various parameters are used for the precomputation step and the optimization step.
These values can be adjusted in config/params_XXXX.yaml file.
