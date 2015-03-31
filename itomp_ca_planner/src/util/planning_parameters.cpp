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

#include <itomp_ca_planner/util/planning_parameters.h>
#include <ros/ros.h>

namespace itomp_ca_planner
{

PlanningParameters::PlanningParameters() :
    num_time_steps_(0), updateIndex(-1)
{
}

PlanningParameters::~PlanningParameters()
{
}

void PlanningParameters::initFromNodeHandle()
{
	++updateIndex;

	ros::NodeHandle node_handle("itomp_planner");
	node_handle.param("num_trials", num_trials_, 1);
	node_handle.param("planning_time_limit", planning_time_limit_, 1.0);
	node_handle.param("max_iterations", max_iterations_, 500);
	node_handle.param("max_iterations_after_collision_free",
                      max_iterations_after_collision_free_, 2);
	node_handle.param("num_trajectories", num_trajectories_, 1);
	node_handle.param("trajectory_duration", trajectory_duration_, 5.0);
	node_handle.param("trajectory_discretization", trajectory_discretization_,
                      0.05);

	node_handle.param("smoothness_cost_weight", smoothness_cost_weight_,
                      0.0001);
	node_handle.param("obstacle_cost_weight", obstacle_cost_weight_, 1.0);
	node_handle.param("torque_cost_weight", torque_cost_weight_, 0.0);
	node_handle.param("state_validity_cost_weight", state_validity_cost_weight_,
                      1.0);
	node_handle.param("endeffector_velocity_cost_weight",
                      endeffector_velocity_cost_weight_, 0.0);
	node_handle.param("contact_invariant_cost_weight",
                      contact_invariant_cost_weight_, 1.0);
	node_handle.param("physics_violation_cost_weight",
                      physics_violation_cost_weight_, 1.0);
	node_handle.param("goal_pose_cost_weight", goal_pose_cost_weight_, 1.0);
	node_handle.param("CoM_cost_weight", com_cost_weight_, 1.0);
	node_handle.param("FTR_cost_weight", ftr_cost_weight_, 1.0);
	node_handle.param("cartesian_trajectory_cost_weight",
                      cartesian_trajectory_cost_weight_, 1.0);
	node_handle.param("singularity_cost_weight", singularity_cost_weight_, 1.0);

	node_handle.param("smoothness_cost_velocity", smoothness_cost_velocity_,
                      0.0);
	node_handle.param("smoothness_cost_acceleration",
                      smoothness_cost_acceleration_, 1.0);
	node_handle.param("smoothness_cost_jerk", smoothness_cost_jerk_, 0.0);
	node_handle.param("ridge_factor", ridge_factor_, 0.0);

	node_handle.param("animate_path", animate_path_, false);
	node_handle.param("animate_endeffector", animate_endeffector_, true);

	node_handle.param("print_planning_info", print_planning_info_, true);

	animate_endeffector_segment_.clear();
	if (node_handle.hasParam("animate_endeffector_segment"))
	{
		XmlRpc::XmlRpcValue segment;

		node_handle.getParam("animate_endeffector_segment", segment);

		if (segment.getType() == XmlRpc::XmlRpcValue::TypeStruct)
		{
			if (segment.size() > 0)
			{
				for (XmlRpc::XmlRpcValue::iterator it = segment.begin();
						it != segment.end(); it++)
				{
					std::string component = it->first;
					if (it->second.getType() == XmlRpc::XmlRpcValue::TypeString)
					{
						std::string endeffector = it->second;
						animate_endeffector_segment_.insert(
                            std::make_pair<std::string, std::string>(
                                component, endeffector));
					}
					else if (it->second.getType()
                             == XmlRpc::XmlRpcValue::TypeArray)
					{
						int size = it->second.size();
						for (int i = 0; i < size; ++i)
						{
							std::string endeffector = it->second[i];
							animate_endeffector_segment_.insert(
                                std::make_pair<std::string, std::string>(
                                    component, endeffector));
						}
					}
				}
			}
		}
	}

	node_handle.param("phase_duration", phase_duration_, 0.25);
	node_handle.param("friction_coefficient", friction_coefficient_, 2.0);
	node_handle.param<std::string>("lower_body_root", lower_body_root_,
                                   "pelvis_link");

	temporary_variables_.clear();
	if (node_handle.hasParam("temp"))
	{
		XmlRpc::XmlRpcValue segment;

		node_handle.getParam("temp", segment);

		if (segment.getType() == XmlRpc::XmlRpcValue::TypeArray)
		{
			int size = segment.size();
			for (int i = 0; i < size; ++i)
			{
				double value = segment[i];
				temporary_variables_.push_back(value);
			}
		}
	}

	node_handle.param("planning_step_size", planning_step_size_, 5.0);

	if (node_handle.hasParam("joint_velocity_limits"))
	{
		XmlRpc::XmlRpcValue velocity_limits;

		node_handle.getParam("joint_velocity_limits", velocity_limits);

		if (velocity_limits.getType() == XmlRpc::XmlRpcValue::TypeStruct)
		{
			if (velocity_limits.size() > 0)
			{
				for (XmlRpc::XmlRpcValue::iterator it = velocity_limits.begin();
						it != velocity_limits.end(); it++)
				{
					joint_velocity_limits_[it->first] = it->second;
				}
			}
		}
	}

	node_handle.param("num_rollouts", num_rollouts_, 10);
	node_handle.param("num_reused_rollouts", num_reused_rollouts_, 5);
	node_handle.param("noise_stddev", noise_stddev_, 2.0);
	node_handle.param("noise_decay", noise_decay_, 0.999);
	node_handle.param("use_cumulative_costs", use_cumulative_costs_, true);
	node_handle.param("use_smooth_noises", use_smooth_noises_, true);

	node_handle.param("num_contacts", num_contacts_, 0);

	contact_variable_initial_values_.clear();
	if (node_handle.hasParam("contact_variable_initial_values"))
	{
		XmlRpc::XmlRpcValue segment;

		node_handle.getParam("contact_variable_initial_values", segment);

		if (segment.getType() == XmlRpc::XmlRpcValue::TypeArray)
		{
			int size = segment.size();
			for (int i = 0; i < size; ++i)
			{
				double value = segment[i];
				contact_variable_initial_values_.push_back(value);
			}
		}
	}
	contact_variable_goal_values_.clear();
	if (node_handle.hasParam("contact_variable_goal_values"))
	{
		XmlRpc::XmlRpcValue segment;

		node_handle.getParam("contact_variable_goal_values", segment);

		if (segment.getType() == XmlRpc::XmlRpcValue::TypeArray)
		{
			int size = segment.size();
			for (int i = 0; i < size; ++i)
			{
				double value = segment[i];
				contact_variable_goal_values_.push_back(value);
			}
		}
	}

	node_handle.param<std::string>("environment_model", environment_model_, "");
	environment_model_position_.clear();
	if (node_handle.hasParam("environment_model_position"))
	{
		XmlRpc::XmlRpcValue segment;

		node_handle.getParam("environment_model_position", segment);

		if (segment.getType() == XmlRpc::XmlRpcValue::TypeArray)
		{
			int size = segment.size();
			for (int i = 0; i < size; ++i)
			{
				double value = segment[i];
				environment_model_position_.push_back(value);
			}
		}
	}
	node_handle.param("environment_model_scale", environment_model_scale_, 1.0);

	node_handle.param("has_root_6d", has_root_6d_, true);

    node_handle.param("use_precomputation", use_precomputation_, true);
	node_handle.param("precomputation_init_milestones",
                      precomputation_init_milestones_, 1000);
	node_handle.param("precomputation_add_milestones",
                      precomputation_add_milestones_, 1000);
	node_handle.param("precomputation_grow_milestones",
                      precomputation_grow_milestones_, 100);
	node_handle.param("precomputation_expand_milestones",
                      precomputation_expand_milestones_, 400);
	node_handle.param("precomputation_nn", precomputation_nn_, 10);
	node_handle.param("precomputation_max_valid_segment_dist",
                      precomputation_max_valid_segment_dist_, 0.3);
	node_handle.param("draw_precomputation",
                      draw_precomputation_, true);
}

} // namespace

