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

#ifndef PLANNINGPARAMETERS_H_
#define PLANNINGPARAMETERS_H_

//#include <ros/ros.h>
#include <itomp_ca_planner/util/singleton.h>

namespace itomp_ca_planner
{

class PlanningParameters: public Singleton<PlanningParameters>
{
public:
	PlanningParameters();
	virtual ~PlanningParameters();

	void initFromNodeHandle();
	int getUpdateIndex() const;

	void setTrajectoryDuration(double trajectory_duration);
	double getTrajectoryDuration() const;
	double getTrajectoryDiscretization() const;
	double getPlanningTimeLimit() const;
	void setPlanningTimeLimit(double planning_time_limit);
	int getMaxIterations() const;
	int getMaxIterationsAfterCollisionFree() const;
	double getSmoothnessCostWeight() const;
	double getObstacleCostWeight() const;
	double getStateValidityCostWeight() const;
	double getEndeffectorVelocityCostWeight() const;
	double getTorqueCostWeight() const;
	double getContactInvariantCostWeight() const;
	double getPhysicsViolationCostWeight() const;
	double getGoalPoseCostWeight() const;
	double getCoMCostWeight() const;
	double getFTRCostWeight() const;
	double getCartesianTrajectoryCostWeight() const;
	double getSingularityCostWeight() const;
    double getPointCloudCostWeight() const;

	bool getAnimatePath() const;
	double getSmoothnessCostVelocity() const;
	double getSmoothnessCostAcceleration() const;
	double getSmoothnessCostJerk() const;
	std::vector<double> getSmoothnessCosts() const;
	double getRidgeFactor() const;
	bool getAnimateEndeffector() const;
	const std::multimap<std::string, std::string>& getAnimateEndeffectorSegment() const;
	int getNumTrajectories() const;
	int getNumTrials() const;
	int getNumRollouts() const;
	int getNumReusedRollouts() const;
	double getNoiseStddev() const;
	double getNoiseDecay() const;
	bool getUseCumulativeCosts() const;
	bool getUseSmoothNoises() const;
	int getNumContacts() const;
	const std::vector<double>& getContactVariableInitialValues() const;
	const std::vector<double>& getContactVariableGoalValues() const;
	bool getPrintPlanningInfo() const
	{
		return print_planning_info_;
	}

	double getPhaseDuration() const
	{
		return phase_duration_;
	}
	double getFrictionCoefficient() const
	{
		return friction_coefficient_;
	}
	std::string getLowerBodyRoot() const
	{
		return lower_body_root_;
	}
	double getTemporaryVariable(int i) const
	{
		return temporary_variables_[i];
	}
	double getPlanningStepSize() const
	{
		return planning_step_size_;
	}

	int getNumTimeSteps() const
	{
        return (int) (trajectory_duration_ / trajectory_discretization_ + 1e-7) - 1;
	}

	const std::map<std::string, double>& getJointVelocityLimits() const;

	std::string getEnvironmentModel() const;
	const std::vector<double>& getEnvironmentModelPosition() const;
	double getEnvironmentModelScale() const;
	bool getHasRoot6d() const;

        bool getUsePrecomputation() const;
	int getPrecomputationInitMilestones();
	int getPrecomputationAddMilestones();
	int getPrecomputationGrowMilestones();
	int getPrecomputationExpandMilestones();
	int getPrecomputationNn();
        double getPrecomputationMaxValidSegmentDist() const;
	bool getDrawPrecomputation();

    int getInputSequence() const;
    bool getExactCollisionDetection() const;

private:
	int updateIndex;
	double trajectory_duration_;
	double trajectory_discretization_;
	double planning_time_limit_;
	int max_iterations_;
	int max_iterations_after_collision_free_;
	double smoothness_cost_weight_;
	double obstacle_cost_weight_;
	double state_validity_cost_weight_;
	double torque_cost_weight_;
	double endeffector_velocity_cost_weight_;
	double contact_invariant_cost_weight_;
	double physics_violation_cost_weight_;
	double goal_pose_cost_weight_;
	double com_cost_weight_;
	double ftr_cost_weight_;
	double cartesian_trajectory_cost_weight_;
	double singularity_cost_weight_;
    double point_cloud_cost_weight_;
	bool animate_path_;
	double smoothness_cost_velocity_;
	double smoothness_cost_acceleration_;
	double smoothness_cost_jerk_;
	double ridge_factor_;
	bool animate_endeffector_;
	std::multimap<std::string, std::string> animate_endeffector_segment_;
	int num_trajectories_;
	double planning_step_size_;
	int num_time_steps_;

	std::map<std::string, double> joint_velocity_limits_;

	double phase_duration_;
	double friction_coefficient_;
	std::string lower_body_root_;

	bool print_planning_info_;

	int num_contacts_;
	std::vector<double> contact_variable_initial_values_;
	std::vector<double> contact_variable_goal_values_;

	int num_trials_;

	int num_rollouts_;
	int num_reused_rollouts_;
	double noise_stddev_;
	double noise_decay_;
	bool use_cumulative_costs_;
	bool use_smooth_noises_;

	std::vector<double> temporary_variables_;

	std::string environment_model_;
	std::vector<double> environment_model_position_;
	double environment_model_scale_;

	bool has_root_6d_;

        bool use_precomputation_;
	int precomputation_init_milestones_;
	int precomputation_add_milestones_;
	int precomputation_grow_milestones_;
	int precomputation_expand_milestones_;
	int precomputation_nn_;
	double precomputation_max_valid_segment_dist_;
	bool draw_precomputation_;

    int input_sequence_;
    bool exact_collision_detection_;

	friend class Singleton<PlanningParameters> ;
};

/////////////////////// inline functions follow ////////////////////////
inline int PlanningParameters::getUpdateIndex() const
{
	return updateIndex;
}

inline void PlanningParameters::setTrajectoryDuration(
		double trajectory_duration)
{
	trajectory_duration_ = trajectory_duration;
}

inline double PlanningParameters::getTrajectoryDuration() const
{
	return trajectory_duration_;
}

inline double PlanningParameters::getTrajectoryDiscretization() const
{
	return trajectory_discretization_;
}

inline double PlanningParameters::getPlanningTimeLimit() const
{
	return planning_time_limit_;
}

inline void PlanningParameters::setPlanningTimeLimit(double planning_time_limit)
{
	planning_time_limit_ = planning_time_limit;
}

inline int PlanningParameters::getMaxIterations() const
{
	return max_iterations_;
}

inline int PlanningParameters::getMaxIterationsAfterCollisionFree() const
{
	return max_iterations_after_collision_free_;
}

inline double PlanningParameters::getSmoothnessCostWeight() const
{
	return smoothness_cost_weight_;
}

inline double PlanningParameters::getObstacleCostWeight() const
{
	return obstacle_cost_weight_;
}

inline double PlanningParameters::getStateValidityCostWeight() const
{
	return state_validity_cost_weight_;
}

inline double PlanningParameters::getEndeffectorVelocityCostWeight() const
{
	return endeffector_velocity_cost_weight_;
}

inline double PlanningParameters::getTorqueCostWeight() const
{
	return torque_cost_weight_;
}

inline double PlanningParameters::getContactInvariantCostWeight() const
{
	return contact_invariant_cost_weight_;
}

inline double PlanningParameters::getPhysicsViolationCostWeight() const
{
	return physics_violation_cost_weight_;
}

inline double PlanningParameters::getGoalPoseCostWeight() const
{
	return goal_pose_cost_weight_;
}

inline double PlanningParameters::getCartesianTrajectoryCostWeight() const
{
	return cartesian_trajectory_cost_weight_;
}

inline double PlanningParameters::getSingularityCostWeight() const
{
	return singularity_cost_weight_;
}

inline double PlanningParameters::getPointCloudCostWeight() const
{
    return point_cloud_cost_weight_;
}

inline bool PlanningParameters::getAnimatePath() const
{
	return animate_path_;
}

inline double PlanningParameters::getSmoothnessCostVelocity() const
{
	return smoothness_cost_velocity_;
}

inline double PlanningParameters::getSmoothnessCostAcceleration() const
{
	return smoothness_cost_acceleration_;
}

inline double PlanningParameters::getSmoothnessCostJerk() const
{
	return smoothness_cost_jerk_;
}

inline double PlanningParameters::getRidgeFactor() const
{
	return ridge_factor_;
}

inline bool PlanningParameters::getAnimateEndeffector() const
{
	return animate_endeffector_;
}

inline const std::multimap<std::string, std::string>& PlanningParameters::getAnimateEndeffectorSegment() const
{
	return animate_endeffector_segment_;
}

inline std::vector<double> PlanningParameters::getSmoothnessCosts() const
{
	std::vector<double> ret(3);
	ret[0] = smoothness_cost_velocity_;
	ret[1] = smoothness_cost_acceleration_;
	ret[2] = smoothness_cost_jerk_;
	return ret;
}

inline double PlanningParameters::getCoMCostWeight() const
{
	return com_cost_weight_;
}

inline double PlanningParameters::getFTRCostWeight() const
{
	return ftr_cost_weight_;
}

inline int PlanningParameters::getNumTrajectories() const
{
	return num_trajectories_;
}

inline const std::map<std::string, double>& PlanningParameters::getJointVelocityLimits() const
{
	return joint_velocity_limits_;
}

inline int PlanningParameters::getNumTrials() const
{
	return num_trials_;
}

inline int PlanningParameters::getNumContacts() const
{
	return num_contacts_;
}

inline const std::vector<double>& PlanningParameters::getContactVariableInitialValues() const
{
	return contact_variable_initial_values_;
}

inline const std::vector<double>& PlanningParameters::getContactVariableGoalValues() const
{
	return contact_variable_goal_values_;
}

inline int PlanningParameters::getNumRollouts() const
{
	return num_rollouts_;
}
inline int PlanningParameters::getNumReusedRollouts() const
{
	return num_reused_rollouts_;
}
inline double PlanningParameters::getNoiseStddev() const
{
	return noise_stddev_;
}
inline double PlanningParameters::getNoiseDecay() const
{
	return noise_decay_;
}
inline bool PlanningParameters::getUseCumulativeCosts() const
{
	return use_cumulative_costs_;
}
inline bool PlanningParameters::getUseSmoothNoises() const
{
	return use_smooth_noises_;
}

inline std::string PlanningParameters::getEnvironmentModel() const
{
	return environment_model_;
}

inline const std::vector<double>& PlanningParameters::getEnvironmentModelPosition() const
{
	return environment_model_position_;
}

inline double PlanningParameters::getEnvironmentModelScale() const
{
	return environment_model_scale_;
}

inline bool PlanningParameters::getHasRoot6d() const
{
	return has_root_6d_;
}

inline bool PlanningParameters::getUsePrecomputation() const
{
    return use_precomputation_;
}
inline int PlanningParameters::getPrecomputationInitMilestones()
{
	return precomputation_init_milestones_;
}
inline int PlanningParameters::getPrecomputationAddMilestones()
{
	return precomputation_add_milestones_;
}
inline int PlanningParameters::getPrecomputationGrowMilestones()
{
	return precomputation_grow_milestones_;
}
inline int PlanningParameters::getPrecomputationExpandMilestones()
{
	return precomputation_expand_milestones_;
}
inline int PlanningParameters::getPrecomputationNn()
{
	return precomputation_nn_;
}
inline double PlanningParameters::getPrecomputationMaxValidSegmentDist() const
{
	return precomputation_max_valid_segment_dist_;
}
inline bool PlanningParameters::getDrawPrecomputation()
{
	return draw_precomputation_;
}

inline int PlanningParameters::getInputSequence() const
{
    return input_sequence_;
}

inline bool PlanningParameters::getExactCollisionDetection() const
{
    return exact_collision_detection_;
}

}
#endif /* PLANNINGPARAMETERS_H_ */
