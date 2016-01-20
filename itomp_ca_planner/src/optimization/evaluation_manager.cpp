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
#include <ros/ros.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/PlanningScene.h>
#include <itomp_ca_planner/optimization/evaluation_manager.h>
#include <itomp_ca_planner/model/itomp_planning_group.h>
#include <itomp_ca_planner/contact/ground_manager.h>
#include <itomp_ca_planner/visualization/visualization_manager.h>
#include <itomp_ca_planner/contact/contact_force_solver.h>
#include <itomp_ca_planner/util/min_jerk_trajectory.h>
#include <itomp_ca_planner/util/planning_parameters.h>
#include <itomp_ca_planner/util/vector_util.h>
#include <itomp_ca_planner/util/multivariate_gaussian.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <iostream>

using namespace std;
using namespace Eigen;

namespace itomp_ca_planner
{
static int LeftLegStart = 0;



static bool STABILITY_COST_VERBOSE = false;

EvaluationManager::EvaluationManager(int* iteration) :
    iteration_(iteration), data_(&default_data_), count_(0)
{
	print_debug_texts_ = false;
}

EvaluationManager::~EvaluationManager()
{

}

void EvaluationManager::initialize(ItompCIOTrajectory *full_trajectory,
                                   ItompCIOTrajectory *group_trajectory, ItompRobotModel *robot_model,
                                   const ItompPlanningGroup *planning_group, double planning_start_time,
                                   double trajectory_start_time,
                                   const moveit_msgs::Constraints& path_constraints,
                                   const planning_scene::PlanningSceneConstPtr& planning_scene)
{
    //omp_set_num_threads(getNumParallelThreads());

	planning_start_time_ = planning_start_time;
	trajectory_start_time_ = trajectory_start_time;

	robot_model_ = robot_model;
	planning_group_ = planning_group;
	robot_name_ = robot_model_->getRobotName();

	// init some variables:
	num_joints_ = group_trajectory->getNumJoints();
	num_contacts_ = group_trajectory->getNumContacts();
	num_points_ = group_trajectory->getNumPoints();
	num_contact_points_ = group_trajectory->getNumContactPhases() + 1;

	num_vars_full_ = num_points_ - 2 * (DIFF_RULE_LENGTH - 2); // 111 - 2(7 - 2) = 101
	full_vars_start_ = (DIFF_RULE_LENGTH - 2); // 7 - 2 = 5
	full_vars_end_ = num_points_ - (DIFF_RULE_LENGTH - 2); // 111 - (7 - 2) = 106(next var of the last full traj var)

	// set up joint index:
	group_joint_to_kdl_joint_index_.resize(num_joints_);
	for (int i = 0; i < num_joints_; ++i)
		group_joint_to_kdl_joint_index_[i] =
            planning_group_->group_joints_[i].kdl_joint_index_;

	is_collision_free_ = false;
	last_trajectory_collision_free_ = false;

    vis_marker_pub_ = VisualizationManager::getInstance()->getVisualizationMarkerPublisher();
    vis_marker_array_pub_ = VisualizationManager::getInstance()->getVisualizationMarkerArrayPublisher();

	computeMassAndGravityForce();

	GroundManager::getInstance().init();

	default_data_.initialize(full_trajectory, group_trajectory, robot_model,
                             planning_group, this, num_mass_segments_, path_constraints,
                             planning_scene);

	timings_.resize(100, 0);
	for (int i = 0; i < 100; ++i)
		timings_[i] = 0;

	for (int i = 0; i < 3; ++i)
	{
		phaseJointArray_[i].resize(robot_model_->getKDLTree()->getNrOfJoints());
	}

    min_jerk_curve_.resize(num_vars_full_);
    for (int i = 0; i < num_vars_full_; ++i)
    {
        double t = (double)i / (num_vars_full_ - 1);
        double t3 = t * t * t;
        double t4 = t3 * t;
        double t5 = t4 * t;
        min_jerk_curve_[i] = 6.0 * t5 - 15.0 * t4 + 10.0 * t3;
    }

    const double timestep = 0.05;               // 0.05 s
    const double sensor_error = 0.001;          // 1 mm
    const double collision_probability = 0.95;  // 95%
    const int acceleration_inference_window_size = 5;
    const int prediction_frames = 10;
    ros::Rate rate(1. / timestep);
    // initialize predictor
    bvh_predictor_.reset(new pcpred::BvhPredictor("../data/bvh/waving.bvh"));  // load bvh file
    bvh_predictor_->setTimestep(timestep);
    bvh_predictor_->setSensorDiagonalCovariance(sensor_error * sensor_error);   // variance is proportional to square of sensing error
    bvh_predictor_->setCollisionProbability(collision_probability);
    bvh_predictor_->setAccelerationInferenceWindowSize(acceleration_inference_window_size);
    bvh_predictor_->setVisualizerTopic("bvh_prediction_test");

    tf::TransformListener tf_listener;
    tf::StampedTransform tf_transform;
    while (true)
    {
        try
        {
            tf_listener.lookupTransform("/world", "/odom_combined", ros::Time(0), tf_transform);
            tf::transformTFToEigen(tf_transform, point_cloud_transform_);
            break;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(0.1).sleep();
        }
    }

}

double EvaluationManager::evaluate()
{
    INIT_TIME_MEASUREMENT(10)

    ADD_TIMER_POINT

	// do forward kinematics:
	last_trajectory_collision_free_ = performForwardKinematics();

    ADD_TIMER_POINT

    //handleTrajectoryConstraint();

	computeTrajectoryValidity();
	last_trajectory_collision_free_ &= trajectory_validity_;

    ADD_TIMER_POINT

	// TODO: reuse
	computeCollisionCosts();

    ADD_TIMER_POINT

	//computeFTRs();
	computeSingularityCosts();

    ADD_TIMER_POINT

	computeCartesianTrajectoryCosts();

    ADD_TIMER_POINT

    computePointCloudCosts();

    ADD_TIMER_POINT

	data_->costAccumulator_.compute(data_);

    ADD_TIMER_POINT

	last_trajectory_collision_free_ &= data_->costAccumulator_.isFeasible();

	// TODO: if trajectory is changed in handle joint limits,
	// update parameters

    UPDATE_TIME
    PRINT_TIME(evaluate, 10)

	return data_->costAccumulator_.getTrajectoryCost();
}

double EvaluationManager::evaluate(Eigen::VectorXd& costs)
{
	double ret = evaluate();

	//int num_vars_free = getGroupTrajectory()->getFreePoints().rows();

	// TODO
	int num_vars_free = num_points_ - 10 - 2;
	int start = 6;
	for (int i = 0; i < num_vars_free; i++)
	{
		costs(i) = data_->costAccumulator_.getWaypointCost(start + i);
	}

	return ret;
}

double EvaluationManager::evaluate(DERIVATIVE_VARIABLE_TYPE variable_type,
                                   int free_point_index, int joint_index)
{
	int stride = getGroupTrajectory()->getContactPhaseStride();
	int begin = (free_point_index - 1) * stride;
	int end = (free_point_index + 1) * stride;

	INIT_TIME_MEASUREMENT(10)

	// do forward kinematics:
	if (variable_type != DERIVATIVE_CONTACT_VARIABLE)
		performForwardKinematics(begin, end);

	ADD_TIMER_POINT

	computeTrajectoryValidity();

	ADD_TIMER_POINT

	ADD_TIMER_POINT

    if (variable_type != DERIVATIVE_CONTACT_VARIABLE)
		computeCollisionCosts(begin, end);

	ADD_TIMER_POINT

	computeFTRs(begin, end);

	data_->costAccumulator_.compute(data_);

	UPDATE_TIME
	PRINT_TIME(evaluate, 10000)

	return data_->costAccumulator_.getTrajectoryCost();
}

void EvaluationManager::setTrajectory(const Eigen::MatrixXd& parameters,
                                      const Eigen::MatrixXd& vel_parameters,
                                      const Eigen::MatrixXd& contact_parameters)
{
	// copy the parameters into group_trajectory:
	int num_free_points = parameters.rows();
	ROS_ASSERT(getGroupTrajectory()->getFreePoints().rows() == num_free_points);

	getGroupTrajectory()->getFreePoints() = parameters;
	getGroupTrajectory()->getFreeVelPoints() = vel_parameters;
	getGroupTrajectory()->getContactTrajectory() = contact_parameters;

	getGroupTrajectory()->updateTrajectoryFromFreePoints();

	// respect joint limits:
	handleJointLimits();

	// copy to full traj:
	updateFullTrajectory();
}

void EvaluationManager::setTrajectory(
    const std::vector<Eigen::VectorXd>& parameters,
    const std::vector<Eigen::VectorXd>& contact_parameters)
{
	// copy the parameters into group_trajectory:
	int cols = getGroupTrajectory()->getTrajectory().cols();
	for (int d = 0; d < num_joints_; ++d)
	{
		getGroupTrajectory()->getFreeJointTrajectoryBlock(d) = parameters[d];
	}

	cols = num_contact_points_;
	for (int d = 0; d < num_contacts_; ++d)
	{
		//getGroupTrajectory()->getContactTrajectory().block(i, 0, 1, cols) = contact_parameters[i];
		getGroupTrajectory()->getFreeContactTrajectoryBlock(d) =
            contact_parameters[d];
	}

	//getGroupTrajectory()->updateTrajectoryFromFreePoints();

	// respect joint limits:
	handleJointLimits();

	// copy to full traj:
	updateFullTrajectory();
}

void EvaluationManager::backupAndSetVariables(double new_value,
		DERIVATIVE_VARIABLE_TYPE variable_type, int free_point_index,
		int joint_index)
{
	// backup trajectory value
	Eigen::MatrixXd* target_trajectory;
	switch (variable_type)
	{
	case DERIVATIVE_POSITION_VARIABLE:
		target_trajectory = &getGroupTrajectory()->getFreePoints();
		break;

	case DERIVATIVE_VELOCITY_VARIABLE:
		target_trajectory = &getGroupTrajectory()->getFreeVelPoints();
		break;

	case DERIVATIVE_CONTACT_VARIABLE:
		target_trajectory = &getGroupTrajectory()->getContactTrajectory();
		break;
	}
	backup_data_.trajectory_value_ = (*target_trajectory)(free_point_index,
                                     joint_index);

	ROS_ASSERT(
        variable_type != DERIVATIVE_CONTACT_VARIABLE || new_value >= 0.0);

	// change trajectory variable
	(*target_trajectory)(free_point_index, joint_index) = new_value;

	// update trajectory
	if (variable_type != DERIVATIVE_CONTACT_VARIABLE)
	{
		getGroupTrajectory()->updateTrajectoryFromFreePoint(free_point_index,
				joint_index);
		handleJointLimits();

		// TODO: below function does not update free var trajectory
		updateFullTrajectory(free_point_index, joint_index);
	}

	int stride = getGroupTrajectory()->getContactPhaseStride();
	if (free_point_index == 0)
		free_point_index = 1;
	int begin = (free_point_index - 1) * stride;

	backup_data_.segment_frames_.resize(2 * stride);

	backup_data_.wrenchSum_.resize(2 * stride + 1);
	backup_data_.linkPositions_.resize(num_mass_segments_);
	backup_data_.linkVelocities_.resize(num_mass_segments_);
	backup_data_.linkAngularVelocities_.resize(num_mass_segments_);
	for (int i = 0; i < num_mass_segments_; ++i)
	{
		backup_data_.linkPositions_[i].resize(2 * stride + 1);
		backup_data_.linkVelocities_[i].resize(2 * stride + 1);
		backup_data_.linkAngularVelocities_[i].resize(2 * stride + 1);
	}
	backup_data_.CoMPositions_.resize(2 * stride + 1);
	backup_data_.CoMVelocities_.resize(2 * stride + 1);
	backup_data_.CoMAccelerations_.resize(2 * stride + 1);
	backup_data_.AngularMomentums_.resize(2 * stride + 1);
	backup_data_.Torques_.resize(2 * stride + 1);
	backup_data_.contactViolationVector_.resize(num_contacts_);
	backup_data_.contactPointVelVector_.resize(num_contacts_);
	for (int i = 0; i < num_contacts_; ++i)
	{
		backup_data_.contactViolationVector_[i].resize(2 * stride + 1);
		backup_data_.contactPointVelVector_[i].resize(2 * stride + 1);
	}

	backup_data_.state_contact_invariant_cost_.resize(2 * stride + 1);
	backup_data_.state_physics_violation_cost_.resize(2 * stride + 1);
	backup_data_.state_collision_cost_.resize(2 * stride);
	backup_data_.state_ftr_cost_.resize(2 * stride);

	for (int i = 0; i < 2 * stride; ++i)
	{
		backup_data_.segment_frames_[i] = data_->segment_frames_[begin + i];
	}

	memcpy(&backup_data_.wrenchSum_[0], &data_->wrenchSum_[begin],
           sizeof(KDL::Wrench) * 2 * stride + 1);
	for (int i = 0; i < num_mass_segments_; ++i)
	{
		memcpy(&backup_data_.linkPositions_[i][0],
               &data_->linkPositions_[i][begin],
               sizeof(KDL::Vector) * 2 * stride + 1);
		memcpy(&backup_data_.linkVelocities_[i][0],
               &data_->linkVelocities_[i][begin],
               sizeof(KDL::Vector) * 2 * stride + 1);
		memcpy(&backup_data_.linkAngularVelocities_[i][0],
               &data_->linkAngularVelocities_[i][begin],
               sizeof(KDL::Vector) * 2 * stride + 1);
	}
	memcpy(&backup_data_.CoMPositions_[0], &data_->CoMPositions_[begin],
           sizeof(KDL::Vector) * 2 * stride + 1);
	memcpy(&backup_data_.CoMVelocities_[0], &data_->CoMVelocities_[begin],
           sizeof(KDL::Vector) * 2 * stride + 1);
	memcpy(&backup_data_.CoMAccelerations_[0], &data_->CoMAccelerations_[begin],
           sizeof(KDL::Vector) * 2 * stride + 1);
	memcpy(&backup_data_.AngularMomentums_[0], &data_->AngularMomentums_[begin],
           sizeof(KDL::Vector) * 2 * stride + 1);
	memcpy(&backup_data_.Torques_[0], &data_->Torques_[begin],
           sizeof(KDL::Vector) * 2 * stride + 1);
	for (int i = 0; i < num_contacts_; ++i)
	{
		memcpy(&backup_data_.contactViolationVector_[i][0],
               &data_->contactViolationVector_[i][begin],
               sizeof(Vector4d) * 2 * stride + 1);
		memcpy(&backup_data_.contactPointVelVector_[i][0],
               &data_->contactPointVelVector_[i][begin],
               sizeof(KDL::Vector) * 2 * stride + 1);
	}

	memcpy(&backup_data_.state_contact_invariant_cost_[0],
           &data_->stateContactInvariantCost_[begin],
           sizeof(double) * 2 * stride + 1);
	memcpy(&backup_data_.state_physics_violation_cost_[0],
           &data_->statePhysicsViolationCost_[begin],
           sizeof(double) * 2 * stride + 1);
	memcpy(&backup_data_.state_collision_cost_[0],
           &data_->stateCollisionCost_[begin], sizeof(double) * 2 * stride);
	memcpy(&backup_data_.state_ftr_cost_[0], &data_->stateFTRCost_[begin],
           sizeof(double) * 2 * stride + 1);
}

void EvaluationManager::restoreVariable(DERIVATIVE_VARIABLE_TYPE variable_type,
                                        int free_point_index, int joint_index)
{
	Eigen::MatrixXd* target_trajectory;
	switch (variable_type)
	{
	case DERIVATIVE_POSITION_VARIABLE:
		target_trajectory = &getGroupTrajectory()->getFreePoints();
		break;

	case DERIVATIVE_VELOCITY_VARIABLE:
		target_trajectory = &getGroupTrajectory()->getFreeVelPoints();
		break;

	case DERIVATIVE_CONTACT_VARIABLE:
		target_trajectory = &getGroupTrajectory()->getContactTrajectory();
		break;
	}

	// restore trajectory value
	(*target_trajectory)(free_point_index, joint_index) =
        backup_data_.trajectory_value_;
	if (variable_type != DERIVATIVE_CONTACT_VARIABLE)
	{
		getGroupTrajectory()->updateTrajectoryFromFreePoint(free_point_index,
				joint_index);
		updateFullTrajectory(free_point_index, joint_index);
	}
	// restore variables
	int stride = getGroupTrajectory()->getContactPhaseStride();
	if (free_point_index == 0)
		free_point_index = 1;
	int begin = (free_point_index - 1) * stride;

	for (int i = 0; i < 2 * stride; ++i)
		data_->segment_frames_[begin + i] = backup_data_.segment_frames_[i];

	memcpy(&data_->wrenchSum_[begin], &backup_data_.wrenchSum_[0],
           sizeof(KDL::Wrench) * 2 * stride + 1);
	for (int i = 0; i < num_mass_segments_; ++i)
	{
		memcpy(&data_->linkPositions_[i][begin],
               &backup_data_.linkPositions_[i][0],
               sizeof(KDL::Vector) * 2 * stride + 1);
		memcpy(&data_->linkVelocities_[i][begin],
               &backup_data_.linkVelocities_[i][0],
               sizeof(KDL::Vector) * 2 * stride + 1);
		memcpy(&data_->linkAngularVelocities_[i][begin],
               &backup_data_.linkAngularVelocities_[i][0],
               sizeof(KDL::Vector) * 2 * stride + 1);
	}
	memcpy(&data_->CoMPositions_[begin], &backup_data_.CoMPositions_[0],
           sizeof(KDL::Vector) * 2 * stride + 1);
	memcpy(&data_->CoMVelocities_[begin], &backup_data_.CoMVelocities_[0],
           sizeof(KDL::Vector) * 2 * stride + 1);
	memcpy(&data_->CoMAccelerations_[begin], &backup_data_.CoMAccelerations_[0],
           sizeof(KDL::Vector) * 2 * stride + 1);
	memcpy(&data_->AngularMomentums_[begin], &backup_data_.AngularMomentums_[0],
           sizeof(KDL::Vector) * 2 * stride + 1);
	memcpy(&data_->Torques_[begin], &backup_data_.Torques_[0],
           sizeof(KDL::Vector) * 2 * stride + 1);
	for (int i = 0; i < num_contacts_; ++i)
	{
		memcpy(&data_->contactViolationVector_[i][begin],
               &backup_data_.contactViolationVector_[i][0],
               sizeof(Vector4d) * 2 * stride + 1);
		memcpy(&data_->contactPointVelVector_[i][begin],
               &backup_data_.contactPointVelVector_[i][0],
               sizeof(KDL::Vector) * 2 * stride + 1);
	}

	memcpy(&data_->stateContactInvariantCost_[begin],
           &backup_data_.state_contact_invariant_cost_[0],
           sizeof(double) * 2 * stride + 1);
	memcpy(&data_->statePhysicsViolationCost_[begin],
           &backup_data_.state_physics_violation_cost_[0],
           sizeof(double) * 2 * stride + 1);
	memcpy(&data_->stateCollisionCost_[begin],
           &backup_data_.state_collision_cost_[0],
           sizeof(double) * 2 * stride);
	memcpy(&data_->stateFTRCost_[begin], &backup_data_.state_ftr_cost_[0],
           sizeof(double) * 2 * stride + 1);
}

double EvaluationManager::evaluateDerivatives(double value,
		DERIVATIVE_VARIABLE_TYPE variable_type, int free_point_index,
		int joint_index)
{
	// backup old values and update trajectory
	backupAndSetVariables(value, variable_type, free_point_index, joint_index);

	// evaluate
	double cost = evaluate(variable_type, free_point_index, joint_index);

	restoreVariable(variable_type, free_point_index, joint_index);

	return cost;
}

void EvaluationManager::render(int trajectory_index, bool is_best)
{
	if (PlanningParameters::getInstance()->getAnimatePath())
	{
        VisualizationManager::getInstance()->animatePath(trajectory_index, getFullTrajectoryConst(), is_best, planning_group_->name_);
        VisualizationManager::getInstance()->animateCollisionSpheres(trajectory_index, getFullTrajectoryConst(), is_best, planning_group_->name_);
	}

	if (PlanningParameters::getInstance()->getAnimateEndeffector())
	{
        VisualizationManager::getInstance()->animateEndeffector(trajectory_index, full_vars_start_, full_vars_end_,
                data_->segment_frames_, is_best);
	}


    const int prediction_frames = 10;
    double current_time = 0.0;
    for (int i = 0; i < 10; ++i)
    {
        bvh_predictor_->moveTo(current_time);
        bvh_predictor_->predict(prediction_frames);
        current_time += 0.05;
    }

    /*
    const int prediction_frames = 10;
    bvh_predictor_->moveTo(0.0);
    for (int i = 0; i < full_vars_start_ + 1; ++i)
    {
        bvh_predictor_->predict(prediction_frames);
        bvh_predictor_->moveToNextFrame();
    }
    */


    for (int future_frame_index = 0; future_frame_index < prediction_frames; future_frame_index++)
    {
        std::vector<Eigen::Vector3d> centers;
        std::vector<Eigen::Matrix3d> A;

        bvh_predictor_->getPredictedEllipsoids(future_frame_index, centers, A);

        for (int j=0; j<centers.size(); j++)
        {
            // an ellipsoid is defined by
            //   (x - centers[j])^T A[j] (x - centers[j]) = 1
        }
    }

    bvh_predictor_->visualizeHuman();
    bvh_predictor_->visualizePredictionUpto(prediction_frames);

}

void EvaluationManager::computeMassAndGravityForce()
{
	total_mass_ = 0.0;
	const KDL::SegmentMap& segmentMap =
        robot_model_->getKDLTree()->getSegments();
	num_mass_segments_ = 0;
	for (KDL::SegmentMap::const_iterator it = segmentMap.begin();
			it != segmentMap.end(); ++it)
	{
		const KDL::Segment& segment = it->second.segment;
		double mass = segment.getInertia().getMass();
		if (mass == 0)
			continue;

		total_mass_ += mass;
		masses_.push_back(mass);

		++num_mass_segments_;
	}
	gravity_force_ = total_mass_ * KDL::Vector(0.0, 0.0, -9.8);

	// normalize gravity force to 1.0 and rescale masses
	gravity_force_ = KDL::Vector(0.0, 0.0, -1.0);
	for (int i = 0; i < masses_.size(); ++i)
		masses_[i] /= total_mass_ * 9.8;
	total_mass_ = 1.0 / 9.8;

}

void EvaluationManager::handleJointLimits()
{
	for (int joint = 0; joint < num_joints_; joint++)
	{
		if (!planning_group_->group_joints_[joint].has_joint_limits_)
			continue;

		double joint_max =
            planning_group_->group_joints_[joint].joint_limit_max_;
		double joint_min =
            planning_group_->group_joints_[joint].joint_limit_min_;

		int count = 0;

		for (int i = 1; i < num_points_ - 2; i++)
		{
			if ((*getGroupTrajectory())(i, joint) > joint_max)
			{
				(*getGroupTrajectory())(i, joint) = joint_max;
			}
			else if ((*getGroupTrajectory())(i, joint) < joint_min)
			{
				(*getGroupTrajectory())(i, joint) = joint_min;
			}
		}
	}
}

void EvaluationManager::handleTrajectoryConstraint()
{
	if (data_->cartesian_waypoints_.size() == 0)
		return;

	// TODO: temp
	// handle cartesian traj
	robot_state::RobotStatePtr kinematic_state(
        new robot_state::RobotState(robot_model_->getRobotModel()));
	const robot_state::JointModelGroup* joint_model_group =
        robot_model_->getRobotModel()->getJointModelGroup(planning_group_->name_);

	KDL::Vector start_pos = data_->cartesian_waypoints_[0].p;
	KDL::Vector end_pos = data_->cartesian_waypoints_[1].p;
	KDL::Vector dir = (end_pos - start_pos);
	dir.Normalize();
	KDL::Rotation orientation = data_->cartesian_waypoints_[0].M;

	int start = 6;

	Eigen::Matrix3d mat;
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			mat(i, j) = orientation(i, j);

	// set state to the start config
	std::vector<double> positions(num_joints_);
	for (std::size_t k = 0; k < num_joints_; k++)
	{
		positions[k] = (*getGroupTrajectory())(5, k);
	}
	kinematic_state->setVariablePositions(&positions[0]);
	kinematic_state->update();

	const int END_EFFECTOR_SEGMENT_INDEX =
        robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(
            "segment_7");
	int num_vars_free = num_points_ - 10 - 2;

	for (int i = start; i < start + num_vars_free; i++)
	{
		int full_traj_index = getGroupTrajectory()->getFullTrajectoryIndex(i);

		for (std::size_t k = 0; k < num_joints_; k++)
			positions[k] = (*getGroupTrajectory())(i, k);
		kinematic_state->setVariablePositions(&positions[0]);
		kinematic_state->update();

		KDL::Frame& frame =
            data_->segment_frames_[i][END_EFFECTOR_SEGMENT_INDEX];
		KDL::Vector proj = start_pos
                           + KDL::dot(dir, (frame.p - start_pos)) * dir;
		if (KDL::dot((proj - start_pos), dir) < 0)
			proj = start_pos;
		if (KDL::dot((proj - end_pos), dir) > 0)
			proj = end_pos;

		proj = start_pos
               + (end_pos - start_pos) * (double) (i - start) / num_vars_free;

		double dist = (frame.p - proj).Norm();

		// Use IK to compute joint values
		vector<double> ik_solution(num_joints_);

		Eigen::Affine3d end_effector_state = Eigen::Affine3d::Identity();
		Eigen::Vector3d trans(proj.x(), proj.y(), proj.z());

		end_effector_state.linear() = mat;
		end_effector_state.translation() = trans;

		kinematics::KinematicsQueryOptions options;
		options.return_approximate_solution = false;
		bool found_ik = kinematic_state->setFromIK(joint_model_group,
                        end_effector_state, 10, 0.1,
                        moveit::core::GroupStateValidityCallbackFn(), options);
		if (found_ik)
		{
			std::vector<double> group_values;
			kinematic_state->copyJointGroupPositions(joint_model_group,
					group_values);
			double* state_pos = kinematic_state->getVariablePositions();
			for (std::size_t k = 0; k < kinematic_state->getVariableCount();
					++k)
			{
				ik_solution[k] = state_pos[k];
				(*getGroupTrajectory())(i, k) = state_pos[k];
				(*getFullTrajectory())(full_traj_index, k) = state_pos[k];
			}
		}
		else
		{
			ROS_INFO("Could not find IK solution for waypoint %d", i);
		}
	}
	performForwardKinematics();

	// check
	for (int i = start; i < start + num_vars_free; i++)
	{
		KDL::Frame& frame =
            data_->segment_frames_[i][END_EFFECTOR_SEGMENT_INDEX];
		KDL::Vector proj = start_pos
                           + KDL::dot(dir, (frame.p - start_pos)) * dir;
		double dist = (frame.p - proj).Norm();
		if (dist > 0.1)
			ROS_INFO("error : %d dist : %f", i, dist);
	}
}

void EvaluationManager::updateFullTrajectory()
{
	getFullTrajectory()->updateFromGroupTrajectory(*getGroupTrajectory());
}

void EvaluationManager::updateFullTrajectory(int point_index, int joint_index)
{
	getFullTrajectory()->updateFromGroupTrajectory(*getGroupTrajectory(),
			point_index, joint_index);
}

bool EvaluationManager::performForwardKinematics(int begin, int end)
{
	double invTime = 1.0 / getGroupTrajectory()->getDiscretization();
	double invTimeSq = invTime * invTime;

	is_collision_free_ = true;

	int safe_begin = max(0, begin);
	int safe_end = min(num_points_, end);

	// used in computeBaseFrames
	int full_traj_index = getGroupTrajectory()->getFullTrajectoryIndex(
                              num_points_ - 1);
	getFullTrajectory()->getTrajectoryPointKDL(full_traj_index,
			data_->kdl_joint_array_);
	data_->fk_solver_.JntToCartFull(data_->kdl_joint_array_,
                                    data_->joint_pos_[num_points_ - 1],
                                    data_->joint_axis_[num_points_ - 1],
                                    data_->segment_frames_[num_points_ - 1]);

	// for each point in the trajectory
	for (int i = safe_begin; i < safe_end; ++i)
	{
		int full_traj_index = getGroupTrajectory()->getFullTrajectoryIndex(i);
		getFullTrajectory()->getTrajectoryPointKDL(full_traj_index,
				data_->kdl_joint_array_);
		// TODO: ?
		/*
		 // update kdl_joint_array with vel, acc
		 if (i < 1)
		 {
		 for (int j = 0; j < planning_group_->num_joints_; j++)
		 {
		 int target_joint = planning_group_->group_joints_[j].kdl_joint_index_;
		 data_->kdl_joint_array_(target_joint) = (*getGroupTrajectory())(i, j);
		 }
		 }
		 */

		//computeBaseFrames(data_->kdl_joint_array_, i);
		if (i == safe_begin)
			data_->fk_solver_.JntToCartFull(data_->kdl_joint_array_,
                                            data_->joint_pos_[i], data_->joint_axis_[i],
                                            data_->segment_frames_[i]);
		else
			data_->fk_solver_.JntToCartFull(data_->kdl_joint_array_,
                                            data_->joint_pos_[i], data_->joint_axis_[i],
                                            data_->segment_frames_[i]);
		// TODO: check patrial FK
		/*
		 data_->fk_solver_.JntToCartPartial(data_->kdl_joint_array_,
		 data_->joint_pos_[i], data_->joint_axis_[i],
		 data_->segment_frames_[i]);
		 */

		data_->state_is_in_collision_[i] = false;

		if (data_->state_is_in_collision_[i])
		{
			is_collision_free_ = false;
		}
	}

	return is_collision_free_;
}

void EvaluationManager::computeTrajectoryValidity()
{
	trajectory_validity_ = true;
	const double clearance = 0.001;
	int collisionBV = 8001;
	visualization_msgs::Marker marker;
	visualization_msgs::Marker markerTemp;

	for (int i = 1; i < num_points_ - 1; i++)
	{
		bool valid = true;

		/*
		 dynamic_obstacle_cost_(i) = 0;

		 int full_traj_index = group_trajectory_->getFullTrajectoryIndex(i);
		 full_trajectory_->getTrajectoryPointKDL(full_traj_index, kdl_joint_array_);
		 for (int j = 0; j < full_trajectory_->getNumJoints(); ++j)
		 {
		 robot_state_.joint_state.position[j] = kdl_joint_array_(j);
		 }

		 planning_environment::setRobotStateAndComputeTransforms(robot_state_, *kinematic_state_);

		 collision_space::EnvironmentModel
		 * env_model =
		 const_cast<collision_space::EnvironmentModel*> (collision_proximity_space_->getCollisionModelsInterface()->getOde());
		 env_model->updateRobotModel(&(*kinematic_state_));

		 // check collision points with dynamic obstacles
		 double point_time = full_trajectory_->getDiscretization() * (i - 1);
		 double cur_time = trajectory_start_time_ - planning_start_time_ + point_time;

		 // TODO: dynamic obs
		 double obstacleScale = 1.1 * (1 + cur_time * SENSOR_NOISE);
		 obstacleScale = 1.0;

		 bool inNextExecution = point_time < PlanningParameters::getInstance()->getPlanningStepSize();
		 for (unsigned int j = 0; j < dynamic_obstacles_->size(); ++j)
		 {
		 const pomp_dynamic_obs_msgs::DynamicObstacle& dynamic_obstacle = dynamic_obstacles_->at(j);

		 btVector3 origin(dynamic_obstacle.x, dynamic_obstacle.y, dynamic_obstacle.z);
		 double obstacle_radius = dynamic_obstacle.lengthX * 0.5 * obstacleScale;
		 }
		 */

		data_->state_validity_[i] = valid;
		if (!valid)
			trajectory_validity_ = false;

	}

}

void EvaluationManager::updateCoM(int point)
{
	const KDL::SegmentMap& segmentMap =
        robot_model_->getKDLTree()->getSegments();
	// compute CoM, p_j
	int mass_segment_index = 0;
	data_->CoMPositions_[point] = KDL::Vector::Zero();
	for (KDL::SegmentMap::const_iterator it = segmentMap.begin();
			it != segmentMap.end(); ++it)
	{
		const KDL::Segment& segment = it->second.segment;
		double mass = segment.getInertia().getMass();
		if (mass == 0.0)
			continue;
		mass = masses_[mass_segment_index];

		int sn = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(
                     segment.getName());
		const KDL::Vector& pos = data_->segment_frames_[point][sn]
                                 * segment.getInertia().getCOG();

		data_->CoMPositions_[point] += pos * mass;
		data_->linkPositions_[mass_segment_index][point] = pos;

		++mass_segment_index;
	}
	data_->CoMPositions_[point] = data_->CoMPositions_[point] / total_mass_;

	if (STABILITY_COST_VERBOSE)
	{
		printf("[%d] CoM Pos : (%f %f %f)\n", point,
               data_->CoMPositions_[point].x(),
               data_->CoMPositions_[point].y(),
               data_->CoMPositions_[point].z());
	}
}

void EvaluationManager::computeCollisionCosts(int begin, int end)
{
    if (PlanningParameters::getInstance()->getObstacleCostWeight() == 0.0)
        return;

	int num_all_joints = data_->kinematic_state_[0]->getVariableCount();

    int num_threads = getNumParallelThreads();

	collision_detection::CollisionRequest collision_request;
	collision_request.verbose = false;
	collision_request.contacts = true;
	collision_request.max_contacts = 1000;

    std::vector<collision_detection::CollisionResult> collision_result(num_threads);
	std::vector<std::vector<double> > positions(num_threads);

	for (int i = 0; i < num_threads; ++i)
	{
		positions[i].resize(num_all_joints);
	}

	int safe_begin = max(0, begin);
	int safe_end = min(num_points_, end);
    #pragma omp parallel for
	for (int i = safe_begin; i < safe_end; ++i)
	{
        int thread_num = omp_get_thread_num();

		double depthSum = 0.0;

		int full_traj_index = getGroupTrajectory()->getFullTrajectoryIndex(i);
		for (std::size_t k = 0; k < num_all_joints; k++)
		{
            positions[thread_num][k] = (*getFullTrajectory())(full_traj_index, k);
		}
        data_->kinematic_state_[thread_num]->setVariablePositions(&positions[thread_num][0]);
        data_->planning_scene_->checkCollisionUnpadded(collision_request, collision_result[thread_num],
				*data_->kinematic_state_[thread_num]);

        const collision_detection::CollisionResult::ContactMap& contact_map = collision_result[thread_num].contacts;
        for (collision_detection::CollisionResult::ContactMap::const_iterator it = contact_map.begin(); it != contact_map.end(); ++it)
		{
			const collision_detection::Contact& contact = it->second[0];

			depthSum += contact.depth;

			last_trajectory_collision_free_ = false;
		}
		collision_result[thread_num].clear();
		data_->stateCollisionCost_[i] = depthSum;
	}
}

std::vector<double> computeFTR(const std::string& group_name,
                               int contact_point_index, int begin, int end, const EvaluationData* data,
                               const ItompPlanningGroup * planning_group)
{
	std::vector<double> positions, trajectory_ftrs;
	int num_joints = data->getFullTrajectory()->getNumJoints();
	positions.resize(num_joints);
	for (int i = begin; i < end; ++i)
	{
		int full_traj_index =
            data->getGroupTrajectory()->getFullTrajectoryIndex(i);
		double cost = 0;
		for (std::size_t k = 0; k < num_joints; k++)
		{
			positions[k] = (*data->getFullTrajectory())(full_traj_index, k);
		}
		data->kinematic_state_[0]->setVariablePositions(&positions[0]);
		robot_model::RobotModelConstPtr robot_model_ptr =
            data->getItompRobotModel()->getRobotModel();
		Eigen::MatrixXd jacobianFull = (data->kinematic_state_[0]->getJacobian(
                                            robot_model_ptr->getJointModelGroup(group_name)));
		Eigen::MatrixXd jacobian = jacobianFull.block(0, 0, 3,
                                   jacobianFull.cols());
		Eigen::MatrixXd jacobian_transpose = jacobian.transpose();

		// computing direction, first version as COM velocity between poses
		const KDL::Vector& dir_kdl =
            data->contact_forces_[i][contact_point_index];
		Eigen::Vector3d direction(dir_kdl.x(), dir_kdl.y(), dir_kdl.z());
		if (direction.norm() != 0)
		{
			direction.normalize();
			double ftr = 1
                         / std::sqrt(
                             direction.transpose()
                             * (jacobian * jacobian_transpose)
                             * direction);
			KDL::Vector position, unused, normal;
			planning_group->contactPoints_[contact_point_index].getPosition(i,
					position, data->segment_frames_);
			GroundManager::getInstance().getNearestGroundPosition(position,
					unused, normal, data->planning_scene_); // TODO get more accurate normal
			Eigen::Vector3d normalEigen(normal.x(), normal.y(), normal.z());
			double contact_variable =
                data->getGroupTrajectory()->getContactTrajectory()(
                    i
                    / data->getGroupTrajectory()->getContactPhaseStride(),
                    contact_point_index);

			ftr *= -direction.dot(normalEigen);
			// bound value btw -10 and 10, then 0 and 1
			ftr = (ftr < -10) ? -10 : ftr;
			ftr = (ftr > 10) ? 10 : ftr;
			ftr = (ftr + 10) / 20;
			cost = dir_kdl.Norm() - ftr;
			cost = (cost < 0) ? 0 : cost;
		}
		trajectory_ftrs.push_back(cost);
	}
	return trajectory_ftrs;
}

void EvaluationManager::computeFTRs(int begin, int end)
{
	int safe_begin = max(0, begin);
	int safe_end = min(num_points_, end);
	std::vector<double> left_leg_cost = computeFTR("left_leg", 0, safe_begin,
                                        safe_end, data_, planning_group_);
	std::vector<double> right_leg_cost = computeFTR("right_leg", 1, safe_begin,
                                         safe_end, data_, planning_group_);
	std::vector<double> left_arm_cost = computeFTR("left_arm", 2, safe_begin,
                                        safe_end, data_, planning_group_);
	std::vector<double> right_arm_cost = computeFTR("right_arm", 3, safe_begin,
                                         safe_end, data_, planning_group_);
	for (unsigned int i = safe_begin; i < safe_end; ++i)
	{
		int v_index = i - safe_begin;
		data_->stateFTRCost_[i] = (left_leg_cost[v_index]
                                   + right_leg_cost[v_index])
                                  + 0.5 * (left_arm_cost[v_index] + right_arm_cost[v_index]);
	}

}

void EvaluationManager::printDebugInfo()
{
    return;

	if (PlanningParameters::getInstance()->getCartesianTrajectoryCostWeight()
			== 0.0)
		return;

    // TODO : safety constraint
    {
        const int END_EFFECTOR_SEGMENT_INDEX = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex("revetting_tcp");

        for (int i = 0; i < num_points_; ++i)
            data_->stateCartesianTrajectoryCost_[i] = 0;

        int num_vars_free = num_points_ - 10 - 2;

        int point_index = 6;
        for (int i = point_index; i < point_index + num_vars_free; ++i)
        {
            KDL::Frame& frame = data_->segment_frames_[i][END_EFFECTOR_SEGMENT_INDEX];
            KDL::Frame& frame_prev = data_->segment_frames_[i - 1][END_EFFECTOR_SEGMENT_INDEX];
            KDL::Frame& frame_next = data_->segment_frames_[i + 1][END_EFFECTOR_SEGMENT_INDEX];

            KDL::Vector tangent = frame_next.p - frame_prev.p;
            double norm = tangent.Norm();
            tangent.Normalize();
            KDL::Vector z_dir = frame.M * KDL::Vector(0, 0, 1.0);
            double dot = (norm < 0.001) ? 0.0 : KDL::dot(z_dir, tangent);// + 0.1;
            ROS_INFO("safety [%d] z_dir : %f %f %f tangent : %f %f %f dot : %f", i,
                     z_dir.x(), z_dir.y(), z_dir.z(),
                     tangent.x(), tangent.y(), tangent.z(), dot);
        }

    }

	if (data_->cartesian_waypoints_.size() != 0)
	{
		// position constraint

		const int END_EFFECTOR_SEGMENT_INDEX =
            robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(
                "tcp_2_link");

		data_->costAccumulator_.is_last_trajectory_valid_ = true;

		const double rotation_weight = 0.0;

		int num_vars_free = 100;

		if (data_->cartesian_waypoints_.size() == 0)
			return;

		KDL::Vector start_pos = data_->cartesian_waypoints_[0].p;
		KDL::Vector end_pos = data_->cartesian_waypoints_[1].p;
		KDL::Vector dir = (end_pos - start_pos);
		double dir_max = dir.Normalize();

		double max_dist = 0;

		// TODO: fix
		int point_index = 6;
		for (int i = point_index; i < point_index + num_vars_free; ++i)
		{
			KDL::Frame& frame =
                data_->segment_frames_[i][END_EFFECTOR_SEGMENT_INDEX];

			double proj_dist = KDL::dot(dir, (frame.p - start_pos));
			KDL::Vector proj_pt = proj_dist * dir;
			if (proj_dist < 0)
				proj_pt = KDL::Vector::Zero();
			else if (proj_dist > dir_max)
				proj_pt = dir_max * dir;

			KDL::Vector distToLine = (frame.p - start_pos) - proj_pt;
			double distance = distToLine.Norm();

            //printf("[%d] dist: %f (%f %f %f)\n", i, distance, frame.p.x(), frame.p.y(), frame.p.z());

			if (distance > max_dist)
				max_dist = distance;
		}
        //printf("Max dist : %f\n", max_dist);

	}
}

void EvaluationManager::computeCartesianTrajectoryCosts()
{
	if (PlanningParameters::getInstance()->getCartesianTrajectoryCostWeight()
			== 0.0)
		return;

    // TODO : safety constraint
    {
        const int END_EFFECTOR_SEGMENT_INDEX = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex("revetting_tcp");

        for (int i = 0; i < num_points_; ++i)
            data_->stateCartesianTrajectoryCost_[i] = 0;

        int num_vars_free = num_points_ - 10 - 2;

        int point_index = 6;
        for (int i = point_index; i < point_index + num_vars_free; ++i)
        {
            KDL::Frame& frame = data_->segment_frames_[i][END_EFFECTOR_SEGMENT_INDEX];
            KDL::Frame& frame_prev = data_->segment_frames_[i - 1][END_EFFECTOR_SEGMENT_INDEX];
            KDL::Frame& frame_next = data_->segment_frames_[i + 1][END_EFFECTOR_SEGMENT_INDEX];

            KDL::Vector tangent = frame_next.p - frame_prev.p;
            double norm = tangent.Norm();
            tangent.Normalize();
            if (norm > 0.001)
            {
                KDL::Vector z_dir = frame.M * KDL::Vector(0, 0, 1.0);
                double dot = KDL::dot(z_dir, tangent) + 0.1;
                if (dot > 0.0)
                {
                    data_->stateCartesianTrajectoryCost_[i] = dot * dot;
                    last_trajectory_collision_free_ = false;
                }
            }
        }

    }

	if (data_->cartesian_waypoints_.size() != 0)
	{
		// position constraint

        const int END_EFFECTOR_SEGMENT_INDEX = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex("tcp_2_link");

		const double rotation_weight = 0.0;

		int num_vars_free = 100;

		for (int i = 0; i < num_points_; ++i)
			data_->stateCartesianTrajectoryCost_[i] = 0;

		if (data_->cartesian_waypoints_.size() == 0)
			return;

		KDL::Vector start_pos = data_->cartesian_waypoints_[0].p;
		KDL::Vector end_pos = data_->cartesian_waypoints_[1].p;
		KDL::Vector dir = (end_pos - start_pos);
		double dir_max = dir.Normalize();

		// TODO: fix
		int point_index = 6;
		for (int i = point_index; i < point_index + num_vars_free; ++i)
		{
            KDL::Frame& frame = data_->segment_frames_[i][END_EFFECTOR_SEGMENT_INDEX];

            // linear
            //double t = min_jerk_curve_[i - (point_index - 1)];
            double t = (double)(i - (point_index - 1)) / (num_vars_free);
            KDL::Vector cartesian_pos = start_pos * (1.0 - t) + end_pos * t;
            double distance = (frame.p - cartesian_pos).Norm();

            if (distance > 0.01)
                last_trajectory_collision_free_ = false;

            data_->stateCartesianTrajectoryCost_[i] = distance * distance;


            /*
			double proj_dist = KDL::dot(dir, (frame.p - start_pos));
			KDL::Vector proj_pt = proj_dist * dir;
			if (proj_dist < 0)
				proj_pt = KDL::Vector::Zero();
			else if (proj_dist > dir_max)
				proj_pt = dir_max * dir;

			KDL::Vector distToLine = (frame.p - start_pos) - proj_pt;
			double distance = distToLine.Norm();

			double cost = distance;

			if (distance > 0.005)
				last_trajectory_collision_free_ = false;

			data_->stateCartesianTrajectoryCost_[i] = cost * cost;
            */
		}
	}
	else
	{
		if (planning_group_->name_ != "lower_body")
			return;

		// orientation constraint

		// TODO: fix hard-coded values
		const int END_EFFECTOR_SEGMENT_INDEX =
            robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(
                "segment_7");

		data_->costAccumulator_.is_last_trajectory_valid_ = true;

		// orientation constraint
		int num_vars_free = 100;
		int point_index = 5;
		for (int i = point_index; i < point_index + num_vars_free; ++i)
		{
			KDL::Frame& frame =
                data_->segment_frames_[i][END_EFFECTOR_SEGMENT_INDEX];
			KDL::Rotation rot = frame.M;
			KDL::Vector x_dir = rot.UnitX();
			KDL::Vector y_dir = rot.UnitY();
			KDL::Vector z_dir = rot.UnitZ();

			double cost = 0;
			double dot = KDL::dot(y_dir, KDL::Vector(0, 0, -1));
			double angle = (dot > 1.0) ? 0 : acos(dot);
			//angle -= 5.0 * M_PI / 180.0;
			// TODO: ?
			if (angle > 5.0 * M_PI / 180.0)
			{
				//data_->costAccumulator_.is_last_trajectory_valid_ = false;
				last_trajectory_collision_free_ = false;
                cost = angle * angle;
			}
			else
			{
				cost = angle * angle * 0.1;
			}

			data_->stateCartesianTrajectoryCost_[i] = cost;
		}
	}
}

double EvaluationManager::getTrajectoryCost(int trajectory_index, bool verbose)
{
	if (verbose)
        data_->costAccumulator_.print(trajectory_index, *iteration_);
	return data_->costAccumulator_.getTrajectoryCost();
}

////////////////////////////////////////////////////////////////////////////////
void computeFrameDebug(int point, const char* label, const char* v1_name,
                       KDL::Frame& f1, const char* v2_name, KDL::Frame& f2,
                       const char* v3_name, KDL::Frame& f3)
{
	// %.14lf?
	//printf("[%d] %s %s(%.14lf %.14lf %.14lf %f) %s(%.14lf %.14lf %.14lf %f) %s(%.14lf %.14lf %.14lf %f)\n", point, label, v1_name,
	printf("[%d] %s %s(%f %f %f %f) %s(%f %f %f %f) %s(%f %f %f %f)\n", point,
           label, v1_name, f1.p.x(), f1.p.y(), f1.p.z(),
           atan2(f1.M.data[3], f1.M.data[0]) * 180.0 / M_PI, v2_name, f2.p.x(),
           f2.p.y(), f2.p.z(),
           atan2(f2.M.data[3], f2.M.data[0]) * 180.0 / M_PI, v3_name, f3.p.x(),
           f3.p.y(), f3.p.z(),
           atan2(f3.M.data[3], f3.M.data[0]) * 180.0 / M_PI);
}
//#define DEBUG_COMPUTE_FRAME
#ifdef DEBUG_COMPUTE_FRAME
#define COMPUTE_FRAME_DEBUG(point, label, v1_name, v1, v2_name, v2, v3_name, v3) computeFrameDebug(point, label, v1_name, v1, v2_name, v2, v3_name, v3);
#else
#define COMPUTE_FRAME_DEBUG(point, label, v1_name, v1, v2_name, v2, v3_name, v3)
#endif

void EvaluationManager::computeSingularityCosts(int begin, int end)
{
	const string group_name = planning_group_->name_;

	if (PlanningParameters::getInstance()->getSingularityCostWeight() == 0.0)
		return;

	return;

	double min_singular_value = std::numeric_limits<double>::max();
	int min_singular_value_index = begin;

	std::vector<double> positions;
	int num_joints = data_->getFullTrajectory()->getNumJoints();
	positions.resize(num_joints);
	for (int i = begin; i < end; ++i)
	{
		data_->stateSingularityCost_[i] = 0.0;

		int full_traj_index =
            data_->getGroupTrajectory()->getFullTrajectoryIndex(i);
		double cost = 0;
		for (std::size_t k = 0; k < num_joints; k++)
		{
			positions[k] = (*data_->getFullTrajectory())(full_traj_index, k);
		}
		int sz = data_->kinematic_state_[0]->getVariableCount();
		data_->kinematic_state_[0]->setVariablePositions(&positions[0]);
		data_->kinematic_state_[0]->update();
		robot_model::RobotModelConstPtr robot_model_ptr =
            data_->getItompRobotModel()->getRobotModel();

		const moveit::core::JointModelGroup* jmg =
            robot_model_ptr->getJointModelGroup(group_name);
		Eigen::MatrixXd jacobianFull = (data_->kinematic_state_[0]->getJacobian(
                                            jmg));

		int rows2 = jacobianFull.rows();
		cout << jacobianFull << endl;
		for (int k = 0; k < num_joints; ++k)
			printf("%f ", positions[k]);
		printf("\n");

		Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobianFull);
		cout << svd.singularValues() << endl;

		/*
		 printf("[%d] svd values : ", i);
		 for (int j = 0; j < svd.singularValues().rows(); ++j)
		 {
		 printf("%f ", svd.singularValues()(j));
		 }
		 printf("\n");
		 */

		int rows = svd.singularValues().rows();
		double value = (rows == 0 ? 0.0 : svd.singularValues()(rows - 1));
		if (value < min_singular_value)
		{
			min_singular_value = value;
			min_singular_value_index = i;
		}
	}
	if (print_debug_texts_)
		printf("Min Singular value : (%d) %f ", min_singular_value_index,
               min_singular_value);
}

void EvaluationManager::computePointCloudCosts(int begin, int end)
{
    if (PlanningParameters::getInstance()->getPointCloudCostWeight() == 0.0)
        return;

    int num_all_joints = data_->kinematic_state_[0]->getVariableCount();

    int num_threads = getNumParallelThreads();

    std::vector<std::vector<double> > positions(num_threads);

    for (int i = 0; i < num_threads; ++i)
    {
        positions[i].resize(num_all_joints);
    }

    int safe_begin = max(0, begin);
    int safe_end = min(num_points_, end);

    const int replanning_frames = 3;
    const int prediction_frames = replanning_frames * 2;
    bvh_predictor_->moveTo(0.0);
    int current_point = 0;
    while (true)
    {
        bvh_predictor_->predict(prediction_frames);

        //#pragma omp parallel for
        for (int i = current_point + replanning_frames; i < current_point + prediction_frames; ++i)
        {
            if (i < safe_begin)
                continue;
            if (i >= safe_end)
                continue;

            int future_frame_index = i - current_point;

            std::vector<Eigen::Vector3d> mu;
            std::vector<Eigen::Matrix3d> sigma;
            std::vector<double> obstacle_sphere_sizes;
            bvh_predictor_->getPredictedGaussianDistribution(future_frame_index, mu, sigma, obstacle_sphere_sizes);
            std::vector<Eigen::Matrix3d> sigma_inverse(sigma.size());
            std::vector<double> determinant(sigma.size());
            for (unsigned int i = 0; i < sigma.size(); ++i)
            {
                sigma_inverse[i] = sigma[i].inverse();
                determinant[i] = sigma[i].determinant();
            }

            int thread_num = omp_get_thread_num();
            robot_state::RobotStatePtr& robot_state = data_->kinematic_state_[thread_num];

            double max_collision_probability = 0.0;

            int full_traj_index = getGroupTrajectory()->getFullTrajectoryIndex(i);
            for (std::size_t k = 0; k < num_all_joints; k++)
            {
                positions[thread_num][k] = (*getFullTrajectory())(full_traj_index, k);
            }
            robot_state->setVariablePositions(&positions[thread_num][0]);
            robot_state->updateLinkTransforms();

            const std::map<std::string, std::vector<CollisionSphere> >& collision_spheres_map = robot_model_->getRobotCollisionModel().getCollisionSpheres();
            for (std::map<std::string, std::vector<CollisionSphere> >::const_iterator it = collision_spheres_map.begin(); it != collision_spheres_map.end() ; ++it)
            {
                const Eigen::Affine3d& transform = robot_state->getGlobalLinkTransform(it->first);
                const std::vector<CollisionSphere>& collision_spheres = it->second;

                for (unsigned int i = 0; i < collision_spheres.size(); ++i)
                {
                    Eigen::Vector3d global_position = point_cloud_transform_ * transform * collision_spheres[i].position_;

                    for (unsigned int j = 0; j < mu.size(); ++j)
                    {
                        double radius = collision_spheres[i].radius_ + obstacle_sphere_sizes[j];
                        double volume = 4.0 / 3.0 * M_PI * radius * radius * radius;
                        Eigen::Vector3d diff = global_position - mu[j];
                        double max_point_probability = 1.0 / std::sqrt(std::pow(2 * M_PI, 3) * determinant[j]) * std::exp(-0.5 * diff.transpose() * sigma_inverse[i] * diff);
                        max_collision_probability = std::max(max_collision_probability, std::min(1.0, max_point_probability * volume));
                    }
                }
            }
            // 95%?
            max_collision_probability = std::max(0.0, max_collision_probability - 0.05);
            data_->statePointCloudCost_[i] = max_collision_probability * max_collision_probability;
        }

        for (int i = 0; i < replanning_frames; ++i)
            bvh_predictor_->moveToNextFrame();
        current_point += replanning_frames;
        if (current_point >= safe_end)
            break;
    }
}

}

