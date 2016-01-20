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

#include <itomp_ca_planner/optimization/evaluation_data.h>
#include <itomp_ca_planner/cost/trajectory_cost.h>
#include <itomp_ca_planner/util/planning_parameters.h>

using namespace std;

namespace itomp_ca_planner
{
//static int LeftLegStart = 0;

TrajectoryCostPtr TrajectoryCost::CreateTrajectoryCost(COST_TYPE type)
{
    TrajectoryCostPtr newCost;
    switch (type)
    {
    case COST_SMOOTHNESS:
        newCost.reset(new TrajectorySmoothnessCost());
        break;

    case COST_COLLISION:
        newCost.reset(new TrajectoryCollisionCost());
        break;

    case COST_VALIDITY:
        newCost.reset(new TrajectoryValidityCost());
        break;

    case COST_CONTACT_INVARIANT:
        newCost.reset(new TrajectoryContactInvariantCost());
        break;

    case COST_PHYSICS_VIOLATION:
        newCost.reset(new TrajectoryPhysicsViolationCost());
        break;

    case COST_GOAL_POSE:
        newCost.reset(new TrajectoryGoalPoseCost());
        break;

    case COST_COM:
        newCost.reset(new TrajectoryCoMCost());
        break;

    case COST_FTR:
        newCost.reset(new TrajectoryFTRCost());
        break;

    case COST_CARTESIAN_TRAJECTORY:
        newCost.reset(new TrajectoryCartesianCost());
        break;

    case COST_SINGULARITY:
        newCost.reset(new TrajectorySingularityCost());
        break;

    case COST_POINT_CLOUD:
        newCost.reset(new TrajectoryPointCloudCost());
        break;

    case COST_ENDEFFECTOR_VELOCITY:
    case COST_TORQUE:
    case COST_RVO:
    default:
        assert(false);
        break;
    }

    return newCost;
}

////////////////////////////////////////////////////////////////////////////////

void TrajectoryCost::init(const EvaluationData* data)
{

}

void TrajectoryCost::computeCostSum(const EvaluationData* data, Eigen::VectorXd& costData, double& sum)
{
    sum = 0.0;
    for (int i = 1; i <= data->getNumPoints() - 2; i++)
    {
        sum += costData(i);
    }
}

void TrajectoryCost::compute(const EvaluationData* data, Eigen::VectorXd& costData, double& sum)
{
    doCompute(data, costData);
    computeCostSum(data, costData, sum);
}
////////////////////////////////////////////////////////////////////////////////

void TrajectorySmoothnessCost::doCompute(const EvaluationData* data, Eigen::VectorXd& costData)
{
    double smoothness_cost = 0.0;
    // joint costs:
    for (int i = 0; i < data->getNumJoints(); i++)
        smoothness_cost += data->joint_costs_[i].getCost(data->getGroupTrajectory()->getJointTrajectory(i));

    costData(1) = smoothness_cost;
}

double TrajectorySmoothnessCost::getWeight() const
{
    return PlanningParameters::getInstance()->getSmoothnessCostWeight();
}
////////////////////////////////////////////////////////////////////////////////

void TrajectoryCollisionCost::doCompute(const EvaluationData* data, Eigen::VectorXd& costData)
{
    for (int i = 1; i <= data->getNumPoints() - 2; i++)
    {
        costData(i) = data->stateCollisionCost_[i];
    }
}

double TrajectoryCollisionCost::getWeight() const
{
    return PlanningParameters::getInstance()->getObstacleCostWeight();
}
////////////////////////////////////////////////////////////////////////////////

void TrajectoryValidityCost::doCompute(const EvaluationData* data, Eigen::VectorXd& costData)
{
    for (int i = 1; i <= data->getNumPoints() - 2; i++)
    {
        double state_validity_cost = 0.0;
        if (!data->state_validity_[i])
            state_validity_cost = 1.0;

        costData(i) = state_validity_cost;
    }
}

double TrajectoryValidityCost::getWeight() const
{
    return PlanningParameters::getInstance()->getStateValidityCostWeight();
}
////////////////////////////////////////////////////////////////////////////////

void TrajectoryContactInvariantCost::doCompute(const EvaluationData* data, Eigen::VectorXd& costData)
{
    for (int i = 1; i <= data->getNumPoints() - 2; i++)
    {
        costData(i) = data->stateContactInvariantCost_[i];
    }
}

double TrajectoryContactInvariantCost::getWeight() const
{
    return PlanningParameters::getInstance()->getContactInvariantCostWeight();
}
////////////////////////////////////////////////////////////////////////////////

void TrajectoryPhysicsViolationCost::doCompute(const EvaluationData* data, Eigen::VectorXd& costData)
{
    for (int i = 1; i <= data->getNumPoints() - 2; i++)
    {
        costData(i) = data->statePhysicsViolationCost_[i];
    }
}

double TrajectoryPhysicsViolationCost::getWeight() const
{
    return PlanningParameters::getInstance()->getPhysicsViolationCostWeight();
}
////////////////////////////////////////////////////////////////////////////////

void TrajectoryGoalPoseCost::doCompute(const EvaluationData* data, Eigen::VectorXd& costData)
{
    double cost = 0.0;

    int goal_index = data->getNumPoints() - (DIFF_RULE_LENGTH - 1);
    const Eigen::MatrixXd::ConstRowXpr goal_point = data->getGroupTrajectory()->getTrajectoryPoint(goal_index);

    // TODO:
    const std::vector<ItompRobotJoint>& itomp_joints = data->getItompRobotModel()->getPlanningGroup("ur5_m2")->group_joints_;

    for (int i = 0; i < data->getNumJoints(); ++i)
    {
        const ItompRobotJoint& joint = itomp_joints[i];
        double mid = (joint.joint_limit_max_ + joint.joint_limit_min_) * 0.5;
        double diff = goal_point(i) - mid;
        cost += diff * diff;
    }

    costData(goal_index) = cost;
    /*
     double goal_pose_cost = 0.0;
     if (evaluator->planning_group_->name_ == "lower_body" || evaluator->planning_group_->name_ == "whole_body")
     {
     goal_pose_cost += (evaluator->getSegmentPosition(evaluator->free_vars_end_,
     PlanningParameters::getInstance()->getLowerBodyRoot()) - evaluator->getSegmentPosition(
     evaluator->free_vars_end_ + 1, PlanningParameters::getInstance()->getLowerBodyRoot())).Norm();
     goal_pose_cost += (evaluator->getSegmentPosition(evaluator->free_vars_end_, "left_foot_endeffector_link")
     - evaluator->getSegmentPosition(evaluator->free_vars_end_ + 1, "left_foot_endeffector_link")).Norm();

     KDL::Vector rightFoot =
     (evaluator->getSegmentPosition(evaluator->free_vars_end_, "right_foot_endeffector_link")
     - evaluator->getSegmentPosition(evaluator->free_vars_end_ + 1, "right_foot_endeffector_link"));
     goal_pose_cost += rightFoot.Norm();
     }

     for (int i = 0; i < evaluator->num_points_ - 2; i++)
     {
     costs_(i) = goal_pose_cost / evaluator->num_points_ - 2;
     }
     */
}

double TrajectoryGoalPoseCost::getWeight() const
{
    return PlanningParameters::getInstance()->getGoalPoseCostWeight();
}
////////////////////////////////////////////////////////////////////////////////

void TrajectoryCoMCost::doCompute(const EvaluationData* data, Eigen::VectorXd& costData)
{
    /*
     for (int i = evaluator->free_vars_start_; i <= evaluator->free_vars_end_; i++)
     {
     double state_CoM_cost = 0.0;
     if (evaluator->planning_group_->name_ == "lower_body" || evaluator->planning_group_->name_ == "whole_body")
     {
     state_CoM_cost += evaluator->CoMAccelerations_[i].Norm();
     if (evaluator->CoMVelocities_[i].x() < 0.0)
     state_CoM_cost += -evaluator->CoMVelocities_[i].x()
     * PlanningParameters::getInstance()->getTemporaryVariable(0);

     // check CoM is in two legs
     const string contact_foot = (evaluator->group_trajectory_->getContactPhase(i)/2
     + LeftLegStart) % 2 == 0 ? "left_foot_endeffector_link"
     : "right_foot_endeffector_link";
     const string other_foot = (evaluator->group_trajectory_->getContactPhase(i)/2
     + LeftLegStart) % 2 == 1 ? "left_foot_endeffector_link"
     : "right_foot_endeffector_link";
     int ef_sn = evaluator->robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(contact_foot);
     int of_sn = evaluator->robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(other_foot);
     KDL::Vector ef = evaluator->segment_frames_[i][ef_sn].p;
     KDL::Vector of = evaluator->segment_frames_[i][of_sn].p;
     double maxX = max(ef.x(), of.x()) - 0.028;
     double minX = min(ef.x(), of.x()) - 0.028;
     double centerX = (ef.x() + of.x()) * 0.5f - 0.028;
     state_CoM_cost += abs(evaluator->CoMPositions_[i].x() - centerX)
     * PlanningParameters::getInstance()->getTemporaryVariable(1);

     state_CoM_cost /= (evaluator->free_vars_end_ - evaluator->free_vars_start_ + 1);
     }

     costs_(i - evaluator->free_vars_start_) = state_CoM_cost;
     }
     */
}

double TrajectoryCoMCost::getWeight() const
{
    return PlanningParameters::getInstance()->getCoMCostWeight();
}

////////////////////////////////////////////////////////////////////////////////
void TrajectoryFTRCost::doCompute(const EvaluationData* data, Eigen::VectorXd& costData)
{
    for (int i = 1; i <= data->getNumPoints() - 2; i++)
    {
        costData(i) = data->stateFTRCost_[i];
    }
}

double TrajectoryFTRCost::getWeight() const
{
    return PlanningParameters::getInstance()->getFTRCostWeight();
}

////////////////////////////////////////////////////////////////////////////////
void TrajectoryCartesianCost::doCompute(const EvaluationData* data, Eigen::VectorXd& costData)
{
    for (int i = 0; i < data->getNumPoints(); i++)
    {
        costData(i) = data->stateCartesianTrajectoryCost_[i];
    }
}

double TrajectoryCartesianCost::getWeight() const
{
    return PlanningParameters::getInstance()->getCartesianTrajectoryCostWeight();
}

////////////////////////////////////////////////////////////////////////////////
void TrajectorySingularityCost::doCompute(const EvaluationData* data, Eigen::VectorXd& costData)
{
    for (int i = 0; i < data->getNumPoints(); i++)
    {
        costData(i) = data->stateSingularityCost_[i];
    }
}

double TrajectorySingularityCost::getWeight() const
{
    return PlanningParameters::getInstance()->getSingularityCostWeight();
}

////////////////////////////////////////////////////////////////////////////////
void TrajectoryPointCloudCost::doCompute(const EvaluationData* data, Eigen::VectorXd& costData)
{
    for (int i = 0; i < data->getNumPoints(); i++)
    {
        costData(i) = data->statePointCloudCost_[i];
    }
}

double TrajectoryPointCloudCost::getWeight() const
{
    return PlanningParameters::getInstance()->getPointCloudCostWeight();
}

}
