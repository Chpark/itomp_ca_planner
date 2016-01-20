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
#ifndef EVALUATION_DATA_H_
#define EVALUATION_DATA_H_

#include <itomp_ca_planner/common.h>
#include <itomp_ca_planner/model/itomp_robot_model.h>
#include <itomp_ca_planner/trajectory/itomp_cio_trajectory.h>
#include <itomp_ca_planner/cost/smoothness_cost.h>
#include <itomp_ca_planner/cost/trajectory_cost_accumulator.h>
#include <itomp_ca_planner/util/vector_util.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <Eigen/StdVector>
#include <moveit/planning_scene/planning_scene.h>
#include <itomp_ca_planner/contact/contact_force_solver.h>

namespace itomp_ca_planner
{
class EvaluationManager;
class ItompPlanningGroup;
class EvaluationData
{
public:
  EvaluationData();
  virtual ~EvaluationData();

  void initialize(ItompCIOTrajectory *full_trajectory, ItompCIOTrajectory *group_trajectory,
      ItompRobotModel *robot_model, const ItompPlanningGroup *planning_group,
      const EvaluationManager* evaluation_manager, int num_mass_segments,
      const moveit_msgs::Constraints& path_constraints,
      const planning_scene::PlanningSceneConstPtr& planning_scene);

  double getNumPoints() const;
  double getNumJoints() const;

  void setTrajectories(ItompCIOTrajectory* group_trajectory, ItompCIOTrajectory* full_trajectory);

  ItompCIOTrajectory* getGroupTrajectory();
  const ItompCIOTrajectory* getGroupTrajectory() const;
  ItompCIOTrajectory* getFullTrajectory();
  const ItompCIOTrajectory* getFullTrajectory() const;
  const ItompRobotModel* getItompRobotModel() const;

  KDL::JntArray kdl_joint_array_;

  std::vector<itomp_ca_planner::SmoothnessCost> joint_costs_;

  std::vector<std::vector<KDL::Vector> > joint_axis_;
  std::vector<std::vector<KDL::Vector> > joint_pos_;
  std::vector<std::vector<KDL::Frame> > segment_frames_;

  std::vector<int> state_is_in_collision_;
  std::vector<int> state_validity_;

  Eigen::VectorXd dynamic_obstacle_cost_;

  // physics
  std::vector<KDL::Wrench> wrenchSum_;
  std::vector<std::vector<KDL::Vector> > linkPositions_;
  std::vector<std::vector<KDL::Vector> > linkVelocities_;
  std::vector<std::vector<KDL::Vector> > linkAngularVelocities_;
  std::vector<KDL::Vector> CoMPositions_;
  std::vector<KDL::Vector> CoMVelocities_;
  std::vector<KDL::Vector> CoMAccelerations_;
  std::vector<KDL::Vector> AngularMomentums_;
  std::vector<KDL::Vector> Torques_;
  std::vector<std::vector<Vector4d> > contactViolationVector_;
  std::vector<std::vector<KDL::Vector> > contactPointVelVector_;

  std::vector<double> stateContactInvariantCost_;
  std::vector<double> statePhysicsViolationCost_;
  std::vector<double> stateCollisionCost_;
  std::vector<double> stateFTRCost_;
  std::vector<double> stateCartesianTrajectoryCost_;
  std::vector<double> stateSingularityCost_;
  std::vector<double> statePointCloudCost_;

  std::vector<std::vector<KDL::Vector> > contact_forces_;

  TrajectoryCostAccumulator costAccumulator_;

  KDL::TreeFkSolverJointPosAxisPartial fk_solver_;
  ContactForceSolver contact_force_solver_;

  planning_scene::PlanningSceneConstPtr planning_scene_;
  std::vector<robot_state::RobotStatePtr> kinematic_state_;

  std::vector<KDL::Frame> cartesian_waypoints_;

  EvaluationData* clone() const;
  void deepCopy(const EvaluationData& data);

  void compare(const EvaluationData& ref) const;

protected:
  void initStaticEnvironment();

  ItompRobotModel *robot_model_;

  ItompCIOTrajectory* group_trajectory_;
  ItompCIOTrajectory* full_trajectory_;
};
typedef boost::shared_ptr<EvaluationData> EvaluationDataPtr;

inline ItompCIOTrajectory* EvaluationData::getGroupTrajectory()
{
  return group_trajectory_;
}

inline const ItompCIOTrajectory* EvaluationData::getGroupTrajectory() const
{
  return group_trajectory_;
}

inline ItompCIOTrajectory* EvaluationData::getFullTrajectory()
{
  return full_trajectory_;
}

inline const ItompCIOTrajectory* EvaluationData::getFullTrajectory() const
{
  return full_trajectory_;
}
inline const ItompRobotModel* EvaluationData::getItompRobotModel() const
{
  return robot_model_;
}
inline double EvaluationData::getNumPoints() const
{
  return group_trajectory_->getNumPoints();
}
inline double EvaluationData::getNumJoints() const
{
  return group_trajectory_->getNumJoints();
}

inline void EvaluationData::setTrajectories(ItompCIOTrajectory* group_trajectory, ItompCIOTrajectory* full_trajectory)
{
  group_trajectory_ = group_trajectory;
  full_trajectory_ = full_trajectory;
}

}

#endif
