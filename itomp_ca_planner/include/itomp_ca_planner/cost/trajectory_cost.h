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

#ifndef TRAJECTORYCOST_H_
#define TRAJECTORYCOST_H_
#include <itomp_ca_planner/common.h>

namespace itomp_ca_planner
{
class EvaluationData;
class TrajectoryCost
{
public:
  enum COST_TYPE
  {
    COST_SMOOTHNESS = 0,
    COST_COLLISION,
    COST_VALIDITY,
    COST_CONTACT_INVARIANT,
    COST_PHYSICS_VIOLATION,
    COST_GOAL_POSE,
    COST_COM,
    COST_ENDEFFECTOR_VELOCITY,
    COST_TORQUE,
    COST_RVO,
    COST_FTR,
    COST_CARTESIAN_TRAJECTORY,
    COST_SINGULARITY,
    COST_TYPES_NUM,
    COST_TYPE_INVALID = COST_TYPES_NUM,
  };

  TrajectoryCost(COST_TYPE type) :
      isHardConstraint_(false), type_(type)
  {
  }
  virtual ~TrajectoryCost()
  {
  }

  virtual void init(const EvaluationData* data);

  void compute(const EvaluationData* data, Eigen::VectorXd& costData, double& sum);
  virtual double getWaypointCost(int waypoint, const Eigen::VectorXd& costData) const
  {
    return costData(waypoint);
  }
  bool getIsHardConstraint() const
  {
    return isHardConstraint_;
  }
  COST_TYPE getType() const
  {
    return type_;
  }

  virtual double getWeight() const
  {
    return 0.0;
  }

  static boost::shared_ptr<TrajectoryCost> CreateTrajectoryCost(COST_TYPE type);

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData) = 0;
  void computeCostSum(const EvaluationData* data, Eigen::VectorXd& costData, double& sum);

  bool isHardConstraint_;
  COST_TYPE type_;

};

typedef boost::shared_ptr<TrajectoryCost> TrajectoryCostPtr;

class TrajectorySmoothnessCost: public TrajectoryCost
{
public:
  TrajectorySmoothnessCost() :
      TrajectoryCost(COST_SMOOTHNESS)
  {
  }
  virtual ~TrajectorySmoothnessCost()
  {
  }

  virtual double getWaypointCost(int waypoint, const Eigen::VectorXd& costData) const
  {
    return 0.0;
  }
  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

class TrajectoryCollisionCost: public TrajectoryCost
{
public:
  TrajectoryCollisionCost() :
      TrajectoryCost(COST_COLLISION)
  {
  }
  virtual ~TrajectoryCollisionCost()
  {
  }

  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

class TrajectoryValidityCost: public TrajectoryCost
{
public:
  TrajectoryValidityCost() :
      TrajectoryCost(COST_VALIDITY)
  {
  }
  virtual ~TrajectoryValidityCost()
  {
  }

  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

class TrajectoryContactInvariantCost: public TrajectoryCost
{
public:
  TrajectoryContactInvariantCost() :
      TrajectoryCost(COST_CONTACT_INVARIANT)
  {
  }
  virtual ~TrajectoryContactInvariantCost()
  {
  }

  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

class TrajectoryPhysicsViolationCost: public TrajectoryCost
{
public:
  TrajectoryPhysicsViolationCost() :
      TrajectoryCost(COST_PHYSICS_VIOLATION)
  {
  }
  virtual ~TrajectoryPhysicsViolationCost()
  {
  }

  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

class TrajectoryGoalPoseCost: public TrajectoryCost
{
public:
  TrajectoryGoalPoseCost() :
      TrajectoryCost(COST_GOAL_POSE)
  {
  }
  virtual ~TrajectoryGoalPoseCost()
  {
  }

  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

class TrajectoryCoMCost: public TrajectoryCost
{
public:
  TrajectoryCoMCost() :
      TrajectoryCost(COST_COM)
  {
  }
  virtual ~TrajectoryCoMCost()
  {
  }

  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

class TrajectoryFTRCost: public TrajectoryCost
{
public:
  TrajectoryFTRCost() :
      TrajectoryCost(COST_FTR)
  {
  }
  virtual ~TrajectoryFTRCost()
  {
  }

  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

class TrajectoryCartesianCost: public TrajectoryCost
{
public:
  TrajectoryCartesianCost() :
      TrajectoryCost(COST_CARTESIAN_TRAJECTORY)
  {
  }
  virtual ~TrajectoryCartesianCost()
  {
  }

  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

class TrajectorySingularityCost: public TrajectoryCost
{
public:
  TrajectorySingularityCost() :
      TrajectoryCost(COST_SINGULARITY)
  {
  }
  virtual ~TrajectorySingularityCost()
  {
  }

  virtual double getWeight() const;

protected:
  virtual void doCompute(const EvaluationData* data, Eigen::VectorXd& costData);
};

}
;

#endif /* TRAJECTORYCOST_H_ */
