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

#ifndef SMOOTHNESS_COST_H_
#define SMOOTHNESS_COST_H_

#include <itomp_ca_planner/common.h>
#include <itomp_ca_planner/trajectory/itomp_cio_trajectory.h>

namespace itomp_ca_planner
{

class SmoothnessCost
{
public:
  SmoothnessCost(const ItompCIOTrajectory& trajectory, int joint_number, const std::vector<double>& derivative_costs,
      double ridge_factor = 0.0);
  virtual ~SmoothnessCost();

  const Eigen::MatrixXd& getQuadraticCostInverse() const;
  const Eigen::MatrixXd& getQuadraticCost() const;

  double getCost(Eigen::MatrixXd::ColXpr joint_trajectory) const;
  double getCost(Eigen::MatrixXd::ConstColXpr joint_trajectory) const;

  double getMaxQuadCostInvValue() const;

  void scale(double scale);

private:
  Eigen::MatrixXd quad_cost_full_;
  Eigen::MatrixXd quad_cost_;
  Eigen::MatrixXd quad_cost_inv_;

  Eigen::MatrixXd getDiffMatrix(int size, const double* diff_rule) const;

};

inline const Eigen::MatrixXd& SmoothnessCost::getQuadraticCostInverse() const
{
  return quad_cost_inv_;
}

inline const Eigen::MatrixXd& SmoothnessCost::getQuadraticCost() const
{
  return quad_cost_;
}

inline double SmoothnessCost::getCost(Eigen::MatrixXd::ColXpr joint_trajectory) const
{
  return joint_trajectory.dot(quad_cost_full_ * joint_trajectory);
}

inline double SmoothnessCost::getCost(Eigen::MatrixXd::ConstColXpr joint_trajectory) const
{
  return joint_trajectory.dot(quad_cost_full_ * joint_trajectory);
}

}

#endif
