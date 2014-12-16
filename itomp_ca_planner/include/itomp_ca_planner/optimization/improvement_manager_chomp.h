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
#ifndef IMPROVEMENT_MANAGER_CHOMP_H_
#define IMPROVEMENT_MANAGER_CHOMP_H_

#include <itomp_ca_planner/optimization/improvement_manager.h>
#include <itomp_ca_planner/common.h>
#include <itomp_ca_planner/optimization/evaluation_manager.h>
#include <itomp_ca_planner/optimization/rollout.h>
#include <itomp_ca_planner/util/multivariate_gaussian.h>

namespace itomp_ca_planner
{

class ImprovementManagerChomp: public ImprovementManager
{
public:
  ImprovementManagerChomp();
  virtual ~ImprovementManagerChomp();

  virtual bool updatePlanningParameters();
  virtual void runSingleIteration(int iteration);

private:
  void initializeCosts();
  void initializeNoiseGenerators();
  void initializeRollouts();
  bool preAllocateTempVariables();
  bool generateRollouts(const std::vector<double>& noise_stddev, const std::vector<double>& contact_noise_stddev);
  void copyGroupTrajectory();
  bool setRolloutCosts();
  void computeUpdates();
  bool computeRolloutCumulativeCosts();
  bool computeRolloutProbabilities();
  bool computeParameterUpdates();
  bool updateParameters();
  void getParameters();
  void addExtraRollout(std::vector<Eigen::VectorXd>& parameters, std::vector<Eigen::VectorXd>& contact_parameters,
      Eigen::VectorXd& costs);
  bool computeNoise(Rollout& rollout);
  bool computeProjectedNoise(Rollout& rollout);
  void computeRolloutControlCost(Rollout& rollout);

  int last_planning_parameter_index_;

  int num_dimensions_;
  int num_contact_dimensions_;
  int num_time_steps_;
  int num_contact_time_steps_;
  double noise_decay_;
  std::vector<double> noise_stddev_;
  double noise_stddev_contacts_;

  int num_vars_free_;
  int num_vars_all_;
  int free_vars_start_index_;
  int free_vars_end_index_;
  int num_contact_vars_free_;
  int num_contact_vars_all_;
  int free_contact_vars_start_index_;
  int free_contact_vars_end_index_;

  std::vector<Eigen::VectorXd> parameters_all_;
  std::vector<Eigen::VectorXd> contact_parameters_all_;
  std::vector<Eigen::VectorXd> parameters_; /**< [num_dimensions] num_parameters */
  std::vector<Eigen::VectorXd> contact_parameters_;

  Eigen::MatrixXd rollout_costs_;
  Eigen::VectorXd tmp_rollout_cost_;

  bool use_cumulative_costs_;
  bool use_smooth_noises_;

  int num_rollouts_;
  int num_rollouts_reused_;
  int num_rollouts_extra_;
  int num_rollouts_gen_;
  bool rollouts_reused_; /**< Are we reusing rollouts for this iteration? */
  bool rollouts_reused_next_; /**< Can we reuse rollouts for the next iteration? */
  bool extra_rollouts_added_; /**< Have the "extra rollouts" been added for use in the next iteration? */
  std::vector<std::pair<double, int> > rollout_cost_sorter_; /**< vector used for sorting rollouts by their cost */

  std::vector<Rollout> rollouts_;
  std::vector<Rollout> reused_rollouts_;
  std::vector<Rollout> extra_rollouts_;

  std::vector<Eigen::MatrixXd> differentiation_matrices_;
  std::vector<Eigen::MatrixXd> control_costs_all_;
  std::vector<Eigen::MatrixXd> control_costs_; /**< [num_dimensions] num_parameters x num_parameters */
  std::vector<Eigen::MatrixXd> inv_control_costs_; /**< [num_dimensions] num_parameters x num_parameters */
  std::vector<Eigen::MatrixXd> projection_matrix_; /**< [num_dimensions] num_parameters x num_parameters */
  double control_cost_weight_;

  std::vector<MultivariateGaussian> noise_generators_; /**< objects that generate noise for each dimension */
  std::vector<MultivariateGaussian> contact_noise_generators_; /**< objects that generate noise for each dimension */

  // temporary variables pre-allocated for efficiency:
  std::vector<Eigen::VectorXd> tmp_noise_; /**< [num_dimensions] num_parameters */
  std::vector<Eigen::VectorXd> tmp_contact_noise_; /**< [num_dimensions] num_parameters */
  std::vector<Eigen::VectorXd> tmp_parameters_; /**< [num_dimensions] num_parameters */
  Eigen::VectorXd tmp_max_cost_; /**< num_time_steps */
  Eigen::VectorXd tmp_min_cost_; /**< num_time_steps */
  Eigen::VectorXd tmp_max_minus_min_cost_; /**< num_time_steps */
  Eigen::VectorXd tmp_sum_rollout_probabilities_; /**< num_time_steps */
  std::vector<Eigen::MatrixXd> parameter_updates_; /**< [num_dimensions] num_time_steps x num_parameters */
  std::vector<Eigen::VectorXd> contact_parameter_updates_; /**< [num_dimensions] num_time_steps x num_parameters */
  std::vector<Eigen::VectorXd> time_step_weights_; /**< [num_dimensions] num_time_steps: Weights computed for updates per time-step */

};

}

#endif
