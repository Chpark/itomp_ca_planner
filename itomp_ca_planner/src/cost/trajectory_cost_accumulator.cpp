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

#include <itomp_ca_planner/cost/trajectory_cost_accumulator.h>
#include <itomp_ca_planner/optimization/evaluation_data.h>
#include <ros/console.h>

namespace itomp_ca_planner
{

TrajectoryCostAccumulator::TrajectoryCostAccumulator() :
    is_last_trajectory_valid_(true)
{
	best_cost_ = std::numeric_limits<double>::max();
}

TrajectoryCostAccumulator::~TrajectoryCostAccumulator()
{

}

void TrajectoryCostAccumulator::addCost(TrajectoryCostPtr cost)
{
	if (cost != NULL)
	{
		costMap_[cost->getType()] = cost;
	}
}

void TrajectoryCostAccumulator::init(const EvaluationData* data)
{
	for (std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr>::iterator it =
                costMap_.begin(); it != costMap_.end(); ++it)
	{
		it->second->init(data);
		costDataMap_[it->first] = Eigen::VectorXd::Zero(data->getNumPoints());
		costSumMap_[it->first] = 0.0;
	}
}

void TrajectoryCostAccumulator::compute(const EvaluationData* data)
{
	for (std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr>::iterator it =
                costMap_.begin(); it != costMap_.end(); ++it)
	{
		if (it->second->getWeight() != 0.0)
			it->second->compute(data, costDataMap_[it->first],
                                costSumMap_[it->first]);
	}
}

double TrajectoryCostAccumulator::getWaypointCost(int waypoint) const
{
	double accumulatedCost = 0.0;
	for (std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr>::const_iterator it =
                costMap_.begin(); it != costMap_.end(); ++it)
	{
		const Eigen::VectorXd& costData = costDataMap_.find(it->first)->second;
		accumulatedCost += it->second->getWaypointCost(waypoint, costData)
                           * it->second->getWeight();
	}
	return accumulatedCost;
}

double TrajectoryCostAccumulator::getWaypointCost(int waypoint,
		TrajectoryCost::COST_TYPE type) const
{
	std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr>::const_iterator it =
        costMap_.find(type);
	const Eigen::VectorXd& costData = costDataMap_.find(type)->second;
	return it->second->getWaypointCost(waypoint, costData)
           * it->second->getWeight();
}

double TrajectoryCostAccumulator::getTrajectoryCost(
    TrajectoryCost::COST_TYPE type) const
{
	std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr>::const_iterator it =
        costMap_.find(type);
	if (it != costMap_.end())
	{
		double costSum = costSumMap_.find(it->first)->second;
		return costSum * it->second->getWeight();
	}
	return 0.0;
}

double TrajectoryCostAccumulator::getTrajectoryCost() const
{
	double accumulatedCost = 0.0;
	for (std::map<TrajectoryCost::COST_TYPE, TrajectoryCostPtr>::const_iterator it =
                costMap_.begin(); it != costMap_.end(); ++it)
	{
		double costSum = costSumMap_.find(it->first)->second;
		accumulatedCost += costSum * it->second->getWeight();
	}
	return accumulatedCost;
}

void TrajectoryCostAccumulator::print(int trajectory_index, int number) const
{
	double cost = getTrajectoryCost();
	if (cost < best_cost_)
	{
		best_cost_ = cost;

        printf("[%d:%d] Trajectory cost : %f/%f (s=%f, c=%f, ca=%f, pc=%f)\n",
               trajectory_index,
               number, cost, best_cost_,
               getTrajectoryCost(TrajectoryCost::COST_SMOOTHNESS),
               getTrajectoryCost(TrajectoryCost::COST_COLLISION),
               getTrajectoryCost(TrajectoryCost::COST_CARTESIAN_TRAJECTORY),
               getTrajectoryCost(TrajectoryCost::COST_POINT_CLOUD));
    }
    else
    {

    }
}

bool TrajectoryCostAccumulator::isFeasible() const
{
	//if (getTrajectoryCost(TrajectoryCost::COST_CONTACT_INVARIANT) > 1E-7)
	//return false;

	//if (getTrajectoryCost() - getTrajectoryCost(TrajectoryCost::COST_SMOOTHNESS) < 0.01)
	//return true;

	if (!is_last_trajectory_valid_)
		return false;

	if (getTrajectoryCost(TrajectoryCost::COST_COLLISION) < 1E-7)
		return true;

	return false;
}

}
