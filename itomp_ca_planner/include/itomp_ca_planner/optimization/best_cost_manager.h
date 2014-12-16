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

#ifndef BEST_COST_MANAGER_H_
#define BEST_COST_MANAGER_H_

#include <itomp_ca_planner/common.h>

namespace itomp_ca_planner
{
class BestCostManager
{
public:
	BestCostManager();
	~BestCostManager();

	void reset();

	bool updateBestCost(int trajectory_index, double cost, bool feasible);
	int getBestCostTrajectoryIndex() const;

	bool isSolutionFound() const;

protected:
	boost::mutex mtx_;

	double best_cost;
	bool is_feasible_;

	unsigned int best_trajectory_index_;
};

inline BestCostManager::BestCostManager() :
		best_cost(std::numeric_limits<double>::max()), best_trajectory_index_(
				0), is_feasible_(false)
{

}

inline BestCostManager::~BestCostManager()
{

}

inline void BestCostManager::reset()
{
	best_cost = std::numeric_limits<double>::max();
	best_trajectory_index_ = 0;
	is_feasible_ = false;
}

inline bool BestCostManager::updateBestCost(int trajectory_index, double cost,
		bool feasible)
{
	if (!feasible && is_feasible_)
		return false;

	if (cost < best_cost || (feasible && !is_feasible_))
	{
		boost::lock_guard<boost::mutex> guard(mtx_);
		if (cost < best_cost || (feasible && !is_feasible_))
		{
			best_cost = cost;
			best_trajectory_index_ = trajectory_index;
			is_feasible_ = feasible;
		}
	}
	return trajectory_index == best_trajectory_index_;
}

inline int BestCostManager::getBestCostTrajectoryIndex() const
{
	return best_trajectory_index_;
}

inline bool BestCostManager::isSolutionFound() const
{
	return is_feasible_;
}

}

#endif /* BEST_COST_MANAGER_H_ */
