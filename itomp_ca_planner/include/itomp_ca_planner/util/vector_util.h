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
#ifndef VECTOR_UTIL_H_
#define VECTOR_UTIL_H_

#include <itomp_ca_planner/common.h>
#include <itomp_ca_planner/util/differentiation_rules.h>

namespace itomp_ca_planner
{

template<typename KDLType, typename EigenType>
void kdlVecToEigenVec(std::vector<KDLType>& kdl_v, std::vector<Eigen::Map<EigenType> >& eigen_v, int rows, int cols)
{
	int size = kdl_v.size();
	eigen_v.clear();
	for (int i = 0; i < size; i++)
	{
		eigen_v.push_back(Eigen::Map<EigenType>(kdl_v[i].data, rows, cols));
	}
}

template<typename KDLType, typename EigenType>
void kdlVecVecToEigenVecVec(std::vector<std::vector<KDLType> >& kdl_vv,
		std::vector<std::vector<Eigen::Map<EigenType> > > & eigen_vv, int rows, int cols)
{
	int size = kdl_vv.size();
	eigen_vv.resize(size);
	for (int i = 0; i < size; i++)
	{
		kdlVecToEigenVec(kdl_vv[i], eigen_vv[i], rows, cols);
	}
}

template<typename VecType>
void getVectorVelocities(int start, int end, double discretization, const std::vector<VecType>& pos, std::vector<
		VecType>& vel, const VecType& zeroVec)
{
	const double invTime = 1.0 / discretization;

	for (int point = start; point <= end; ++point)
	{
		vel[point] = zeroVec;

		for (int k = -DIFF_RULE_LENGTH / 2; k <= DIFF_RULE_LENGTH / 2; k++)
		{
			vel[point] += (invTime * DIFF_RULES[DIFF_RULE_VELOCITY][k + DIFF_RULE_LENGTH / 2]) * pos[point + k];
		}
	}
}

template<typename VecType>
void getVectorVelocitiesAndAccelerations(int start, int end, double discretization, const std::vector<VecType>& pos,
		std::vector<VecType>& vel, std::vector<VecType>& acc, const VecType& zeroVec)
{
	const double invTime = 1.0 / discretization;

	for (int point = start; point <= end; ++point)
	{
		acc[point] = vel[point] = zeroVec;

		for (int k = -DIFF_RULE_LENGTH / 2; k <= DIFF_RULE_LENGTH / 2; k++)
		{
			vel[point] += (invTime * DIFF_RULES[DIFF_RULE_VELOCITY][k + DIFF_RULE_LENGTH / 2]) * pos[point + k];
			acc[point] += (invTime * invTime * DIFF_RULES[DIFF_RULE_ACCELERATION][k + DIFF_RULE_LENGTH / 2]) * pos[point + k];
		}
	}
}

class Vector4d
{
public:
  Vector4d() {}
  Vector4d(double x, double y, double z, double w)
  {
    data_[0] = x;
    data_[1] = y;
    data_[2] = z;
    data_[3] = w;
  }
  double data_[4];
};

}
#endif
