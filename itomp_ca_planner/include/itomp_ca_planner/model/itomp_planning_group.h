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

#ifndef ITOMP_PLANNING_GROUP_H_
#define ITOMP_PLANNING_GROUP_H_

#include <itomp_ca_planner/common.h>
#include <kdl/tree.hpp>
#include <itomp_ca_planner/model/itomp_robot_joint.h>
#include <itomp_ca_planner/model/treefksolverjointposaxis_partial.hpp>
#include <itomp_ca_planner/contact/contact_point.h>

namespace itomp_ca_planner
{

class ItompPlanningGroup
{
public:
	std::string name_; /**< Name of the planning group */
	int num_joints_; /**< Number of joints used in planning */
	std::vector<ItompRobotJoint> group_joints_; /**< Joints used in planning */
	std::vector<std::string> link_names_; /**< Links used in planning */
	std::vector<std::string> collision_link_names_; /**< Links used in collision checking */
	boost::shared_ptr<KDL::TreeFkSolverJointPosAxisPartial> fk_solver_; /**< Forward kinematics solver for the group */
	std::vector<ContactPoint> contactPoints_;
	std::map<int, int> kdl_to_group_joint_;

	std::vector<std::string> getJointNames() const;
	int getNumContacts() const;
};

////////////////////////////////////////////////////////////////////////////////

inline int ItompPlanningGroup::getNumContacts() const
{
	return contactPoints_.size();
}

}
#endif
