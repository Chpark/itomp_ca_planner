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
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/profiler/profiler.h>
#include <class_loader/class_loader.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <itomp_ca_planner/itomp_planning_interface.h>

namespace itomp_ca_planner
{

class ITOMPPlannerManager: public planning_interface::PlannerManager
{
public:

	ITOMPPlannerManager() :
			planning_interface::PlannerManager()
	{
	}

	virtual bool initialize(const robot_model::RobotModelConstPtr& model, const std::string &ns)
	{
		context_.reset(new ItompPlanningContext("ITOMP", "right_arm"));
		return context_->initialize(model);
	}

	virtual bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
	{
		return true;
	}

	virtual std::string getDescription() const
	{
		return "ITOMP";
	}

	virtual void getPlanningAlgorithms(std::vector<std::string> &algs) const
	{
		algs.push_back("ITOMP");
        algs.push_back("ITOMP_3steps");
	}

	virtual void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pconfig)
	{
		// this call can add a few more configs than we pass in (adds defaults)
		//ompl_interface_->setPlannerConfigurations(pconfig);
		// so we read the configs instead of just setting pconfig
		//PlannerManager::setPlannerConfigurations(ompl_interface_->getPlannerConfigurations());
		PlannerManager::setPlannerConfigurations(pconfig);
	}

	virtual planning_interface::PlanningContextPtr getPlanningContext(
			const planning_scene::PlanningSceneConstPtr& planning_scene,
			const planning_interface::MotionPlanRequest &req, moveit_msgs::MoveItErrorCodes &error_code) const
	{
		//return ompl_interface_->getPlanningContext(planning_scene, req, error_code);
		context_->setPlanningScene(planning_scene);
		context_->setPlanRequest(req);

		return context_;
	}

private:
	boost::shared_ptr<ItompPlanningContext> context_;
};

}

CLASS_LOADER_REGISTER_CLASS(itomp_ca_planner::ITOMPPlannerManager, planning_interface::PlannerManager);
