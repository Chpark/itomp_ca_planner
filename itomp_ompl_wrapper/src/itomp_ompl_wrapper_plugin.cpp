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
#include <pluginlib/class_loader.h>
#include <class_loader/class_loader.h>

namespace itomp_ompl_wrapper
{

class ItompOmplPlannerManager: public planning_interface::PlannerManager
{
public:

    ItompOmplPlannerManager() : planning_interface::PlannerManager()
	{
        loadPlugin("itomp_ca_planner/ItompPlanner", itomp_planner_instance_, itomp_planner_plugin_loader_);
        loadPlugin("ompl_interface/OMPLPlanner", ompl_planner_instance_, ompl_planner_plugin_loader_);
	}

	virtual bool initialize(const robot_model::RobotModelConstPtr& model, const std::string &ns)
	{
        return itomp_planner_instance_->initialize(model, ns) &&
                ompl_planner_instance_->initialize(model, ns);
	}

	virtual bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
	{
        return itomp_planner_instance_->canServiceRequest(req) ||
                ompl_planner_instance_->canServiceRequest(req);
	}

	virtual std::string getDescription() const
	{
        return "Merged wrapper for ITOMP and OMPL Planners";
	}

	virtual void getPlanningAlgorithms(std::vector<std::string> &algs) const
	{
        ompl_planner_instance_->getPlanningAlgorithms(algs);
        itomp_planner_instance_->getPlanningAlgorithms(algs);
	}

	virtual void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pconfig)
	{
        itomp_planner_instance_->setPlannerConfigurations(pconfig);
        ompl_planner_instance_->setPlannerConfigurations(pconfig);
	}

	virtual planning_interface::PlanningContextPtr getPlanningContext(
        const planning_scene::PlanningSceneConstPtr& planning_scene,
        const planning_interface::MotionPlanRequest &req, moveit_msgs::MoveItErrorCodes &error_code) const
	{
        if (req.planner_id.find("ITOMP") != std::string::npos)
            return itomp_planner_instance_->getPlanningContext(planning_scene, req, error_code);
        else
            return ompl_planner_instance_->getPlanningContext(planning_scene, req, error_code);
	}

private:
    void loadPlugin(const std::string& planner_plugin_name, planning_interface::PlannerManagerPtr& planner_instance,
                    boost::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> >& planner_plugin_loader)
    {
        try
        {
            planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
        }
        catch (pluginlib::PluginlibException& ex)
        {
            ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
        }

        try
        {
            planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
        }
        catch (pluginlib::PluginlibException& ex)
        {
            ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
        }
        try
        {
            planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        }
        catch (pluginlib::PluginlibException& ex)
        {
            const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
            std::stringstream ss;
            for (std::size_t i = 0 ; i < classes.size() ; ++i)
                ss << classes[i] << " ";
            ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                             << "Available plugins: " << ss.str());
        }
    }

    boost::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > itomp_planner_plugin_loader_;
    boost::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > ompl_planner_plugin_loader_;

    planning_interface::PlannerManagerPtr itomp_planner_instance_;
    planning_interface::PlannerManagerPtr ompl_planner_instance_;
};

}

CLASS_LOADER_REGISTER_CLASS(itomp_ompl_wrapper::ItompOmplPlannerManager, planning_interface::PlannerManager)
