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

#ifndef VISUALIZATIONMANAGER_H_
#define VISUALIZATIONMANAGER_H_

#include <itomp_ca_planner/common.h>
#include <ros/publisher.h>
#include <kdl/frames.hpp>
#include <itomp_ca_planner/trajectory/itomp_cio_trajectory.h>
#include <itomp_ca_planner/util/singleton.h>

namespace itomp_ca_planner
{
class ItompRobotModel;

class VisualizationManager: public Singleton<VisualizationManager>
{
public:
	VisualizationManager();
	virtual ~VisualizationManager();

	void initialize(const ItompRobotModel& robot_model);
	void setPlanningGroup(const ItompRobotModel& robot_model,
			const std::string& groupName);

	void render();

	void animateEndeffector(int trajectory_index, int point_start,
			int point_end,
			const std::vector<std::vector<KDL::Frame> >& segmentFrames,
			bool best);

	void animateCoM(int numFreeVars, int freeVarStartIndex,
			const std::vector<KDL::Vector>& CoM, bool best);
	void animateRoot(int numFreeVars, int freeVarStartIndex,
			const std::vector<std::vector<KDL::Frame> >& segmentFrames,
			bool best);
	void animatePath(int trajectory_index, const ItompCIOTrajectory* traj,
			bool is_best, const std::string& group_name);
    void animateCollisionSpheres(int trajectory_index, const ItompCIOTrajectory* traj, bool is_best, const std::string& group_name);

	void publish(const visualization_msgs::Marker& msg);
	void publish(const visualization_msgs::MarkerArray& msg);

	void clearCollisionPointMarkPositions()
	{
		collision_point_mark_positions_.clear();
	}
	void insertCollisionPointMarkPosition(const Eigen::Vector3d& pos)
	{
		collision_point_mark_positions_.push_back(pos);
	}
	void renderEnvironment();
	void renderGround();
	void clearAnimations()
	{
	}

	ros::Publisher& getVisualizationMarkerPublisher()
	{
		return vis_marker_publisher_;
	}
	ros::Publisher& getVisualizationMarkerArrayPublisher()
	{
		return vis_marker_array_publisher_;
	}

private:
	boost::mutex mtx_;

	ros::Publisher vis_marker_array_publisher_;
	ros::Publisher vis_marker_publisher_;

	std::vector<int> animate_endeffector_segment_numbers_;
	int root_segment_number_;
	std::string reference_frame_;

	std::vector<Eigen::Vector3d> collision_point_mark_positions_;

	const itomp_ca_planner::ItompRobotModel* robot_model_;
	std::vector<robot_state::RobotStatePtr> robot_states_;
};

inline void VisualizationManager::publish(const visualization_msgs::Marker& msg)
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	vis_marker_publisher_.publish(msg);
}
inline void VisualizationManager::publish(const visualization_msgs::MarkerArray& msg)
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	vis_marker_array_publisher_.publish(msg);
}

}
;

#endif /* VISUALIZATIONMANAGER_H_ */
