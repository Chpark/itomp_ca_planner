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

#include <itomp_ca_planner/visualization/visualization_manager.h>
#include <itomp_ca_planner/model/itomp_robot_model.h>
#include <itomp_ca_planner/util/planning_parameters.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>

using namespace std;

namespace itomp_ca_planner
{

std::map<int, std_msgs::ColorRGBA> COLOR_MAP;
enum COLORS
{
    BLACK = 0,
    RED,
    GREEN,
    YELLOW,
    BLUE,
    MAGENTA,
    CYAN,
    WHITE,
    NUM_COLORS,
};

VisualizationManager::VisualizationManager()
{
    COLOR_MAP.clear();
    for (int i = 0; i < NUM_COLORS; ++i)
    {
        std_msgs::ColorRGBA color;
        color.a = 1.0;
        color.r = i % 2;
        color.g = (i >> 1) % 2;
        color.b = (i >> 2) % 2;
        COLOR_MAP[i] = color;
    }
}

VisualizationManager::~VisualizationManager()
{
}

void VisualizationManager::render()
{
	//renderGround();
    //renderEnvironment();

}

void VisualizationManager::renderEnvironment()
{
	string environment_file =
        PlanningParameters::getInstance()->getEnvironmentModel();
	if (environment_file.empty())
		return;

	vector<double> environment_position =
        PlanningParameters::getInstance()->getEnvironmentModelPosition();
	double scale =
        PlanningParameters::getInstance()->getEnvironmentModelScale();
	environment_position.resize(3, 0);

	visualization_msgs::MarkerArray ma;
	visualization_msgs::Marker msg;
	msg.header.frame_id = reference_frame_;
	msg.header.stamp = ros::Time::now();
	msg.ns = "environment";
	msg.type = visualization_msgs::Marker::MESH_RESOURCE;
	msg.action = visualization_msgs::Marker::ADD;
	msg.scale.x = scale;
	msg.scale.y = scale;
	msg.scale.z = scale;
	msg.id = 0;
	msg.pose.position.x = environment_position[0];
	msg.pose.position.y = environment_position[1];
	msg.pose.position.z = environment_position[2];
	/*
	 msg.pose.orientation.x = sqrt(0.5);
	 msg.pose.orientation.y = 0.0;
	 msg.pose.orientation.z = 0.0;
	 msg.pose.orientation.w = sqrt(0.5);
	 */
	msg.pose.orientation.x = 0.0;
	msg.pose.orientation.y = 0.0;
	msg.pose.orientation.z = 0.0;
	msg.pose.orientation.w = 1.0;
	msg.color.a = 1.0;
	msg.color.r = 0.5;
	msg.color.g = 0.5;
	msg.color.b = 0.5;
    msg.mesh_resource = environment_file;
	ma.markers.push_back(msg);
	vis_marker_array_publisher_.publish(ma);
}

void VisualizationManager::renderGround()
{
	//return;

	if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 1.0)
	{
		// hrp4
		// stair

		visualization_msgs::MarkerArray ma;
		visualization_msgs::Marker::_color_type FLOOR_COLOR;
		visualization_msgs::Marker::_color_type STAIR_COLOR;
		visualization_msgs::Marker::_color_type STAIR_COLOR2;

		FLOOR_COLOR.a = 1.0;
		FLOOR_COLOR.r = 1.0;
		FLOOR_COLOR.g = 1.0;
		FLOOR_COLOR.b = 1.0;

		visualization_msgs::Marker msg;
		msg.header.frame_id = reference_frame_;
		msg.header.stamp = ros::Time::now();
		msg.ns = "floor";
		msg.type = visualization_msgs::Marker::CUBE;
		msg.action = visualization_msgs::Marker::ADD;

		msg.scale.x = 80.0;
		msg.scale.y = 80.0;
		msg.scale.z = 0.1;

		msg.points.resize(0);

		msg.color = FLOOR_COLOR;
		msg.id = 30000;

		msg.pose.position.z = -0.1;
		ma.markers.push_back(msg);

		msg.points.resize(0);
		STAIR_COLOR.a = 1.0;
		STAIR_COLOR.r = 0.5;
		STAIR_COLOR.g = 0.5;
		STAIR_COLOR.b = 0.8;
		STAIR_COLOR2.a = 1.0;
		STAIR_COLOR2.r = 0.5;
		STAIR_COLOR2.g = 0.8;
		STAIR_COLOR2.b = 0.5;

		msg.color = STAIR_COLOR;
		++msg.id;
		msg.ns = "floor1";
		msg.scale.x = 0.4;
		msg.scale.y = 40.0;
		msg.scale.z = 0.15;
		msg.pose.position.x = -0.8;
		msg.pose.position.z = 0.075;
		ma.markers.push_back(msg);

		msg.color = STAIR_COLOR2;
		++msg.id;
		msg.pose.position.x += 0.4;
		msg.pose.position.z += 0.075;
		msg.scale.z += 0.15;
		ma.markers.push_back(msg);

		msg.color = STAIR_COLOR;
		++msg.id;
		msg.pose.position.x += 0.4;
		msg.pose.position.z += 0.075;
		msg.scale.z += 0.15;
		ma.markers.push_back(msg);

		msg.color = STAIR_COLOR2;
		++msg.id;
		msg.pose.position.x += 0.4;
		msg.pose.position.z -= 0.075;
		msg.scale.z -= 0.15;
		ma.markers.push_back(msg);

		msg.color = STAIR_COLOR;
		++msg.id;
		msg.pose.position.x += 0.4;
		msg.pose.position.z -= 0.075;
		msg.scale.z -= 0.15;
		ma.markers.push_back(msg);

		vis_marker_array_publisher_.publish(ma);
	}
	/*
	 {
	 // human
	 // stair

	 visualization_msgs::MarkerArray ma;
	 visualization_msgs::Marker::_color_type FLOOR_COLOR;
	 visualization_msgs::Marker::_color_type STAIR_COLOR;
	 visualization_msgs::Marker::_color_type STAIR_COLOR2;

	 FLOOR_COLOR.a = 1.0;
	 FLOOR_COLOR.r = 0.5;
	 FLOOR_COLOR.g = 0.5;
	 FLOOR_COLOR.b = 0.5;

	 visualization_msgs::Marker msg;
	 msg.header.frame_id = reference_frame_;
	 msg.header.stamp = ros::Time::now();
	 msg.ns = "floor";
	 msg.type = visualization_msgs::Marker::CUBE;
	 msg.action = visualization_msgs::Marker::ADD;

	 msg.scale.x = 2.0;
	 msg.scale.y = 10.0;
	 msg.scale.z = 0.1;

	 msg.points.resize(0);

	 msg.color = FLOOR_COLOR;
	 msg.id = 30000;

	 msg.pose.position.z = -0.1;
	 ma.markers.push_back(msg);

	 msg.points.resize(0);
	 STAIR_COLOR.a = 1.0;
	 STAIR_COLOR.r = 0.5;
	 STAIR_COLOR.g = 0.5;
	 STAIR_COLOR.b = 0.8;
	 STAIR_COLOR2.a = 1.0;
	 STAIR_COLOR2.r = 0.5;
	 STAIR_COLOR2.g = 0.8;
	 STAIR_COLOR2.b = 0.5;

	 msg.color = STAIR_COLOR;
	 ++msg.id;
	 msg.ns = "floor1";
	 msg.scale.x = 2.0;
	 msg.scale.y = 0.5;
	 msg.scale.z = 0.2;
	 msg.pose.position.y = -1.0;
	 msg.pose.position.z = 0.1;
	 ma.markers.push_back(msg);

	 msg.color = STAIR_COLOR2;
	 ++msg.id;
	 msg.pose.position.y += 0.5;
	 msg.pose.position.z += 0.1;
	 msg.scale.z += 0.2;
	 ma.markers.push_back(msg);

	 msg.color = STAIR_COLOR;
	 ++msg.id;
	 msg.pose.position.y += 0.5;
	 msg.pose.position.z += 0.1;
	 msg.scale.z += 0.2;
	 ma.markers.push_back(msg);

	 msg.color = STAIR_COLOR2;
	 ++msg.id;
	 msg.pose.position.y += 0.5;
	 msg.pose.position.z -= 0.1;
	 msg.scale.z -= 0.2;
	 ma.markers.push_back(msg);

	 msg.color = STAIR_COLOR;
	 ++msg.id;
	 msg.pose.position.y += 0.5;
	 msg.pose.position.z -= 0.1;
	 msg.scale.z -= 0.2;
	 ma.markers.push_back(msg);

	 vis_marker_array_pub_->publish(ma);
	 }
	 */

	if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 11.0)
	{
		// WAFR circle

		visualization_msgs::MarkerArray ma;
		visualization_msgs::Marker::_color_type FLOOR_COLOR;
		visualization_msgs::Marker::_color_type STAIR_COLOR;
		visualization_msgs::Marker::_color_type STAIR_COLOR2;

		FLOOR_COLOR.a = 1.0;
		FLOOR_COLOR.r = 1.0;
		FLOOR_COLOR.g = 1.0;
		FLOOR_COLOR.b = 1.0;

		visualization_msgs::Marker msg;
		msg.header.frame_id = reference_frame_;
		msg.header.stamp = ros::Time::now();
		msg.ns = "floor";
		msg.type = visualization_msgs::Marker::CUBE;
		msg.action = visualization_msgs::Marker::ADD;

		msg.scale.x = 80.0;
		msg.scale.y = 80.0;
		msg.scale.z = 0.1;

		msg.points.resize(0);

		msg.color = FLOOR_COLOR;
		msg.id = 30000;

		msg.pose.position.z = -0.1;
		ma.markers.push_back(msg);

		msg.points.resize(0);
		STAIR_COLOR.a = 1.0;
		STAIR_COLOR.r = 0.5;
		STAIR_COLOR.g = 0.5;
		STAIR_COLOR.b = 0.8;
		STAIR_COLOR2.a = 1.0;
		STAIR_COLOR2.r = 0.5;
		STAIR_COLOR2.g = 0.8;
		STAIR_COLOR2.b = 0.5;

		msg.scale.x = 80.0;

		msg.color = STAIR_COLOR;
		++msg.id;
		msg.ns = "floor1";
		msg.scale.y = 11.5;
		//		msg.scale.x = 5.0;
		msg.scale.z = 0.15;
		msg.pose.position.z = 0.075;
		ma.markers.push_back(msg);

		msg.color = STAIR_COLOR2;
		++msg.id;
		msg.pose.position.z += 0.075;
		msg.scale.y -= 4.0;
		msg.scale.z += 0.15;
		msg.scale.x -= 1.0;
		//msg.scale.x = 30.0;

		// hack
		msg.scale.y += 0.2;
		msg.pose.position.y -= 0.1;
		ma.markers.push_back(msg);
		msg.scale.y -= 0.2;
		msg.pose.position.y += 0.1;

		msg.color = STAIR_COLOR;
		++msg.id;
		msg.pose.position.z += 0.075;
		msg.scale.y -= 4.0;
		msg.scale.z += 0.15;
		msg.scale.x -= 1.0;
		//msg.scale.x = 20.0;
		ma.markers.push_back(msg);

		vis_marker_array_publisher_.publish(ma);
	}
	if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 12.0)
	{
		// WAFR dynamic obstacles
		visualization_msgs::MarkerArray ma;
		visualization_msgs::Marker::_color_type ROAD_COLOR;
		visualization_msgs::Marker::_color_type SIDEWALK_COLOR;
		visualization_msgs::Marker::_color_type YELLOW_COLOR;
		visualization_msgs::Marker::_color_type WHITE_COLOR;

		WHITE_COLOR.a = 1.0;
		WHITE_COLOR.r = 1.0;
		WHITE_COLOR.g = 1.0;
		WHITE_COLOR.b = 1.0;

		ROAD_COLOR.a = 1.0;
		ROAD_COLOR.r = 0.1;
		ROAD_COLOR.g = 0.1;
		ROAD_COLOR.b = 0.1;

		SIDEWALK_COLOR.a = 1.0;
		SIDEWALK_COLOR.r = 0.8;
		SIDEWALK_COLOR.g = 0.6;
		SIDEWALK_COLOR.b = 0.6;

		YELLOW_COLOR.a = 1.0;
		YELLOW_COLOR.r = 1.0;
		YELLOW_COLOR.g = 1.0;
		YELLOW_COLOR.b = 0.0;

		visualization_msgs::Marker msg;
		msg.header.frame_id = reference_frame_;
		msg.header.stamp = ros::Time::now();
		msg.ns = "road";
		msg.type = visualization_msgs::Marker::CUBE;
		msg.action = visualization_msgs::Marker::ADD;

		msg.scale.x = 80.0;
		msg.scale.y = 8.0;
		msg.scale.z = 0.1;
		msg.color = ROAD_COLOR;
		msg.id = 0;
		msg.pose.position.z = -0.1;
		ma.markers.push_back(msg);

		msg.scale.x = 2.0;
		msg.scale.y = 80.0;
		msg.scale.z = 0.1;
		msg.pose.position.x = 3.0;
		msg.color = ROAD_COLOR;
		msg.id = 1;
		msg.pose.position.z = -0.1;
		ma.markers.push_back(msg);

		msg.scale.x = 42.0;
		msg.scale.y = 0.05;
		msg.scale.z = 0.1;
		msg.color = YELLOW_COLOR;
		msg.id = 2;
		msg.pose.position.z = -0.09;
		msg.pose.position.x = -19.0;
		ma.markers.push_back(msg);

		msg.scale.x = 36.0;
		msg.scale.y = 0.05;
		msg.scale.z = 0.1;
		msg.color = YELLOW_COLOR;
		msg.id = 3;
		msg.pose.position.z = -0.09;
		msg.pose.position.x = 22.0;
		ma.markers.push_back(msg);

		msg.scale.x = 42.0;
		msg.scale.y = 0.05;
		msg.scale.z = 0.1;
		msg.color = WHITE_COLOR;
		msg.id = 4;
		msg.pose.position.z = -0.09;
		msg.pose.position.x = -19.0;
		msg.pose.position.y = -2.0;
		ma.markers.push_back(msg);

		msg.scale.x = 36.0;
		msg.scale.y = 0.05;
		msg.scale.z = 0.1;
		msg.color = WHITE_COLOR;
		msg.id = 5;
		msg.pose.position.z = -0.09;
		msg.pose.position.x = 22.0;
		msg.pose.position.y = -2.0;
		ma.markers.push_back(msg);

		msg.scale.x = 42.0;
		msg.scale.y = 0.05;
		msg.scale.z = 0.1;
		msg.color = WHITE_COLOR;
		msg.id = 6;
		msg.pose.position.z = -0.09;
		msg.pose.position.x = -19.0;
		msg.pose.position.y = 2.0;
		ma.markers.push_back(msg);

		msg.scale.x = 36.0;
		msg.scale.y = 0.05;
		msg.scale.z = 0.1;
		msg.color = WHITE_COLOR;
		msg.id = 7;
		msg.pose.position.z = -0.09;
		msg.pose.position.x = 22.0;
		msg.pose.position.y = 2.0;
		ma.markers.push_back(msg);

		msg.scale.x = 42.0;
		msg.scale.y = 36.0;
		msg.scale.z = 0.15;
		msg.color = SIDEWALK_COLOR;
		msg.id = 8;
		msg.pose.position.x = -19.0;
		msg.pose.position.y = -22.0;
		msg.pose.position.z = 0.075;
		ma.markers.push_back(msg);

		msg.scale.x = 42.0;
		msg.scale.y = 36.0;
		msg.scale.z = 0.15;
		msg.color = SIDEWALK_COLOR;
		msg.id = 9;
		msg.pose.position.x = -19.0;
		msg.pose.position.y = 22.0;
		msg.pose.position.z = 0.075;
		ma.markers.push_back(msg);

		msg.scale.x = 36.0;
		msg.scale.y = 36.0;
		msg.scale.z = 0.15;
		msg.color = SIDEWALK_COLOR;
		msg.id = 10;
		msg.pose.position.x = 22.0;
		msg.pose.position.y = -22.0;
		msg.pose.position.z = 0.075;
		ma.markers.push_back(msg);

		msg.scale.x = 36.0;
		msg.scale.y = 36.0;
		msg.scale.z = 0.15;
		msg.color = SIDEWALK_COLOR;
		msg.id = 11;
		msg.pose.position.x = 22.0;
		msg.pose.position.y = 22.0;
		msg.pose.position.z = 0.075;
		ma.markers.push_back(msg);

		vis_marker_array_publisher_.publish(ma);
	}
	if (PlanningParameters::getInstance()->getTemporaryVariable(2) == 13.0)
	{
		// wafr narrow passage
		visualization_msgs::MarkerArray ma;

		visualization_msgs::Marker msg;
		msg.header.frame_id = reference_frame_;
		msg.header.stamp = ros::Time::now();
		msg.ns = "road";
		msg.type = visualization_msgs::Marker::CUBE;
		msg.action = visualization_msgs::Marker::ADD;
		visualization_msgs::Marker::_color_type WALL_COLOR;
		WALL_COLOR.a = 1.0;
		WALL_COLOR.r = 0.5;
		WALL_COLOR.g = 0.5;
		WALL_COLOR.b = 0.5;

		msg.scale.x = 0.2;
		msg.scale.y = 2.0;
		msg.scale.z = 1.2;
		msg.color = WALL_COLOR;
		msg.id = 0;
		msg.pose.position.x = -0.5;
		msg.pose.position.y = 0.0;
		msg.pose.position.z = 0.6;
		ma.markers.push_back(msg);

		msg.id = 1;
		msg.pose.position.x = 0.5;
		ma.markers.push_back(msg);

		msg.id = 2;
		msg.scale.x = 10.0 * sqrt(2.0);
		msg.scale.y = 0.2;
		msg.pose.position.x = -0.4 - 5.0 - 0.1 / sqrt(2.0);
		msg.pose.position.y = -1.0 - 5.0 + 0.1 / sqrt(2.0);
		msg.pose.orientation.x = 0.0;
		msg.pose.orientation.y = 0.0;
		msg.pose.orientation.z = sin(M_PI_4 * 0.5);
		msg.pose.orientation.w = cos(M_PI_4 * 0.5);
		ma.markers.push_back(msg);

		msg.id = 3;
		msg.scale.x = 10.0 * sqrt(2.0);
		msg.scale.y = 0.2;
		msg.pose.position.x = 0.4 + 5.0 + 0.1 / sqrt(2.0);
		msg.pose.position.y = -1.0 - 5.0 + 0.1 / sqrt(2.0);
		msg.pose.orientation.x = 0.0;
		msg.pose.orientation.y = 0.0;
		msg.pose.orientation.z = sin(-M_PI_4 * 0.5);
		msg.pose.orientation.w = cos(-M_PI_4 * 0.5);
		ma.markers.push_back(msg);

		vis_marker_array_publisher_.publish(ma);
	}

}

void VisualizationManager::initialize(
    const itomp_ca_planner::ItompRobotModel& robot_model)
{
	ros::NodeHandle node_handle;
	vis_marker_array_publisher_ = node_handle.advertise<
                                  visualization_msgs::MarkerArray>(
                                      "pomp_planner/visualization_marker_array", 10);
	vis_marker_publisher_ = node_handle.advertise<visualization_msgs::Marker>(
                                "pomp_planner/visualization_marker", 10);

	reference_frame_ = robot_model.getReferenceFrame();

	robot_model_ = &robot_model;

	int num_traj = PlanningParameters::getInstance()->getNumTrajectories();
	robot_states_.resize(num_traj);
	for (int i = 0; i < num_traj; ++i)
		robot_states_[i].reset(
            new robot_state::RobotState(robot_model_->getRobotModel()));
}

void VisualizationManager::setPlanningGroup(
    const itomp_ca_planner::ItompRobotModel& robot_model,
    const std::string& groupName)
{
    animate_endeffector_segment_numbers_.clear();

	const multimap<string, string>& endeffectorSegments =
        PlanningParameters::getInstance()->getAnimateEndeffectorSegment();

	multimap<string, string>::const_iterator it;
	for (it = endeffectorSegments.begin(); it != endeffectorSegments.end();
			++it)
	{
		if (it->first == groupName)
		{
			int segmentIndex =
                robot_model.getForwardKinematicsSolver()->segmentNameToIndex(
                    it->second);
			if (segmentIndex == -1)
			{
				ROS_INFO(
                    "Invalid endeffector segment name %s for %s", it->second.c_str(), it->first.c_str());
			}
			else
			{
				animate_endeffector_segment_numbers_.push_back(segmentIndex);
			}
		}
	}

	root_segment_number_ =
        robot_model.getForwardKinematicsSolver()->segmentNameToIndex(
            PlanningParameters::getInstance()->getLowerBodyRoot());
}

void VisualizationManager::animateEndeffector(int trajectory_index, int point_start, int point_end,
		const vector<vector<KDL::Frame> >& segmentFrames, bool best)
{
    const double scale = 0.05;
    const double scale2 = 0.025;
    const int marker_step = 1;

    visualization_msgs::MarkerArray ma;

    visualization_msgs::Marker::_color_type ORANGE;
    ORANGE.a = 1.0;
    ORANGE.r = 1.0;
    ORANGE.g = 0.6;
    ORANGE.b = 0.0;

    visualization_msgs::Marker msg, msg2;
	msg.header.frame_id = reference_frame_;
	msg.header.stamp = ros::Time::now();
    msg.ns = "itomp_endeffector";
	msg.type = visualization_msgs::Marker::SPHERE_LIST;
	msg.action = visualization_msgs::Marker::ADD;

	msg.scale.x = scale;
	msg.scale.y = scale;
	msg.scale.z = scale;

	msg.points.resize(0);

    msg2 = msg;
    msg2.ns = "itomp_endeffector_path";
    msg2.type = visualization_msgs::Marker::LINE_STRIP;
    msg2.scale.x = msg2.scale.y = msg2.scale.z = scale2;

    //msg2.color = msg.color = ORANGE;
    msg2.color = msg.color = COLOR_MAP[trajectory_index];

    visualization_msgs::Marker msg_best, msg2_best;
    visualization_msgs::Marker msg_frame[3], msg_tangent;
    if (best)
    {
        msg_best = msg;
        msg2_best = msg2;
        msg_best.ns = "itomp_best_endeffector";
        msg2_best.ns = "itomp_best_endeffector_path";

        msg_best.scale.x *= 1.2;
        msg_best.scale.y *= 1.2;
        msg_best.scale.z *= 1.2;

        msg2_best.scale.x *= 1.2;
        msg2_best.scale.y *= 1.2;
        msg2_best.scale.z *= 1.2;

        msg2_best.color = msg_best.color = COLOR_MAP[YELLOW];

        for (int i = 0; i < 3; ++i)
        {
            msg_frame[i] = msg;
            stringstream ss;
            ss << "tan" << i;
            msg_frame[i].ns = ss.str();
            msg_frame[i].type = visualization_msgs::Marker::LINE_LIST;
            msg_frame[i].color = COLOR_MAP[(1 << i)];
        }

        msg_tangent = msg;
        msg_tangent.ns = "vel";
        msg_tangent.type = visualization_msgs::Marker::CYLINDER;
        msg_tangent.color = COLOR_MAP[CYAN];
        msg_tangent.color.a = 0.5;
        msg_tangent.scale.x = 0.1;
        msg_tangent.scale.y = 0.1;
        msg_tangent.scale.z = 0.001;
    }

    for (unsigned int index = 0; index < animate_endeffector_segment_numbers_.size(); ++index)
	{
		if (index != 0)
			break;

        msg.id = trajectory_index * animate_endeffector_segment_numbers_.size() + index;
        msg2.id = msg.id;

		int sn = animate_endeffector_segment_numbers_[index];
		if (sn <= 0)
			continue;

		for (int j = point_start; j < point_end; j += marker_step)
		{
			geometry_msgs::Point point;
			point.x = segmentFrames[j][sn].p.x();
			point.y = segmentFrames[j][sn].p.y();
			point.z = segmentFrames[j][sn].p.z();
			msg.points.push_back(point);
            msg2.points.push_back(point);
		}

        ma.markers.push_back(msg);
        ma.markers.push_back(msg2);

        if (best)
        {
            msg_best.id = index;
            msg2_best.id = index;

            msg_best.points = msg.points;
            msg2_best.points = msg2.points;
            ma.markers.push_back(msg_best);
            ma.markers.push_back(msg2_best);


            msg_frame[0].id = index;
            msg_frame[1].id = index;
            msg_frame[2].id = index;
            for (int j = point_start; j < point_end; j += marker_step)
            {
                geometry_msgs::Point point;
                geometry_msgs::Point point2;
                KDL::Vector t;

                point.x = segmentFrames[j][sn].p.x();
                point.y = segmentFrames[j][sn].p.y();
                point.z = segmentFrames[j][sn].p.z();

                msg_frame[0].points.push_back(point);
                msg_frame[1].points.push_back(point);
                msg_frame[2].points.push_back(point);
                t = segmentFrames[j][sn].M * KDL::Vector(1.0, 0, 0) * 0.05;
                point2.x = point.x + t.x();
                point2.y = point.y + t.y();
                point2.z = point.z + t.z();
                msg_frame[0].points.push_back(point2);
                t = segmentFrames[j][sn].M * KDL::Vector(0, 1.0, 0) * 0.05;
                point2.x = point.x + t.x();
                point2.y = point.y + t.y();
                point2.z = point.z + t.z();
                msg_frame[1].points.push_back(point2);
                t = segmentFrames[j][sn].M * KDL::Vector(0, 0, 1.0) * 0.05;
                point2.x = point.x + t.x();
                point2.y = point.y + t.y();
                point2.z = point.z + t.z();
                msg_frame[2].points.push_back(point2);

                KDL::Vector dir = segmentFrames[j + 1][sn].p - segmentFrames[j - 1][sn].p;
                if (dir.Norm() > 0.01)
                {
                    dir.Normalize();
                    //msg_tangent.points.push_back(point);
                    t = 0.1 * dir;
                    point2.x = point.x + t.x();
                    point2.y = point.y + t.y();
                    point2.z = point.z + t.z();
                    //msg_tangent.points.push_back(point2);

                    msg_tangent.id = j;
                    msg_tangent.pose.position = point;

                    // rotation from z_axis to vel
                    Eigen::Vector3d z_axis(0, 0, 1.0);
                    Eigen::Vector3d vel;
                    vel.x() = dir.x();
                    vel.y() = dir.y();
                    vel.z() = dir.z();
                    Eigen::Vector3d w = z_axis.cross(vel);
                    Eigen::Quaterniond q(1.0 + z_axis.dot(vel), w.x(), w.y(), w.z());
                    q.normalize();
                    msg_tangent.pose.orientation.x = q.x();
                    msg_tangent.pose.orientation.y = q.y();
                    msg_tangent.pose.orientation.z = q.z();
                    msg_tangent.pose.orientation.w = q.w();

                    msg_tangent.scale.x = 0.1;
                    msg_tangent.scale.y = 0.1;
                    msg_tangent.scale.z = 0.001;

                    ma.markers.push_back(msg_tangent);
                }
                else
                {
                    msg_tangent.id = j;
                    msg_tangent.pose.position = point;

                    msg_tangent.scale.x = 0.0;
                    msg_tangent.scale.y = 0.0;
                    msg_tangent.scale.z = 0.0;

                    ma.markers.push_back(msg_tangent);
                }
            }

            ma.markers.push_back(msg_frame[0]);
            ma.markers.push_back(msg_frame[1]);
            ma.markers.push_back(msg_frame[2]);

        }
	}
    publish(ma);
}

void VisualizationManager::animateRoot(int numFreeVars, int freeVarStartIndex,
                                       const std::vector<std::vector<KDL::Frame> >& segmentFrames, bool best)
{
	const double scale = 0.05;

	visualization_msgs::Marker::_color_type RED;

	RED.a = 1.0;
	RED.r = 0.8;
	RED.g = 0.6;
	RED.b = 0.6;

	visualization_msgs::Marker msg;
	msg.header.frame_id = reference_frame_;
	msg.header.stamp = ros::Time::now();
	msg.ns = "root_link";
	msg.type = visualization_msgs::Marker::SPHERE_LIST;
	msg.action = visualization_msgs::Marker::ADD;

	msg.scale.x = scale;
	msg.scale.y = scale;
	msg.scale.z = scale;

	msg.points.resize(0);

	msg.color = RED;
	msg.id = 12;

	if (best)
	{
		msg.ns = "root_link_best";
		msg.id = 13;
		msg.color.a = 1.0;
		msg.color.r = 1.0;
		msg.color.g = 1.0;
		msg.color.b = 0.6;
	}

	int sn = root_segment_number_;
	if (sn > 0)
	{
		int marker_step = 1;
		for (int i = 0; i < numFreeVars; i = i + marker_step)
		{
			int j = i + freeVarStartIndex;
			geometry_msgs::Point point;
			point.x = segmentFrames[j][sn].p.x();
			point.y = segmentFrames[j][sn].p.y();
			point.z = segmentFrames[j][sn].p.z();

			msg.points.push_back(point);

		}
		vis_marker_publisher_.publish(msg);
	}
}

void VisualizationManager::animateCoM(int numFreeVars, int freeVarStartIndex,
                                      const std::vector<KDL::Vector>& CoM, bool best)
{
	const double scale = 0.05;

	visualization_msgs::Marker::_color_type RED;

	RED.a = 1.0;
	RED.r = 1.0;
	RED.g = 0.5;
	RED.b = 0.5;

	visualization_msgs::Marker msg;
	msg.header.frame_id = reference_frame_;
	msg.header.stamp = ros::Time::now();
	msg.ns = "itomp_CoM";
	msg.type = visualization_msgs::Marker::SPHERE_LIST;
	msg.action = visualization_msgs::Marker::ADD;

	msg.scale.x = scale;
	msg.scale.y = scale;
	msg.scale.z = scale;

	msg.points.resize(0);

	msg.color = RED;
	msg.id = 11;

	if (best)
	{
		msg.ns = "CoM_best";
		msg.id = 14;
		/*
		 msg.color.a = 1.0;
		 msg.color.r = 1.0;
		 msg.color.g = 1.0;
		 msg.color.b = 0.6;
		 */
	}

	int marker_step = 1;

	geometry_msgs::Point prevPt;
	for (int i = 0; i < numFreeVars; i = i + marker_step)
	{
		int j = i + freeVarStartIndex;
		geometry_msgs::Point point;
		point.x = CoM[j].x();
		point.y = CoM[j].y();
		point.z = CoM[j].z();
		msg.points.push_back(point);
		if (i != 0)
		{
			geometry_msgs::Point diff;
			diff.x = point.x - prevPt.x;
			diff.y = point.y - prevPt.y;
			diff.z = point.z - prevPt.z;
			double dist = sqrt(
                              diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
			int numNeeded = (int) (dist / scale * 2);
			for (int k = 0; k < numNeeded; ++k)
			{
				geometry_msgs::Point dummy;
				dummy.x = prevPt.x + diff.x * (k + 1) / numNeeded;
				dummy.y = prevPt.y + diff.y * (k + 1) / numNeeded;
				dummy.z = prevPt.z + diff.z * (k + 1) / numNeeded;
				msg.points.push_back(dummy);
			}
		}
		prevPt = point;
	}
	vis_marker_publisher_.publish(msg);
}

void VisualizationManager::animatePath(int trajectory_index,
                                       const ItompCIOTrajectory* traj, bool is_best, const std::string& group_name)
{
    const int step = 2;

	if (!is_best)
		return;

    std::vector<std::string> link_names = robot_model_->getRobotModel()->getJointModelGroup(group_name)->getLinkModelNames();

    std_msgs::ColorRGBA WHITE, YELLOW, RED, GRAY, DIMGRAY;
	WHITE.a = 1.0;
	WHITE.r = 1.0;
	WHITE.g = 1.0;
	WHITE.b = 1.0;
	YELLOW.a = 1.0;
	YELLOW.r = 0.0;
	YELLOW.g = 1.0;
	YELLOW.b = 1.0;
	RED.a = 0.5;
	RED.r = 1.0;
	RED.g = 0.0;
	RED.b = 0.0;
    GRAY.a = 1.0;
    GRAY.r = 0.35;
    GRAY.g = 0.35;
    GRAY.b = 0.35;
    DIMGRAY.a = 0.2;
    DIMGRAY.r = 0.35;
    DIMGRAY.g = 0.35;
    DIMGRAY.b = 0.35;
	ros::Duration dur(100.0);

	visualization_msgs::MarkerArray ma;
    for (int point = 0; point < traj->getNumPoints(); point += step)
	{
		visualization_msgs::MarkerArray ma_point;
		const Eigen::MatrixXd& mat = traj->getTrajectoryPoint(point);
		robot_states_[trajectory_index]->setVariablePositions(mat.data());
		robot_states_[trajectory_index]->updateLinkTransforms();
		std::string ns = "wp_" + boost::lexical_cast<std::string>(point);
        robot_states_[trajectory_index]->getRobotMarkers(ma_point, link_names, point == 0 ? GRAY : DIMGRAY, ns, dur);
        ma.markers.insert(ma.markers.end(), ma_point.markers.begin(), ma_point.markers.end());

        // now only the first waypoint is shown
        break;
    }

	vis_marker_array_publisher_.publish(ma);
}

void VisualizationManager::animateCollisionSpheres(int trajectory_index, const ItompCIOTrajectory* traj, bool is_best, const std::string& group_name)
{
    if (!is_best)
        return;

    std::string reference_frame = robot_model_->getRobotModel()->getModelFrame();

    std_msgs::ColorRGBA RED;
    RED.a = 0.5;
    RED.r = 1.0;
    RED.g = 0.0;
    RED.b = 0.0;
    ros::Duration dur(100.0);

    visualization_msgs::MarkerArray ma;

    visualization_msgs::Marker msg;
    msg.header.frame_id = reference_frame;
    msg.header.stamp = ros::Time::now();
    msg.type = visualization_msgs::Marker::SPHERE_LIST;
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.0;
    msg.pose.orientation.w = 1.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.color = RED;
    msg.lifetime = dur;

    for (int point = 0; point < traj->getNumPoints(); point += 10)
    {
        const Eigen::MatrixXd& mat = traj->getTrajectoryPoint(point);
        robot_states_[trajectory_index]->setVariablePositions(mat.data());
        robot_states_[trajectory_index]->updateLinkTransforms();

        int marker_id = 0;

        const std::map<std::string, std::vector<CollisionSphere> >& collision_spheres = robot_model_->getRobotCollisionModel().getCollisionSpheres();
        for (std::map<std::string, std::vector<CollisionSphere> >::const_iterator it = collision_spheres.begin(); it != collision_spheres.end() ; ++it)
        {
            const Eigen::Affine3d& transform = robot_states_[trajectory_index]->getGlobalLinkTransform(it->first);
            const std::vector<CollisionSphere>& spheres = it->second;
            if (spheres.size() == 0)
                continue;

            msg.ns = "global_spheres_" + boost::lexical_cast<std::string>(point);
            msg.id = marker_id++;
            msg.scale.x = spheres[0].radius_ * 2;
            msg.scale.y = spheres[0].radius_ * 2;
            msg.scale.z = spheres[0].radius_ * 2;
            msg.points.clear();
            for (unsigned int i = 0; i < spheres.size(); ++i)
            {
                Eigen::Vector3d global_position = transform * spheres[i].position_;
                geometry_msgs::Point point;
                point.x = global_position(0);
                point.y = global_position(1);
                point.z = global_position(2);
                msg.points.push_back(point);
            }

            ma.markers.push_back(msg);
        }
    }

    vis_marker_array_publisher_.publish(ma);
}

}
