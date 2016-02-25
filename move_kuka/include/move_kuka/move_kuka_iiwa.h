/*
 * move_itomp.h
 *
 *  Created on: Sep 23, 2014
 *      Author: chonhyon
 */

#ifndef MOVE_KUKA_IIWA_H_
#define MOVE_KUKA_IIWA_H_

#include <moveit_msgs/RobotTrajectory.h>
#include <boost/serialization/nvp.hpp>

namespace move_kuka
{

struct TASK_FRAME_INDEX
{
    TASK_FRAME_INDEX(int r, int c, int d)
        : row(r), col(c), direction(d)
    {

    }

    int row;
    int col;
    int direction; // 0 : top 1 : bottom
};

class MoveKukaIIWA
{
public:
        MoveKukaIIWA(const ros::NodeHandle& node_handle);
        ~MoveKukaIIWA();

        void run(const std::string& group_name);

protected:
	void loadStaticScene();
        void initStartGoalStates(planning_interface::MotionPlanRequest& req, const std::vector<Eigen::Affine3d>& end_effector_poses,
                                 std::vector<robot_state::RobotState>& robot_states, int index = 0);
        bool initTask(std::vector<Eigen::Affine3d>& end_effector_poses, std::vector<robot_state::RobotState>& robot_states);
	bool isStateCollide(const robot_state::RobotState& state);
	bool isStateSingular(robot_state::RobotState& state);
        void loadFrames();

        void plan(planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res, bool use_itomp);
        bool computeIKState(robot_state::RobotState& ik_state, const Eigen::Affine3d& end_effector_state, bool rand = false);

        void renderStartGoalStates(robot_state::RobotState& start_state, robot_state::RobotState& goal_state);
        void drawPath(int id, const Eigen::Vector3d& from, const Eigen::Vector3d& to);
	void drawEndeffectorPosition(int id, const Eigen::Vector3d& position);
        void drawFrames();
        void drawResults(moveit_msgs::DisplayTrajectory& display_trajectory);
        void animateResults(moveit_msgs::DisplayTrajectory& display_trajectory);

	ros::NodeHandle node_handle_;
	robot_model::RobotModelPtr robot_model_;
	planning_scene::PlanningScenePtr planning_scene_;
	planning_interface::PlannerManagerPtr itomp_planner_instance_;

	ros::Publisher planning_scene_diff_publisher_;
	ros::Publisher display_publisher_;
	ros::Publisher vis_marker_array_publisher_;

	std::string group_name_;

        robot_state::RobotStatePtr last_goal_state_;

        Eigen::Affine3d mat_shelf_frame_;
        std::vector<std::vector<Eigen::Affine3d> > mat_task_frames_0_;
        std::vector<std::vector<Eigen::Affine3d> > mat_task_frames_78_;
        Eigen::Affine3d mat_rivet_magazine_;
};

/**
Serialization and deserilization of a 4 x 4 homogenous matrix.
*/
class HMatrix
{
public:
    double data_[16];
    double& operator[](std::size_t idx)       { return data_[idx]; }
    const double& operator[](std::size_t idx) const { return data_[idx]; }
    Eigen::Affine3d getEigen() const {
        Eigen::Matrix4d m;
        for (int i = 0; i < 16; ++i)
            m(i / 4, i % 4) = data_[i];
        Eigen::Affine3d am(m);
        return am;
    }
};

template<class Archive>
void serialize(Archive & ar, HMatrix& m, const unsigned int version)
{
        ar & boost::serialization::make_nvp("r11", m[0]);
        ar & boost::serialization::make_nvp("r12", m[1]);
        ar & boost::serialization::make_nvp("r13", m[2]);
        ar & boost::serialization::make_nvp("r14", m[3]);

        ar & boost::serialization::make_nvp("r21", m[4]);
        ar & boost::serialization::make_nvp("r22", m[5]);
        ar & boost::serialization::make_nvp("r23", m[6]);
        ar & boost::serialization::make_nvp("r24", m[7]);

        ar & boost::serialization::make_nvp("r31", m[8]);
        ar & boost::serialization::make_nvp("r32", m[9]);
        ar & boost::serialization::make_nvp("r33", m[10]);
        ar & boost::serialization::make_nvp("r34", m[11]);

        ar & boost::serialization::make_nvp("r41", m[12]);
        ar & boost::serialization::make_nvp("r42", m[13]);
        ar & boost::serialization::make_nvp("r43", m[14]);
        ar & boost::serialization::make_nvp("r44", m[15]);

}

}

#endif /* MOVE_KUKA_IIWA_H_ */
