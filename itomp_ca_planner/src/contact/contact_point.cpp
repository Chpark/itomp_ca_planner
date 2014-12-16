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

#include <itomp_ca_planner/model/itomp_robot_model.h>
#include <itomp_ca_planner/util/vector_util.h>
#include <itomp_ca_planner/contact/contact_point.h>
#include <itomp_ca_planner/contact/ground_manager.h>

using namespace std;

namespace itomp_ca_planner
{

ContactPoint::ContactPoint(const string& linkName, const ItompRobotModel* robot_model)
{
  linkName_ = linkName;
  linkSegmentNumber_ = robot_model->getForwardKinematicsSolver()->segmentNameToIndex(linkName);
}

ContactPoint::~ContactPoint()
{

}

void ContactPoint::getPosition(int point, KDL::Vector& position,
    const std::vector<std::vector<KDL::Frame> >& segmentFrames) const
{
  position = segmentFrames[point][linkSegmentNumber_].p;
}

void ContactPoint::getFrame(int point, KDL::Frame& frame,
    const std::vector<std::vector<KDL::Frame> >& segmentFrames) const
{
  frame = segmentFrames[point][linkSegmentNumber_];
}

void ContactPoint::updateContactViolationVector(int start, int end, double discretization,
    vector<Vector4d>& contactViolationVector, vector<KDL::Vector>& contactPointVelVector,
    const vector<vector<KDL::Frame> >& segmentFrames, const planning_scene::PlanningSceneConstPtr& planning_scene) const
{
  vector<KDL::Vector> contactPointPosVector(contactViolationVector.size());
  for (int i = start; i <= end; ++i)
  {
    KDL::Vector position = segmentFrames[i][linkSegmentNumber_].p;
    KDL::Vector normal = segmentFrames[i][linkSegmentNumber_].M * KDL::Vector(0.0, 0.0, 1.0);
    normal.Normalize();

    KDL::Vector groundPosition;
    KDL::Vector groundNormal;
    GroundManager::getInstance().getNearestGroundPosition(position, groundPosition, groundNormal, planning_scene);

    KDL::Vector diff = position - groundPosition;
    double angle = acos(KDL::dot(normal, groundNormal));

    contactViolationVector[i] = Vector4d(diff.x(), diff.y(), diff.z(), angle);
    contactPointPosVector[i] = position;
  }
  //for (int i = 0; i < start; ++i)
//  if (start == 1)
  //  contactPointPosVector[0] = segmentFrames[1][linkSegmentNumber_].p;
  //else
    contactPointPosVector[start - 1] = segmentFrames[start - 1][linkSegmentNumber_].p;
  //for (int i = end + 1; i < contactPointPosVector.size(); ++i)
//  if (end == contactPointPosVector.size() - 2)
  //  contactPointPosVector[end + 1] = segmentFrames[end][linkSegmentNumber_].p;
  //else
    contactPointPosVector[end + 1] = segmentFrames[end + 1][linkSegmentNumber_].p;

  itomp_ca_planner::getVectorVelocities(start, end, discretization, contactPointPosVector, contactPointVelVector,
      KDL::Vector::Zero());
}

double ContactPoint::getDistanceToGround(int point, const std::vector<std::vector<KDL::Frame> >& segmentFrames, const planning_scene::PlanningSceneConstPtr& planning_scene) const
{
  KDL::Vector position;
  getPosition(point, position, segmentFrames);

  KDL::Vector groundPosition;
  KDL::Vector groundNormal;
  GroundManager::getInstance().getNearestGroundPosition(position, groundPosition, groundNormal, planning_scene);

  KDL::Vector diff = position - groundPosition;
  if (diff.z() < 0.0)
    diff.z(0.0);
  return diff.Norm();
}
}

