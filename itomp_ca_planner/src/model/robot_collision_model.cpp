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
#include <itomp_ca_planner/model/robot_collision_model.h>
#include <ros/ros.h>
#include <geometric_shapes/body_operations.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/publisher.h>

using namespace std;

namespace itomp_ca_planner
{

RobotCollisionModel::RobotCollisionModel()
{

}

RobotCollisionModel::~RobotCollisionModel()
{

}

void RobotCollisionModel::computeCollisionSpheres(const shapes::Mesh *mesh, std::vector<CollisionSphere>& collision_spheres)
{
    Eigen::Vector3d box_size, box_offset;
    computeMeshBoundingBox(mesh, box_size, box_offset);

    double max_radius = 0.0;
    for (unsigned int i = 0; i < mesh->vertex_count ; ++i)
    {
        Eigen::Vector3d vertex_position;
        for (unsigned int j = 0; j < 3; ++j)
             vertex_position[j] = mesh->vertices[3 * i + j];

        Eigen::Vector3d diff = vertex_position - box_offset;
        diff(2) = 0.0;
        double radius = diff.norm();
        if (max_radius < radius)
            max_radius = radius;
    }

    Eigen::Vector3d min_z, max_z;
    min_z = max_z = box_offset;
    for (unsigned int i = 0; i < mesh->vertex_count ; ++i)
    {
        Eigen::Vector3d vertex_position;
        for (unsigned int j = 0; j < 3; ++j)
             vertex_position[j] = mesh->vertices[3 * i + j];

        if (vertex_position(2) > max_z(2))
        {
            double dist = (vertex_position - max_z).norm();
            if (dist > max_radius)
            {
                double sq_dist_xy = (vertex_position(0) - max_z(0)) * (vertex_position(0) - max_z(0)) + (vertex_position(1) - max_z(1)) * (vertex_position(1) - max_z(1));
                max_z(2) = vertex_position(2) - std::sqrt(std::max(0.0, max_radius * max_radius - sq_dist_xy));
            }
        }
        else if (vertex_position(2) < min_z(2))
        {
            double dist = (vertex_position - min_z).norm();
            if (dist > max_radius)
            {
                double sq_dist_xy = (vertex_position(0) - min_z(0)) * (vertex_position(0) - min_z(0)) + (vertex_position(1) - min_z(1)) * (vertex_position(1) - min_z(1));
                min_z(2) = vertex_position(2) + std::sqrt(std::max(0.0, max_radius * max_radius - sq_dist_xy));
            }
        }
    }

    double spacing = max_radius;
    double distance = max_z(2) - min_z(2);
    int num_points = std::ceil(distance / spacing) + 1;
    spacing = distance / (num_points - 1.0);

    Eigen::Vector3d point_pos;
    for (int i = 0; i < num_points; i++)
    {
            point_pos = Eigen::Vector3d(box_offset(0), box_offset(1), min_z(2) + spacing * i);
            collision_spheres.push_back(CollisionSphere(point_pos, max_radius));
    }
}

void RobotCollisionModel::computeMeshBoundingBox(const shapes::Mesh *mesh, Eigen::Vector3d& box_size, Eigen::Vector3d& box_offset)
{
    ROS_ASSERT(mesh->type == shapes::MESH);

    double maxX = -std::numeric_limits<double>::infinity(), maxY = -std::numeric_limits<double>::infinity(), maxZ = -std::numeric_limits<double>::infinity();
    double minX =  std::numeric_limits<double>::infinity(), minY =  std::numeric_limits<double>::infinity(), minZ  = std::numeric_limits<double>::infinity();

    for (unsigned int i = 0; i < mesh->vertex_count ; ++i)
    {
        double vx = mesh->vertices[3 * i    ];
        double vy = mesh->vertices[3 * i + 1];
        double vz = mesh->vertices[3 * i + 2];

        if (maxX < vx) maxX = vx;
        if (maxY < vy) maxY = vy;
        if (maxZ < vz) maxZ = vz;

        if (minX > vx) minX = vx;
        if (minY > vy) minY = vy;
        if (minZ > vz) minZ = vz;
    }

    if (maxX < minX) maxX = minX = 0.0;
    if (maxY < minY) maxY = minY = 0.0;
    if (maxZ < minZ) maxZ = minZ = 0.0;

    box_size = Eigen::Vector3d(maxX - minX, maxY - minY, maxZ - minZ);
    box_offset = Eigen::Vector3d((minX + maxX) / 2.0, (minY + maxY) / 2.0, (minZ + maxZ) / 2.0);
}

bool RobotCollisionModel::init(robot_model::RobotModelPtr& robot_model)
{
    ros::NodeHandle node_handle;
    ros::Publisher vis_marker_array_publisher = node_handle.advertise<visualization_msgs::MarkerArray>("itomp_planner/test", 10);
    std::string reference_frame = robot_model->getModelFrame();

    const std::vector<const robot_model::LinkModel*>& link_models = robot_model->getLinkModelsWithCollisionGeometry();

    visualization_msgs::MarkerArray ma;

    int marker_id = 0;
    for (int i = 0; i < link_models.size(); ++i)
    {
        const robot_model::LinkModel* link_model = link_models[i];
        const std::string link_name = link_model->getName();
        const std::vector<shapes::ShapeConstPtr>& shapes = link_model->getShapes();
        for (int j = 0; j < shapes.size(); ++j)
        {
            shapes::Shape* shape = shapes[j]->clone();
            if (shape->type != shapes::MESH)
                continue;

            const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape);

            Eigen::Vector3d box_size, box_offset;
            computeMeshBoundingBox(mesh, box_size, box_offset);

            /*
            bodies::Body* body = bodies::createBodyFromShape(shape);
            if (body == NULL)
                continue;

            body->setPadding(0.0);
            body->setScale(1.0);
            bodies::BoundingCylinder cyl;
            body->computeBoundingCylinder(cyl);
            */

            visualization_msgs::Marker msg;
            msg.header.frame_id = reference_frame;
            msg.header.stamp = ros::Time::now();
            msg.ns = link_name;
            msg.type = visualization_msgs::Marker::MESH_RESOURCE;
            msg.action = visualization_msgs::Marker::ADD;
            msg.scale.x = 1.0;
            msg.scale.y = 1.0;
            msg.scale.z = 1.0;
            msg.id = marker_id;
            msg.pose.position.x = 0.0;
            msg.pose.position.y = 0.0;
            msg.pose.position.z = 0.0;
            msg.pose.orientation.x = 0.0;
            msg.pose.orientation.y = 0.0;
            msg.pose.orientation.z = 0.0;
            msg.pose.orientation.w = 1.0;
            msg.color.a = 1.0;
            msg.color.r = 0.0;
            msg.color.g = 1.0;
            msg.color.b = 0.0;
            msg.mesh_resource = link_model->getVisualMeshFilename();
            ma.markers.push_back(msg);

            //visualization_msgs::Marker msg;
            /*
            msg.header.frame_id = reference_frame;
            msg.header.stamp = ros::Time::now();
            stringstream ss;
            ss << link_name << "_cylinder";
            msg.ns = ss.str();
            msg.type = visualization_msgs::Marker::CYLINDER;
            msg.action = visualization_msgs::Marker::ADD;

            msg.scale.x = cyl.radius * 2;
            msg.scale.y = cyl.radius * 2;
            msg.scale.z = cyl.length;
            msg.pose.position.x = cyl.pose.translation().x();
            msg.pose.position.y = cyl.pose.translation().y();
            msg.pose.position.z = cyl.pose.translation().z();
            Eigen::Quaterniond quat(cyl.pose.linear());
            msg.pose.orientation.w = quat.w();
            msg.pose.orientation.x = quat.x();
            msg.pose.orientation.y = quat.y();
            msg.pose.orientation.z = quat.z();
            msg.color.a = 0.5;
            msg.color.r = 1.0;
            msg.color.g = 0.0;
            msg.color.b = 0.0;
            msg.id = marker_id;
            */

            msg.header.frame_id = reference_frame;
            msg.header.stamp = ros::Time::now();
            stringstream ss;
            ss << link_name << "_box";
            msg.ns = ss.str();
            msg.type = visualization_msgs::Marker::CUBE;
            msg.action = visualization_msgs::Marker::ADD;

            msg.scale.x = box_size(0);
            msg.scale.y = box_size(1);
            msg.scale.z = box_size(2);
            msg.pose.position.x = box_offset(0);
            msg.pose.position.y = box_offset(1);
            msg.pose.position.z = box_offset(2);
            msg.pose.orientation.w = 1.0;
            msg.pose.orientation.x = 0.0;
            msg.pose.orientation.y = 0.0;
            msg.pose.orientation.z = 0.0;
            msg.color.a = 0.5;
            msg.color.r = 1.0;
            msg.color.g = 0.0;
            msg.color.b = 0.0;
            msg.id = marker_id;

            ma.markers.push_back(msg);

            std::vector<CollisionSphere> collision_spheres;
            computeCollisionSpheres(mesh, collision_spheres);

            if (collision_spheres.size() > 0)
            {
                msg.header.frame_id = reference_frame;
                msg.header.stamp = ros::Time::now();
                stringstream ss;
                ss << link_name << "_spheres";
                msg.ns = ss.str();
                msg.type = visualization_msgs::Marker::SPHERE_LIST;
                msg.action = visualization_msgs::Marker::ADD;

                msg.scale.x = collision_spheres[0].radius_ * 2;
                msg.scale.y = collision_spheres[0].radius_ * 2;
                msg.scale.z = collision_spheres[0].radius_ * 2;
                msg.pose.position.x = 0.0;
                msg.pose.position.y = 0.0;
                msg.pose.position.z = 0.0;
                msg.points.clear();
                for (unsigned int k = 0; k < collision_spheres.size(); ++k)
                {
                    geometry_msgs::Point point;
                    point.x = collision_spheres[k].position_(0);
                    point.y = collision_spheres[k].position_(1);
                    point.z = collision_spheres[k].position_(2);
                    msg.points.push_back(point);
                }
                msg.pose.orientation.w = 1.0;
                msg.pose.orientation.x = 0.0;
                msg.pose.orientation.y = 0.0;
                msg.pose.orientation.z = 0.0;
                msg.color.a = 0.5;
                msg.color.r = 1.0;
                msg.color.g = 0.0;
                msg.color.b = 0.0;
                msg.id = marker_id;

                ma.markers.push_back(msg);
            }

            ++marker_id;

            delete shape;




            vis_marker_array_publisher.publish(ma);
        }
    }

    vis_marker_array_publisher.publish(ma);

    return true;
}

}
