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

void RobotCollisionModel::computeCollisionSpheres2(const shapes::Mesh *mesh, std::vector<CollisionSphere>& collision_spheres)
{
    Eigen::Vector3d box_size, box_offset;
    computeMeshBoundingBox(mesh, box_size, box_offset);

    unsigned int axis_length_order[] = {0, 2, 1};

    double radius = 0.5 * box_size[axis_length_order[2]] * 1.0;

    Eigen::Vector3d min_axis, max_axis;
    min_axis = max_axis = box_offset;

    for (unsigned int i = 0; i < mesh->vertex_count ; ++i)
    {
        Eigen::Vector3d vertex_position;
        for (unsigned int j = 0; j < 3; ++j)
             vertex_position[j] = mesh->vertices[3 * i + j];
        vertex_position(axis_length_order[1]) = 0.0;

        if (vertex_position(axis_length_order[0]) > max_axis(axis_length_order[0]))
        {
            Eigen::Vector3d diff = vertex_position - max_axis;
            diff(axis_length_order[1]) = 0.0;
            double dist = diff.norm();
            if (dist > radius)
            {
                double sq_dist_2d = (vertex_position(axis_length_order[2]) - max_axis(axis_length_order[2])) *
                                    (vertex_position(axis_length_order[2]) - max_axis(axis_length_order[2]));
                max_axis(axis_length_order[0]) = vertex_position(axis_length_order[0]) - std::sqrt(std::max(0.0, radius * radius - sq_dist_2d));
            }
        }
        else if (vertex_position(axis_length_order[0]) < min_axis(axis_length_order[0]))
        {
            Eigen::Vector3d diff = vertex_position - min_axis;
            diff(axis_length_order[1]) = 0.0;
            double dist = diff.norm();
            if (dist > radius)
            {
                double sq_dist_2d = (vertex_position(axis_length_order[2]) - min_axis(axis_length_order[2])) *
                                    (vertex_position(axis_length_order[2]) - min_axis(axis_length_order[2]));
                min_axis(axis_length_order[0]) = vertex_position(axis_length_order[0]) + std::sqrt(std::max(0.0, radius * radius - sq_dist_2d));
            }
        }
    }

    Eigen::Vector3d spacing;
    spacing(0) = spacing(1) = spacing(2) = radius;
    double distance = max_axis(axis_length_order[0]) - min_axis(axis_length_order[0]);
    int num_points = std::ceil(distance / radius) + 1;
    spacing(axis_length_order[0]) = distance / (num_points - 1.0);

    std::vector<std::pair<double, double> > min_max_points(num_points,
            std::make_pair<double, double>(min_axis(axis_length_order[1]), max_axis(axis_length_order[1])));

    for (unsigned int i = 0; i < mesh->vertex_count ; ++i)
    {
        Eigen::Vector3d vertex_position;
        for (unsigned int j = 0; j < 3; ++j)
             vertex_position[j] = mesh->vertices[3 * i + j];

        int point_index = (int)std::floor((vertex_position(axis_length_order[0]) - (min_axis(axis_length_order[0]))) / spacing(axis_length_order[0]) + 0.5);
        if (point_index < 0)
            point_index = 0;
        if (point_index >= num_points)
            point_index = num_points - 1;
        std::pair<double, double>& point_min_max = min_max_points[point_index];

        if (vertex_position(axis_length_order[1]) > point_min_max.second)
        {
            Eigen::Vector3d point_max;
            point_max(axis_length_order[0]) = min_axis(axis_length_order[0]) + spacing(axis_length_order[0]) * point_index;
            point_max(axis_length_order[2]) = box_offset(axis_length_order[2]);
            point_max(axis_length_order[1]) = point_min_max.second;
            Eigen::Vector3d diff = vertex_position - point_max;
            double dist = diff.norm();
            if (dist > radius)
            {
                double sq_dist_2d = (vertex_position(axis_length_order[0]) - point_max(axis_length_order[0])) *
                                    (vertex_position(axis_length_order[0]) - point_max(axis_length_order[0]));
                point_min_max.second = vertex_position(axis_length_order[1]) - std::sqrt(std::max(0.0, radius * radius - sq_dist_2d));
            }
        }
        else if (vertex_position(axis_length_order[1]) < point_min_max.first)
        {
            Eigen::Vector3d point_min;
            point_min(axis_length_order[0]) = min_axis(axis_length_order[0]) + spacing(axis_length_order[0]) * point_index;
            point_min(axis_length_order[2]) = box_offset(axis_length_order[2]);
            point_min(axis_length_order[1]) = point_min_max.first;
            Eigen::Vector3d diff = vertex_position - point_min;
            double dist = diff.norm();
            if (dist > radius)
            {
                double sq_dist_2d = (vertex_position(axis_length_order[0]) - point_min(axis_length_order[0])) *
                                    (vertex_position(axis_length_order[0]) - point_min(axis_length_order[0]));
                point_min_max.first = vertex_position(axis_length_order[1]) + std::sqrt(std::max(0.0, radius * radius - sq_dist_2d));
            }
        }
    }

    Eigen::Vector3d point_pos;
    point_pos(axis_length_order[2]) = box_offset(axis_length_order[2]);
    for (int i = 0; i < num_points; i++)
    {
        point_pos(axis_length_order[0]) = min_axis(axis_length_order[0]) + spacing(axis_length_order[0]) * i;

        std::pair<double, double>& point_min_max = min_max_points[i];
        double distance = point_min_max.second - point_min_max.first;
        int num_points2 = std::ceil(distance / radius) + 1;
        spacing(axis_length_order[1]) = distance / (num_points2 - 1.0);

        for (int j = 0; j < num_points2; ++j)
        {
            point_pos(axis_length_order[1]) = point_min_max.first + spacing(axis_length_order[1]) * j;
            //point_pos(axis_length_order[1]) = box_offset(axis_length_order[1]);
            collision_spheres.push_back(CollisionSphere(point_pos, radius));

            //break;
        }
    }
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
    const std::vector<const robot_model::LinkModel*>& link_models = robot_model->getLinkModelsWithCollisionGeometry();
    for (int i = 0; i < link_models.size(); ++i)
    {
        const robot_model::LinkModel* link_model = link_models[i];
        const std::string link_name = link_model->getName();
        const std::vector<shapes::ShapeConstPtr>& shapes = link_model->getShapes();
        for (int j = 0; j < shapes.size(); ++j)
        {
            if (shapes[j]->type != shapes::MESH)
                continue;

            const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shapes[j].get());

            std::vector<CollisionSphere> collision_spheres;
            if (link_name != "revetting_tool")
                computeCollisionSpheres(mesh, collision_spheres);
            else
                computeCollisionSpheres2(mesh, collision_spheres);

            collision_spheres_[link_name].insert(collision_spheres_[link_name].end(), collision_spheres.begin(), collision_spheres.end());
        }
    }
    return true;
}

}
