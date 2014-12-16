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

#include <itomp_ca_planner/util/point_to_triangle_projection.h>

namespace itomp_ca_planner
{
// TODO INDENTATION
//triangle / point distance
// source http://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf
Eigen::Vector3d ProjPoint2Triangle(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
                                   const Eigen::Vector3d &p2, const Eigen::Vector3d &source_position)
{
     Eigen::Vector3d edge0 = p1 - p0;
     Eigen::Vector3d edge1 = p2 - p0;
     Eigen::Vector3d v0 = p0 - source_position;

     double a = edge0.dot(edge0);
     double b = edge0.dot(edge1);
     double c = edge1.dot(edge1);
     double d = edge0.dot(v0);
     double e = edge1.dot(v0);

     double det = a*c - b*b;
     double s = b*e - c*d;
     double t = b*d - a*e;

     double lower_bound = 0.0, upper_bound = 1.0;


     if ( s + t < det )
     {
        if ( s < 0.0 )
        {
            if ( t < 0.0 )
            {
                if ( d < 0.0 )
                {
                    s = std::min(std::max(-d/a, lower_bound), upper_bound);
                    t = 0.0;
                }
                else
                {
                     s = 0.0;
                     t = std::min(std::max(-e/c, lower_bound), upper_bound);
                }
             }
             else {
                 s = 0.0;
                 t = std::min(std::max(-e/c, lower_bound), upper_bound);
             }
         }
         else if ( t < 0.0 ) {
             s = std::min(std::max(-d/a, lower_bound), upper_bound);
             t = 0.0;
         }
         else {
             float invDet = 1.0 / det;
             s *= invDet;
             t *= invDet;
         }
     }
     else {
         if ( s < 0.0 ) {
             double tmp0 = b+d;
             double tmp1 = c+e;
             if ( tmp1 > tmp0 ) {
                 double numer = tmp1 - tmp0;
                 double denom = a-2*b+c;
                 s = std::min(std::max(numer/denom, lower_bound), upper_bound);
                 t = 1-s;
             }
             else {
                 t = std::min(std::max(-e/c, lower_bound), upper_bound);
                 s = 0.f;
             }
         }
         else if ( t < 0.f ) {
             if ( a+d > b+e ) {
                 double numer = c+e-b-d;
                 double denom = a-2*b+c;
                 s = std::min(std::max(numer/denom, lower_bound),upper_bound);
                 t = 1-s;
             }
             else {
                 s = std::min(std::max(-e/c, lower_bound), upper_bound);
                 t = 0.f;
             }
         }
         else {
             double numer = c+e-b-d;
             double denom = a-2*b+c;
             s = std::min(std::max(numer/denom, lower_bound), upper_bound);
             t = 1.0 - s;
         }
     }
     return p0 + s * edge0 + t * edge1;
}
} // namespace itomp_ca_planner
