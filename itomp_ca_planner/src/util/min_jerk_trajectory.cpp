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

#include <itomp_ca_planner/util/min_jerk_trajectory.h>

MinJerkTrajectory::MinJerkTrajectory(double x0, double v0, double a0, double x1, double v1, double a1) :
	x0_(x0), x1_(x1), v0_(v0), v1_(v1), a0_(a0), a1_(a1)
{
	// TODO: for now, x1 = v1 = a1 = 0
	coeff[0] = x0_;
	coeff[1] = v0_;
	coeff[2] = 0.5 * a0_;
	coeff[3] = -1.5 * a0_ - 6.0 * v0_ - 10.0 * x0_ + 0.5 * a1_ - 4.0 * v1_ + 10.0 * x1_;
	coeff[4] = 1.5 * a0_ + 8.0 * v0_ + 15.0 * x0_ - 1.0 * a1_ + 7.0 * v1_ -15.0 * x1_;
	coeff[5] = -0.5 * a0_ - 3.0 * v0_ - 6.0 * x0_ + 0.5 * a1_ - 3.0 * v1_ + 6.0 * x1_;
}

MinJerkTrajectory::~MinJerkTrajectory()
{

}

double MinJerkTrajectory::operator()(double t)
{
	double ret = coeff[0];
	double mul = t;
	for (int i = 1; i < 6; ++i)
	{
		ret += mul * coeff[i];
		mul *= t;
	}
	return ret;
}
