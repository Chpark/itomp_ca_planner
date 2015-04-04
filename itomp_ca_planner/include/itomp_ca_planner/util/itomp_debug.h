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
#ifndef ITOMP_DEBUG_H_
#define ITOMP_DEBUG_H_

#include <kdl/jntarray.hpp>

namespace itomp_ca_planner
{
inline void debugJointArray(KDL::JntArray& joint_array)
{
  for (unsigned int i = 0; i < joint_array.rows(); i++)
  {
    std::cout << joint_array(i) << "\t";
  }
  std::cout << std::endl;
}

#define MEASURE_TIME

#ifdef MEASURE_TIME
#define INIT_TIME_MEASUREMENT(N) std::vector<ros::Time> times;static int count=0;static double elapsed[N];if (count==0) for(int i=0;i<N;++i) elapsed[i]=0.0;
#define ADD_TIMER_POINT times.push_back(ros::Time::now());
#define UPDATE_TIME for(int i=0; i < times.size()-1; ++i) elapsed[i]+=(times[i+1] - times[i]).toSec();
#define PRINT_TIME(name, c) if (++count % c == 0) for(int i=0; i < times.size()-1; ++i) printf("%s Timing %d : %f %f\n", #name, i, elapsed[i], elapsed[i] / count);
#else
#define INIT_TIME_MEASUREMENT(N)
#define ADD_TIMER_POINT
#define UPDATE_TIME
#define PRINT_TIME(name, c)
#endif

}
#endif
