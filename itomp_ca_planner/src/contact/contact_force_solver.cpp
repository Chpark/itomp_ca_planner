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

#include <ros/ros.h>
#include <itomp_ca_planner/common.h>
#include <itomp_ca_planner/contact/contact_force_solver.h>
#include <itomp_ca_planner/optimization/f2c.h>
#include <itomp_ca_planner/optimization/clapack.h>

namespace itomp_ca_planner
{
////////////////////////////////////////////////////////////////////////////////
const double w1 = 1e-1;
const double w2 = 1e-7;
const double dz = -0.1;

ContactForceSolver::ContactForceSolver()
{

}

ContactForceSolver::~ContactForceSolver()
{

}

void ContactForceSolver::operator()(double friction_coeff, std::vector<KDL::Vector>& contact_forces,
    std::vector<KDL::Vector>& contact_positions, const KDL::Wrench& wrench, const std::vector<double>& contact_values,
    const std::vector<KDL::Frame> contact_parent_frames)
{
  const double k_0 = 1e-4;
  const double k_1 = 1e-5;

  int num_contacts = contact_forces.size();

  // compute contact forces using linear programming
  {
    char trans = 'N';
    integer m = 6 + num_contacts * 3;
    integer n = num_contacts * 3;
    integer nrhs = 1;
    integer lda = m;
    integer ldb = m;
    integer lwork = m * n;
    integer info = 0;

    doublereal A[m * n];
    doublereal b[m];
    doublereal work[lwork];

    // set A
    memset(A, 0, sizeof(doublereal) * m * n);
    int row = 0, column = 0;
    for (int i = 0; i < num_contacts; ++i)
    {
      // lapack uses column-major matrices
      // A[r + c * lda]
      A[0 + (column + 0) * lda] = 1;
      A[1 + (column + 1) * lda] = 1;
      A[2 + (column + 2) * lda] = 1;
      column += 3;
    }
    row = 3;
    column = 0;
    for (int i = 0; i < num_contacts; ++i)
    {
      const KDL::Vector& p = contact_parent_frames[i].p;
      A[(row + 0) + (column + 1) * lda] = w1 * -p.z();
      A[(row + 0) + (column + 2) * lda] = w1 * p.y();
      A[(row + 1) + (column + 0) * lda] = w1 * p.z();
      A[(row + 1) + (column + 2) * lda] = w1 * -p.x();
      A[(row + 2) + (column + 0) * lda] = w1 * -p.y();
      A[(row + 2) + (column + 1) * lda] = w1 * p.x();

      column += 3;
    }
    row = 6;
    column = 0;
    for (int i = 0; i < num_contacts; ++i)
    {
      const doublereal contact_value = 5.0 * contact_values[i];
      const doublereal e = (i < 2 ? 1.0 : 1.0) * k_0 / (contact_value * contact_value * contact_value * contact_value + k_1);
      A[(row + 0) + (column + 0) * lda] = e;
      A[(row + 1) + (column + 1) * lda] = e;
      A[(row + 2) + (column + 2) * lda] = e;
      row += 3;
      column += 3;
    }

    // set b
    b[0] = -wrench.force.x();
    b[1] = -wrench.force.y();
    b[2] = -wrench.force.z();
    b[3] = w1 * -wrench.torque.x();
    b[4] = w1 * -wrench.torque.y();
    b[5] = w1 * -wrench.torque.z();
    row = 6;
    for (int i = 0; i < num_contacts; ++i)
    {
      b[row] = 0;
      b[row + 1] = 0;
      b[row + 2] = 0;
      row += 3;
    }

    // solve
    dgels_(&trans, &m, &n, &nrhs, A, &lda, b, &ldb, work, &lwork, &info);

    row = 0;
    for (int i = 0; i < num_contacts; ++i)
    {
      double x = b[row];
      double y = b[row + 1];
      double z = b[row + 2];
      contact_forces[i] = KDL::Vector(x, y, z);
      row += 3;
    }
  }

  ///////////////////////////////////////////////////////////////
  // compute center of pressure (contact positions) using linear programming
  {
    const double CONTACT_MIN_DIR = -0.1;
    const double CONTACT_MAX_DIR = 0.19;
    const double CONTACT_MIN_RIGHT = -0.1;
    const double CONTACT_MAX_RIGHT = 0.1;

    char trans = 'N';
    integer m = 3 + num_contacts * 2;
    integer n = num_contacts * 2;
    integer nrhs = 1;
    integer lda = m;
    integer ldb = m;
    integer lwork = m * n;
    integer info = 0;

    doublereal A[m * n];
    doublereal b[m];
    doublereal work[lwork];

    // set A
    memset(A, 0, sizeof(doublereal) * m * n);
    int row = 0, column = 0;
    for (int i = 0; i < num_contacts; ++i)
    {
      // lapack uses column-major matrices
      // A[r + c * lda]
      A[0 + (column + 1) * lda] = contact_forces[i].z();
      A[1 + (column + 0) * lda] = -contact_forces[i].z();
      A[2 + (column + 0) * lda] = contact_forces[i].y();
      A[2 + (column + 1) * lda] = -contact_forces[i].x();
      column += 2;
    }
    row = 3;
    column = 0;
    for (int i = 0; i < num_contacts; ++i)
    {
      A[(row + 0) + (column + 0) * lda] = w2;
      A[(row + 1) + (column + 1) * lda] = w2;
      row += 2;
      column += 2;
    }

    // set b
    b[0] = -wrench.torque.x();
    b[1] = -wrench.torque.y();
    b[2] = -wrench.torque.z();
    for (int i = 0; i < num_contacts; ++i)
    {
      const KDL::Vector& p = contact_parent_frames[i].p;
      const KDL::Vector& f = contact_forces[i];
      const KDL::Vector torque = p * f;
      b[0] -= torque.x();
      b[1] -= torque.y();
      b[2] -= torque.z();

      b[0] += dz * f.y();
      b[1] += dz * f.x();
    }
    row = 3;
    for (int i = 0; i < num_contacts; ++i)
    {
      b[row] = 0;
      b[row + 1] = 0;
      row += 2;
    }

    // solve
    dgels_(&trans, &m, &n, &nrhs, A, &lda, b, &ldb, work, &lwork, &info);

    row = 0;
    for (int i = 0; i < num_contacts; ++i)
    {
      const KDL::Rotation& rot = contact_parent_frames[i].M;
      KDL::Vector diff = KDL::Vector(b[row], b[row + 1], dz);
      diff.x(min(diff.x(), CONTACT_MAX_RIGHT));
      diff.x(max(diff.x(), CONTACT_MIN_RIGHT));
      diff.y(min(diff.y(), CONTACT_MAX_DIR));
      diff.y(max(diff.y(), CONTACT_MIN_DIR));
      diff = rot * diff;
      contact_positions[i] = contact_parent_frames[i].p + diff;
      row += 2;
    }
  }
}

}
;
