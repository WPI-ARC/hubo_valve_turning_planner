/*
 * (C) Copyright 2013 WPI-ARC (http://arc.wpi.edu) and others.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser General Public License
 * (LGPL) version 2.1 which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/lgpl-2.1.html
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * Contributors:
 *      Matt Zucker, Jim Mainprice
 */

#ifndef DRCHUBO_ANALYTICAL_IK_H
#define DRCHUBO_ANALYTICAL_IK_H

#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <complex>

namespace DrcHuboAnalyticalIK
{
typedef Eigen::Matrix< double, 6, 1 > Vector6d;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Isometry3d Isometry3d;
typedef Eigen::Matrix< double, 6, 2 > Matrix62d;
typedef std::vector<int> IntArray;

enum
{
    SIDE_RIGHT = 0,
    SIDE_LEFT = 1
};

enum IkFlags
{
    IK_PREFER_CLOSEST_ANGLES = 0x01, // as opposed to closest FK position
    IK_IGNORE_LIMITS         = 0x02
};

class HuboKin
{
public:

    struct KinConstants
    {
        double arm_l0, arm_l1, arm_l2, arm_l3, arm_l4, arm_l5;

        Matrix62d arm_limits;
        Vector6d  arm_offset;
        IntArray arm_mirror;

        KinConstants();

        Matrix62d getArmLimits(int side) const;
        Matrix62d getLegLimits(int side) const;
        Vector6d  getArmOffset(int side) const;
        Vector6d  getLegOffset(int side) const;
    };

    KinConstants kc;

    static Matrix62d mirrorLimits(const Matrix62d& orig, const IntArray& mirror);
    static Vector6d  mirrorAngles(const Vector6d& orig, const IntArray& mirror);

    void armFK(Isometry3d &B, const Vector6d &q, int side) const;
    void armFK(Isometry3d &B, const Vector6d &q, int side, const Isometry3d &endEffector) const;

    void armIK(Vector6d &q, const Isometry3d& B, const Vector6d& qPrev, int side, int flags) const;
    void armIK(Vector6d &q, const Isometry3d& B, const Vector6d& qPrev, int side, int flags, const Isometry3d &endEffector) const;

    //! Arm IK main function
    //! @return solutions : vector of 8 IK solutions
    //! @param svalid : what solutions are valid
    //! @param B : B is the Frame to obtain
    //! @param qPrev : robot arm configuration
    //! @param side : SIDE_RIGHT or SIDE_LEFT
    //! @param flag : ?
    int armIK(Vector6d solutions[8], bool valid[8], const Isometry3d& B, const Vector6d& qPrev, int side, int flags) const;
    int armIK(Vector6d solutions[8], bool valid[8], const Isometry3d& B, const Vector6d& qPrev, int side, int flags, const Isometry3d &endEffector) const;

    int fixAndFindBest(Vector6d solutions[8], const bool valid[8], const Isometry3d& B, const Vector6d& qprev, const Matrix62d& limits, int side, int flags, bool isLeg) const;
};
}

#endif // ANALYTICAL_IK_H

