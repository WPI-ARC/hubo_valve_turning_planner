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

#include "drchubo_analytical_ik_solver.hpp"

#include <iostream>

using namespace std;
using namespace DrcHuboAnalyticalIK;

enum ArmJointIndex {
    SP=0,
    SR=1,
    SY=2,
    EP=3,
    WY=4,
    WP=5
};

enum LegJointIndex {
    HY=0,
    HR=1,
    HP=2,
    KP=3,
    AP=4,
    AR=5,
};

typedef Eigen::AngleAxisd AngleAxisd;
typedef Eigen::Matrix3d Matrix3d;
typedef Eigen::Quaterniond Quaterniond;

static inline Isometry3d rx(double theta) {
    return Isometry3d( AngleAxisd( theta, Vector3d(1,0,0) ) );
}

static inline Isometry3d ry(double theta) {
    return Isometry3d( AngleAxisd( theta, Vector3d(0,1,0) ) );
}

static inline Isometry3d rz(double theta) {
    return Isometry3d( AngleAxisd( theta, Vector3d(0,0,1) ) );
}

static inline Isometry3d xlate(const Vector3d& v) {
    Isometry3d rval = Isometry3d::Identity();
    return rval.translate(v);
}

static inline bool checkDist(Vector3d& p, double a, double b)
{
    double d = p.norm();
    double dmax = a+b;
    double dmin = fabs(a-b);

    if (d > dmax) {
        p *= dmax/d;
        return false;
    } else if (d < dmin) {
        p *= dmin/d;
        return false;
    } else {
        return true;
    }
}

static inline void clamp_sincos(double& sincos, bool& valid)
{
    if (sincos < -1) {
        valid = false;
        sincos = -1;
    } else if (sincos > 1) {
        valid = false;
        sincos = 1;
    }
}

static inline Vector3d flipEuler3Axis(const Vector3d& u) {
    Vector3d v;
    v[0] = u[0] - M_PI;
    v[1] = M_PI - u[1];
    v[2] = u[2] - M_PI;
    return v;
}

static const double zeroSize = 1e-9;

HuboKin::KinConstants::KinConstants()
{
    leg_l1 = 0;         // base -> hip X
    leg_l2 = 0.0885;    // base -> hip Y
    leg_l3 = 0.164;     // -(base -> hip Z)
    leg_l4 = 0.3299;    // hip -> knee Z
    leg_l5 = 0.33;      // knee -> ankle Z
    leg_l6 = 0.137;     // ankle to foot Z

    arm_l0 = 0.001;     // base -> shoulder X
    arm_l1 = 0.2061;    // base -> shoulder Z
    arm_l2 = 0.2295;    // base -> shoulder Y
    arm_l3 = 0.3;       // shoulder -> elbow Z
    arm_l4 = 0.03;      // elbow offset X
    arm_l5 = 0.3138;    // elbow -> wrist Z

    leg_limits <<
            -1.91986, 1.91986,
            -0.520108, 0.520108,
            -1.86925, 1.60919,
            -0.0698132, 2.60927,
            -1.69995, 1.69995,
            -1.5708, 1.5708;

    arm_limits <<
            -3.13985, 3.13985,
            -3.13985, 0.261799,
            -3.13985, 2.00015,
            -2.96008, 0.200713,
            -3.13985, 3.13985,
            -3.13985, 6.28144;

    arm_mirror.push_back(SR);
    arm_mirror.push_back(SY);
    arm_mirror.push_back(WY);

    leg_mirror.push_back(HY);
    leg_mirror.push_back(HR);
    leg_mirror.push_back(AR);

    arm_offset.setZero();
    leg_offset.setZero();
}

Matrix62d HuboKin::mirrorLimits(const Matrix62d& orig, const IntArray& mirror)
{
    Matrix62d limits = orig;

    for (size_t i=0; i<mirror.size(); ++i) {
        int ax = mirror[i];
        double& lo = limits(ax,0);
        double& hi = limits(ax,1);
        swap(lo,hi);
        lo *= -1;
        hi *= -1;
    }

    return limits;
}

Vector6d HuboKin::mirrorAngles(const Vector6d& orig, const IntArray& mirror)
{
    Vector6d angles = orig;

    for (size_t i=0; i<mirror.size(); ++i) {
        int ax = mirror[i];
        angles[ax] *= -1;
    }

    return angles;
}

Matrix62d HuboKin::KinConstants::getArmLimits(int side) const
{
    if (side == SIDE_RIGHT) {
        return arm_limits;
    } else {
        return HuboKin::mirrorLimits(arm_limits, arm_mirror);
    }
}

Matrix62d HuboKin::KinConstants::getLegLimits(int side) const
{
    if (side == SIDE_RIGHT) {
        return leg_limits;
    } else {
        return HuboKin::mirrorLimits(leg_limits, leg_mirror);
    }
}

Vector6d HuboKin::KinConstants::getArmOffset(int side) const
{
    if (side == SIDE_RIGHT) {
        return arm_offset;
    } else {
        return HuboKin::mirrorAngles(arm_offset, arm_mirror);
    }
}

Vector6d HuboKin::KinConstants::getLegOffset(int side) const
{
    if (side == SIDE_RIGHT) {
        return leg_offset;
    } else {
        return HuboKin::mirrorAngles(leg_offset, leg_mirror);
    }
}

void HuboKin::armFK( Isometry3d &B, const Vector6d &q, int side ) const
{
    const double& l0 = kc.arm_l0;
    const double& l1 = kc.arm_l1;
    const double& l2 = (side == SIDE_LEFT) ? kc.arm_l2 : -kc.arm_l2;
    const double& l3 = kc.arm_l3;
    const double& l4 = kc.arm_l4;
    const double& l5 = kc.arm_l5;

    Vector6d qq = q + kc.getArmOffset(side);

    B = ( xlate(Vector3d(l0, l2, l1)) *
          ry(qq[SP]) *
          rx(qq[SR]) *
          rz(qq[SY]) *
          xlate(Vector3d(l4,0,-l3)) *
          ry(qq[EP]) *
          xlate(Vector3d(-l4,0,-l5)) *
          rz(qq[WY]) *
          ry(qq[WP]) );
}

void HuboKin::armFK( Isometry3d &B, const Vector6d &q, int side, const Isometry3d &endEffector ) const
{
    armFK( B, q, side );
    B = B * endEffector;
}

int HuboKin::armIK( Vector6d solutions[8], bool svalid[8], const Isometry3d& B, const Vector6d& qPrev, int side, int flags ) const
{
    const double& arm_l0 = kc.arm_l0;
    const double& arm_l1 = kc.arm_l1;
    const double& arm_l2 = (side == SIDE_LEFT) ? kc.arm_l2 : -kc.arm_l2;
    const double& arm_l3 = kc.arm_l3;
    const double& arm_l4 = kc.arm_l4;
    const double& arm_l5 = kc.arm_l5;

    Matrix62d limits = kc.getArmLimits(side);
    Vector6d offset = kc.getArmOffset(side);

    Isometry3d shoulder_from_wrist = xlate(Vector3d(-arm_l0, -arm_l2, -arm_l1)) * B;

    Vector3d p = shoulder_from_wrist.inverse().translation();

    size_t nsol = 0;

    const double a2 = arm_l5*arm_l5 + arm_l4*arm_l4;
    const double b2 = arm_l3*arm_l3 + arm_l4*arm_l4;
    const double a = sqrt(a2);
    const double b = sqrt(b2);

    const double alpha = atan2(arm_l5, arm_l4);
    const double beta = atan2(arm_l3, arm_l4);

    bool initValid = checkDist(p, a, b);

    double c2 = p.dot(p);
    double x = p.x();
    double y = p.y();
    double z = p.z();

    for (int flipEP=-1; flipEP<=1; flipEP+=2) {

        Vector6d qsoln;

        bool valid = initValid;

        // this fails if acos is outside its domain
        double cosGamma = (a2 + b2 - c2) / (2*a*b);
        clamp_sincos(cosGamma, valid);

        double gamma = flipEP * acos( cosGamma );
        double theta3 = alpha + beta + gamma - 2*M_PI;

        qsoln(EP) = theta3;

        double c3 = cos(theta3);
        double s3 = sin(theta3);

        // s2 gets double close to zero when elbow is near straight -
        // possible loss of precision
        double num = -y;
        double denom = (-arm_l4*c3 - arm_l3*s3 + arm_l4);

        for (int incWY=0; incWY<2; ++incWY) {

            double s2, theta2;

            if (fabs(denom) < zeroSize) {
                // possibly invalid since elbow straight, grab wrist yaw from orig
                valid = false;
                theta2 = incWY ? qPrev[WY] : M_PI - qPrev[WY];
                s2 = sin(theta2);
            } else {
                // possibly invalid if s2 outside its domain
                s2 = num / denom;
                clamp_sincos(s2, valid);
                theta2 = incWY ? M_PI - asin(s2) : asin(s2);
            }

            qsoln(WY) = theta2;

            double c2 = cos(theta2);

            double p =  arm_l4*c2 - arm_l4*c2*c3 - arm_l3*s3*c2;
            double q = -arm_l4*s3 + arm_l3*c3 + arm_l5;

            double det = -(q*q + p*p);

            // TODO: figure out what it means geometrically for det to be small here
            if (fabs(det) < zeroSize) { valid = false; }

            double k = det < 0 ? -1 : 1;

            double ks1 = k*( q*x - p*z);
            double kc1 = k*(-p*x - q*z);

            // atan2 should be robust (i.e. not deliver nan's) if det is
            // small, but that would mean the angle is not to be trusted
            double theta1 = atan2(ks1,kc1);
            qsoln(WP) = theta1;

            Quaterniond Rlower =
                    Quaterniond(AngleAxisd(qsoln(EP), Vector3d(0,1,0))) *
                    Quaterniond(AngleAxisd(qsoln(WY), Vector3d(0,0,1))) *
                    Quaterniond(AngleAxisd(qsoln(WP), Vector3d(0,1,0)));

            Matrix3d Rupper = B.rotation() * Rlower.inverse().matrix();

            Vector3d euler = Rupper.eulerAngles(1, 0, 2);

            for (int flipShoulder=0; flipShoulder<2; ++flipShoulder) {
                if (flipShoulder) { euler = flipEuler3Axis(euler); }
                qsoln(SP) = euler[0];
                qsoln(SR) = euler[1];
                qsoln(SY) = euler[2];
                solutions[nsol] = qsoln - offset;
                svalid[nsol] = valid;
                ++nsol;
            }
        }
    }

    bool is_leg = false;
    return fixAndFindBest(solutions, svalid, B, qPrev, limits, side, flags, is_leg);
}

void HuboKin::armIK( Vector6d& q, const Isometry3d& B, const Vector6d& qPrev, int side, int flags) const
{
    Vector6d solutions[8];
    bool valid[8];

    int best = armIK( solutions, valid, B, qPrev, side, flags );
    q = solutions[best];
}

void HuboKin::armIK( Vector6d &q, const Isometry3d& B, const Vector6d& qPrev, int side, int flags, const Isometry3d &endEffector) const
{
    Isometry3d B2 = B * endEffector.inverse();
    armIK( q, B2, qPrev, side, flags );
}

int HuboKin::armIK( Vector6d solutions[8], bool valid[8], const Isometry3d& B, const Vector6d& qPrev, int side, int flags, const Isometry3d &endEffector) const
{
    Isometry3d B2 = B * endEffector.inverse();
    return armIK( solutions, valid, B2, qPrev, side, flags );
}

#define debug if (0) std::cout

int HuboKin::fixAndFindBest( Vector6d solutions[8], const bool valid[8], const Isometry3d& B, const Vector6d& qPrev, const Matrix62d& limits, int side, int flags, bool isLeg) const
{
    typedef std::pair<double, double> DistancePair;

    bool best_within = false;
    int best_index = -1;
    DistancePair best_distance(0,0);

    debug << "qPrev = " << qPrev.transpose() << "\n\n";
    debug << "B = \n" << B.matrix() << "\n\n";

    for(int i=0; i<8; i++) {

        bool withinLim = true;

        double jointDist = 0;

        for(int j=0; j<6; j++) {

            double& qj = solutions[i](j);
            const double pj = qPrev(j);

            const double l = limits(j,0);
            const double u = limits(j,1);

            // Set infintessimal angles to zero
            if( qj < zeroSize && qj > -zeroSize) {
                qj = 0.0;
            }

            if (flags & IK_PREFER_CLOSEST_ANGLES) {
                // place within pi of previous
                while (qj > pj + M_PI) { qj -= 2*M_PI; }
                while (qj < pj - M_PI) { qj += 2*M_PI; }
            } else {
                // wrap to inside limits
                while (qj > u && qj - 2*M_PI > l) { qj -= 2*M_PI; }
                while (qj < l && qj + 2*M_PI < u) { qj += 2*M_PI; }
                // see if we can get closer
                while (qj > pj + M_PI && qj - 2*M_PI > l) { qj -= 2*M_PI; }
                while (qj < pj - M_PI && qj + 2*M_PI < u) { qj += 2*M_PI; }
            }

            // Check against limits and clamp.
            if ( !(flags & IK_IGNORE_LIMITS) && (qj < l || qj > u) ) {
                qj = std::max(l, std::min(qj, u));
                withinLim = false;
            }

            // Add to distance
            jointDist += fabs(qj - pj);
        }

        double fkDist;
        double quatDist;

        if (valid[i] && withinLim) {
            fkDist = 0;
            quatDist = 0;
        } else {
            Isometry3d B2;
            armFK(B2, solutions[i], side);
            fkDist = (B.translation() - B2.translation()).norm();
            if (fkDist < zeroSize) { fkDist = 0; }

            Quaterniond qB(B.rotation());
            Quaterniond qB2(B2.rotation());
            quatDist = qB.angularDistance(qB2);
        }

        debug << "got solution for "
              << (side == SIDE_RIGHT ? "RIGHT" :"LEFT") << " "
              << (isLeg ? "LEG" : "ARM") << " at "
              << solutions[i].transpose() << ", withinLim=" << withinLim << ", jointDist=" << jointDist << ", valid=" << valid[i] << ", fkDist=" << fkDist << ", quatDist = " << quatDist;

        if (jointDist < zeroSize) { jointDist = 0; }

        DistancePair d;

        if (flags & IK_PREFER_CLOSEST_ANGLES) {
            d = DistancePair(jointDist, fkDist);
        } else {
            d = DistancePair(fkDist, jointDist);
        }

        if (best_index < 0 ||
                (withinLim && !best_within) ||
                (withinLim == best_within && d < best_distance)) {
            best_index = i;
            best_distance = d;
            best_within = withinLim;
            debug << " <-- *** BEST SO FAR! ***";
        }
        debug << "\n";

    }

    debug << "\n";
    return best_index;
}
