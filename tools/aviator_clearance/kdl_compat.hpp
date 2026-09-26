// KDL-shaped math shim over Eigen + Pinocchio.
//
// clearance_trajectory.cpp was written against the KDL / kdl_parser / TRAC-IK
// API.  The dev branch dropped KDL, kdl_parser and TRAC-IK in favour of
// Pinocchio + PIN-IK.  Rather than rewriting every math call site in the port,
// this header re-provides the small KDL surface the tool actually uses
// (Vector / Rotation / Frame / JntArray) as thin wrappers over Eigen and
// pinocchio::SE3.  The only semantic change versus KDL is the IK backend
// itself, which is swapped in clearance_trajectory.cpp.
//
// This is a local compatibility header, NOT a re-vendoring of KDL: it depends
// only on Eigen (already a system dependency) and Pinocchio (vendored), and it
// carries no kdl_parser / orocos-kdl symbols.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/spatial/se3.hpp>

namespace KDL {

// JntArray is an Eigen column vector (size 7 here); elements are indexed with
// operator()(i), exactly as the original KDL::JntArray API.
using JntArray = Eigen::VectorXd;

// --- Vector -----------------------------------------------------------------
// Thin wrapper over Eigen::Vector3d exposing the subset of the KDL::Vector API
// used by the tool: {x,y,z} construction, component access, normalize-in-place,
// and the +, -, scalar*, scalar/ operators.  (No dot/cross is used.)
class Vector {
  public:
    Eigen::Vector3d v{Eigen::Vector3d::Zero()};

    Vector() = default;
    Vector(double x, double y, double z) : v(x, y, z) {}
    Vector(const Eigen::Vector3d &e) : v(e) {}

    double x() const { return v.x(); }
    double y() const { return v.y(); }
    double z() const { return v.z(); }

    void Normalize() { v.normalize(); }

    Vector operator+(const Vector &o) const { return v + o.v; }
    Vector operator-(const Vector &o) const { return v - o.v; }
    Vector operator*(double s) const { return v * s; }
    Vector operator/(double s) const { return v / s; }
    friend Vector operator*(double s, const Vector &a) { return a.v * s; }
};

// --- Rotation ---------------------------------------------------------------
// Thin wrapper over Eigen::Matrix3d exposing the subset of the KDL::Rotation
// API used by the tool: 9-arg and static construction (Rot / RotZ /
// Quaternion), composition (*), inverse, and axis-angle extraction (GetRot).
class Rotation {
  public:
    Eigen::Matrix3d m{Eigen::Matrix3d::Identity()};

    Rotation() = default;
    Rotation(const Eigen::Matrix3d &M) : m(M) {}
    Rotation(double r00, double r01, double r02,
             double r10, double r11, double r12,
             double r20, double r21, double r22) {
        m << r00, r01, r02,
             r10, r11, r12,
             r20, r21, r22;
    }

    static Rotation Rot(const Vector &axis, double angle) {
        return Rotation(Eigen::AngleAxisd(angle, axis.v.normalized()).toRotationMatrix());
    }
    static Rotation RotZ(double angle) {
        return Rotation(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix());
    }
    // KDL stores the quaternion as (x, y, z, w); Eigen stores (w, x, y, z).
    static Rotation Quaternion(double x, double y, double z, double w) {
        return Rotation(Eigen::Quaterniond(w, x, y, z).toRotationMatrix());
    }

    Rotation Inverse() const { return Rotation(m.transpose()); }
    Rotation operator*(const Rotation &o) const { return Rotation(m * o.m); }

    // KDL::Rotation::GetRot() returns the axis-angle vector (angle * axis).
    Vector GetRot() const {
        Eigen::AngleAxisd aa(m);
        return Vector(aa.angle() * aa.axis());
    }
};

// --- Frame ------------------------------------------------------------------
// Thin wrapper over pinocchio::SE3 exposing the KDL::Frame API used by the tool:
// {Rotation, Vector} construction, .M / .p accessors, composition (*) and
// inverse.  It converts implicitly to pinocchio::SE3 so it can be handed
// directly to PIN_IK::CartToJnt.
class Frame {
  public:
    pinocchio::SE3 t{pinocchio::SE3::Identity()};

    Frame() = default;
    Frame(const pinocchio::SE3 &s) : t(s) {}
    Frame(const Rotation &R, const Vector &p) : t(R.m, p.v) {}

    Rotation M() const { return Rotation(t.rotation()); }
    Vector p() const { return Vector(t.translation()); }

    Frame Inverse() const { return Frame(t.inverse()); }
    Frame operator*(const Frame &o) const { return Frame(t * o.t); }

    operator const pinocchio::SE3 &() const { return t; }
};

} // namespace KDL
