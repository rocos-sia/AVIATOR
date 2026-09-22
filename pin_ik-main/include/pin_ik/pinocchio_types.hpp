/********************************************************************************
Copyright (c) 2024, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#ifndef PINOCCHIO_TYPES_HPP
#define PINOCCHIO_TYPES_HPP

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <stdexcept>
#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace PIN_IK {

// Joint type enumeration for limit handling
enum BasicJointType {
  RotJoint = 0,
  TransJoint = 1,
  Continuous = 2
};

// Type aliases for Pinocchio 3.9.0
// Default instantiation: double scalar, column-major (Options=0), default joint collection
using Model = pinocchio::ModelTpl<double, 0, pinocchio::JointCollectionDefaultTpl>;
using Data = pinocchio::DataTpl<double, 0, pinocchio::JointCollectionDefaultTpl>;
using SE3 = pinocchio::SE3Tpl<double, 0>;
using Motion = pinocchio::MotionTpl<double, 0>;
using Frame = pinocchio::FrameTpl<double, 0>;
using FrameIndex = pinocchio::FrameIndex;
using JointIndex = pinocchio::JointIndex;

// IK uses one scalar per joint (nv); Pinocchio uses (cos(theta), sin(theta))
// for unbounded revolute joints. Multi-DOF joints are deliberately unsupported.
inline bool isContinuous(const pinocchio::JointModel& joint) {
  return joint.nv() == 1 && joint.nq() == 2 &&
    (joint.shortname().find("RevoluteUnbounded") != std::string::npos ||
     joint.shortname().find("JointModelRUB") == 0);
}

inline std::vector<BasicJointType> jointTypes(const Model& model) {
  std::vector<BasicJointType> result(model.nv);
  for (JointIndex i = 1; i < model.joints.size(); ++i) {
    const auto& j = model.joints[i];
    const auto name = j.shortname();
    // Aligned continuous joints use JointModelRUBX/Y/Z short names.
    const bool continuous = isContinuous(j);
    const bool prismatic = name.find("JointModelP") == 0;
    const bool revolute = name.find("JointModelR") == 0;
    if (j.nv() != 1 || (!continuous && j.nq() != 1) || (!prismatic && !revolute))
      throw std::invalid_argument("Only revolute, continuous and prismatic joints are supported");
    result[j.idx_v()] = continuous ? Continuous : (prismatic ? TransJoint : RotJoint);
  }
  return result;
}

inline Eigen::VectorXd toPinocchioConfiguration(const Model& model, const Eigen::VectorXd& angles) {
  if (angles.size() != model.nv || !angles.allFinite())
    throw std::invalid_argument("Expected finite joint values of size model.nv");
  Eigen::VectorXd q(model.nq);
  for (JointIndex i = 1; i < model.joints.size(); ++i) {
    const auto& j = model.joints[i];
    if (j.nv() != 1 || (j.nq() != 1 && j.nq() != 2))
      throw std::invalid_argument("Unsupported joint configuration");
    const double angle = angles[j.idx_v()];
    if (j.nq() == 2) {
      q[j.idx_q()] = std::cos(angle);
      q[j.idx_q() + 1] = std::sin(angle);
    } else q[j.idx_q()] = angle;
  }
  return q;
}

inline void validateLimits(const Model& model, const Eigen::VectorXd& lo, const Eigen::VectorXd& hi) {
  const auto types = jointTypes(model);
  if (lo.size() != model.nv || hi.size() != model.nv)
    throw std::invalid_argument("Wrong joint limit dimensions (expected model.nv)");
  for (int i = 0; i < model.nv; ++i) {
    if (types[i] == Continuous) {
      if (lo[i] != -std::numeric_limits<double>::infinity() ||
          hi[i] != std::numeric_limits<double>::infinity())
        throw std::invalid_argument("Continuous joint limits must be (-inf, +inf)");
    } else if (!std::isfinite(lo[i]) || !std::isfinite(hi[i]) || lo[i] > hi[i])
      throw std::invalid_argument("Invalid joint limits");
  }
}

// Helper function to compute pose difference for IK
inline Motion diffRelative(const SE3& current, const SE3& target) {
  // Compute the relative transformation error
  SE3 error = target.actInv(current);
  // Convert to motion (log map)
  Motion motion_error = pinocchio::log6(error);
  return motion_error;
}

// Helper function to check if two SE3 transforms are approximately equal
inline bool isApprox(const SE3& a, const SE3& b, double prec = 1e-6) {
  return a.isApprox(b, prec);
}

// Helper function to check if a Motion is approximately zero
inline bool isMotionZero(const Motion& m, double eps = 1e-6) {
  return m.linear().norm() < eps && m.angular().norm() < eps;
}

} // namespace PIN_IK

#endif // PINOCCHIO_TYPES_HPP
