/********************************************************************************
Copyright (c) 2015, TRACLabs, Inc.
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

#include <pin_ik/nlopt_ik.hpp>
#include <limits>
#include <pin_ik/dual_quaternion.h>
#include <cmath>
#include <cfloat>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace NLOPT_IK
{

dual_quaternion targetDQ;

double minfunc(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
  NLOPT_IK *c = (NLOPT_IK *) data;
  return c->minJoints(x, grad);
}

double minfuncDQ(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
  NLOPT_IK *c = (NLOPT_IK *) data;

  std::vector<double> vals(x);

  double jump = FLT_EPSILON;
  double result[1];
  c->cartDQError(vals, result);

  if (!grad.empty())
  {
    double v1[1];
    for (uint i = 0; i < x.size(); i++)
    {
      double original = vals[i];

      vals[i] = original + jump;
      c->cartDQError(vals, v1);

      vals[i] = original;
      grad[i] = (v1[0] - result[0]) / (2 * jump);
    }
  }

  return result[0];
}

double minfuncSumSquared(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
  NLOPT_IK *c = (NLOPT_IK *) data;

  std::vector<double> vals(x);

  double jump = FLT_EPSILON;
  double result[1];
  c->cartSumSquaredError(vals, result);

  if (!grad.empty())
  {
    double v1[1];
    for (uint i = 0; i < x.size(); i++)
    {
      double original = vals[i];

      vals[i] = original + jump;
      c->cartSumSquaredError(vals, v1);

      vals[i] = original;
      grad[i] = (v1[0] - result[0]) / (2.0 * jump);
    }
  }

  return result[0];
}

double minfuncL2(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
  NLOPT_IK *c = (NLOPT_IK *) data;

  std::vector<double> vals(x);

  double jump = FLT_EPSILON;
  double result[1];
  c->cartL2NormError(vals, result);

  if (!grad.empty())
  {
    double v1[1];
    for (uint i = 0; i < x.size(); i++)
    {
      double original = vals[i];

      vals[i] = original + jump;
      c->cartL2NormError(vals, v1);

      vals[i] = original;
      grad[i] = (v1[0] - result[0]) / (2.0 * jump);
    }
  }

  return result[0];
}

void constrainfuncm(uint m, double* result, uint n, const double* x, double* grad, void* data)
{
  NLOPT_IK *c = (NLOPT_IK *) data;

  std::vector<double> vals(n);

  for (uint i = 0; i < n; i++)
  {
    vals[i] = x[i];
  }

  double jump = FLT_EPSILON;

  c->cartSumSquaredError(vals, result);

  if (grad != NULL)
  {
    std::vector<double> v1(m);
    for (uint i = 0; i < n; i++)
    {
      double o = vals[i];
      vals[i] = o + jump;
      c->cartSumSquaredError(vals, v1.data());
      vals[i] = o;
      for (uint j = 0; j < m; j++)
      {
        grad[j * n + i] = (v1[j] - result[j]) / (2 * jump);
      }
    }
  }
}

NLOPT_IK::NLOPT_IK(const pinocchio::Model& _model, const Eigen::VectorXd& _q_min,
                   const Eigen::VectorXd& _q_max, pinocchio::FrameIndex _tip_frame_id,
                   double _maxtime, double _eps, OptType _type):
  model(_model), tip_frame_id(_tip_frame_id), maxtime(_maxtime), eps(std::abs(_eps)), TYPE(_type)
{
  PIN_IK::validateLimits(model, _q_min, _q_max);
  if (tip_frame_id >= model.frames.size()) throw std::invalid_argument("Invalid tip frame");

  data.reset(new pinocchio::Data(model));
  reset();

  opt = nlopt::opt(nlopt::LD_SLSQP, model.nv);

  for (int i = 0; i < model.nv; i++)
  {
    lb.push_back(_q_min(i));
    ub.push_back(_q_max(i));
  }

  types = PIN_IK::jointTypes(model);

  std::vector<double> tolerance(1, FLT_EPSILON);
  opt.set_xtol_abs(tolerance[0]);

  switch (TYPE)
  {
  case Joint:
    opt.set_min_objective(minfunc, this);
    opt.add_equality_mconstraint(constrainfuncm, this, tolerance);
    break;
  case DualQuat:
    opt.set_min_objective(minfuncDQ, this);
    break;
  case SumSq:
    opt.set_min_objective(minfuncSumSquared, this);
    break;
  case L2:
    opt.set_min_objective(minfuncL2, this);
    break;
  }
}

double NLOPT_IK::minJoints(const std::vector<double>& x, std::vector<double>& grad)
{
  bool gradient = !grad.empty();

  double err = 0;
  for (uint i = 0; i < x.size(); i++)
  {
    err += pow(x[i] - des[i], 2);
    if (gradient)
      grad[i] = 2.0 * (x[i] - des[i]);
  }

  return err;
}

void NLOPT_IK::cartSumSquaredError(const std::vector<double>& x, double error[])
{
  if (aborted || progress != -3)
  {
    opt.force_stop();
    return;
  }

  Eigen::VectorXd q(x.size());
  for (uint i = 0; i < x.size(); i++)
    q(i) = x[i];

  pinocchio::forwardKinematics(model, *data, PIN_IK::toPinocchioConfiguration(model, q));
  pinocchio::updateFramePlacements(model, *data);
  currentPose = data->oMf[tip_frame_id];

  if (std::isnan(currentPose.translation().x()))
  {
    std::fprintf(stderr, "NaNs from NLOpt!!");
    error[0] = std::numeric_limits<float>::max();
    progress = -1;
    return;
  }

  pinocchio::Motion delta_twist = PIN_IK::diffRelative(targetPose, currentPose);

  for (int i = 0; i < 3; i++)
  {
    if (std::abs(delta_twist.linear()[i]) <= std::abs(bounds.linear()[i]))
      delta_twist.linear()[i] = 0.0;
    if (std::abs(delta_twist.angular()[i]) <= std::abs(bounds.angular()[i]))
      delta_twist.angular()[i] = 0.0;
  }

  error[0] = delta_twist.linear().squaredNorm() + delta_twist.angular().squaredNorm();

  if (PIN_IK::isMotionZero(delta_twist, eps))
  {
    progress = 1;
    best_x = x;
    return;
  }
}

void NLOPT_IK::cartL2NormError(const std::vector<double>& x, double error[])
{
  if (aborted || progress != -3)
  {
    opt.force_stop();
    return;
  }

  Eigen::VectorXd q(x.size());
  for (uint i = 0; i < x.size(); i++)
    q(i) = x[i];

  pinocchio::forwardKinematics(model, *data, PIN_IK::toPinocchioConfiguration(model, q));
  pinocchio::updateFramePlacements(model, *data);
  currentPose = data->oMf[tip_frame_id];

  if (std::isnan(currentPose.translation().x()))
  {
    std::fprintf(stderr, "NaNs from NLOpt!!");
    error[0] = std::numeric_limits<float>::max();
    progress = -1;
    return;
  }

  pinocchio::Motion delta_twist = PIN_IK::diffRelative(targetPose, currentPose);

  for (int i = 0; i < 3; i++)
  {
    if (std::abs(delta_twist.linear()[i]) <= std::abs(bounds.linear()[i]))
      delta_twist.linear()[i] = 0.0;
    if (std::abs(delta_twist.angular()[i]) <= std::abs(bounds.angular()[i]))
      delta_twist.angular()[i] = 0.0;
  }

  error[0] = std::sqrt(delta_twist.linear().squaredNorm() + delta_twist.angular().squaredNorm());

  if (PIN_IK::isMotionZero(delta_twist, eps))
  {
    progress = 1;
    best_x = x;
    return;
  }
}

void NLOPT_IK::cartDQError(const std::vector<double>& x, double error[])
{
  if (aborted || progress != -3)
  {
    opt.force_stop();
    return;
  }

  Eigen::VectorXd q(x.size());
  for (uint i = 0; i < x.size(); i++)
    q(i) = x[i];

  // Normalize continuous joints to enforce unit circle constraint
  // TODO: Need to identify which joints are continuous from the model
  // For now, forward kinematics should be robust to small deviations

  pinocchio::forwardKinematics(model, *data, PIN_IK::toPinocchioConfiguration(model, q));
  pinocchio::updateFramePlacements(model, *data);
  currentPose = data->oMf[tip_frame_id];

  if (std::isnan(currentPose.translation().x()))
  {
    std::fprintf(stderr, "NaNs from NLOpt!!");
    error[0] = std::numeric_limits<float>::max();
    progress = -1;
    return;
  }

  pinocchio::Motion delta_twist = PIN_IK::diffRelative(targetPose, currentPose);

  for (int i = 0; i < 3; i++)
  {
    if (std::abs(delta_twist.linear()[i]) <= std::abs(bounds.linear()[i]))
      delta_twist.linear()[i] = 0.0;
    if (std::abs(delta_twist.angular()[i]) <= std::abs(bounds.angular()[i]))
      delta_twist.angular()[i] = 0.0;
  }

  // Convert SE3 to dual quaternion
  Eigen::Matrix3d R = currentPose.rotation();
  Eigen::Quaterniond currentQuaternion(R);
  Eigen::Vector3d currentTranslation = currentPose.translation();

  math3d::matrix3x3<double> currentRotationMatrix;
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      currentRotationMatrix(i, j) = R(i,j);

  math3d::quaternion<double> currentQuat = math3d::rot_matrix_to_quaternion<double>(currentRotationMatrix);
  math3d::point3d currentTrans;
  currentTrans.x = currentTranslation.x();
  currentTrans.y = currentTranslation.y();
  currentTrans.z = currentTranslation.z();

  dual_quaternion currentDQ = dual_quaternion::rigid_transformation(currentQuat, currentTrans);

  dual_quaternion errorDQ = (currentDQ * !targetDQ).normalize();
  errorDQ.log();
  error[0] = 4.0f * dot(errorDQ, errorDQ);

  if (PIN_IK::isMotionZero(delta_twist, eps))
  {
    progress = 1;
    best_x = x;
    return;
  }
}

int NLOPT_IK::CartToJnt(const Eigen::VectorXd &q_init, const pinocchio::SE3 &p_in,
                        Eigen::VectorXd &q_out, const pinocchio::Motion _bounds,
                        const Eigen::VectorXd& q_desired)
{
  auto start_time = std::chrono::steady_clock::now();

  bounds = _bounds;
  q_out = q_init;

  if (q_init.size() != types.size() || !q_init.allFinite())
  {
    std::fprintf(stderr, "IK seeded with wrong number of joints.  Expected %d but got %d",
                 (int)types.size(), (int)q_init.size());
    return -3;
  }

  opt.set_maxtime(maxtime);

  double minf;
  targetPose = p_in;

  if (TYPE == 1)   // DQ
  {
    Eigen::Matrix3d R = targetPose.rotation();
    math3d::matrix3x3<double> targetRotationMatrix;
    for(int i = 0; i < 3; i++)
      for(int j = 0; j < 3; j++)
        targetRotationMatrix(i, j) = R(i,j);

    math3d::quaternion<double> targetQuaternion = math3d::rot_matrix_to_quaternion<double>(targetRotationMatrix);
    math3d::point3d targetTranslation;
    targetTranslation.x = targetPose.translation().x();
    targetTranslation.y = targetPose.translation().y();
    targetTranslation.z = targetPose.translation().z();
    targetDQ = dual_quaternion::rigid_transformation(targetQuaternion, targetTranslation);
  }

  std::vector<double> x(model.nv);

  for (int i = 0; i < x.size(); i++)
  {
    x[i] = q_init(i);

    if (types[i] == PIN_IK::Continuous)
      continue;

    if (types[i] == PIN_IK::TransJoint)
    {
      x[i] = std::min(x[i], ub[i]);
      x[i] = std::max(x[i], lb[i]);
    }
    else
    {
      if (x[i] > ub[i])
      {
        double diffangle = fmod(x[i] - ub[i], 2 * M_PI);
        x[i] = ub[i] + diffangle - 2 * M_PI;
      }

      if (x[i] < lb[i])
      {
        double diffangle = fmod(lb[i] - x[i], 2 * M_PI);
        x[i] = lb[i] - diffangle + 2 * M_PI;
      }

      if (x[i] > ub[i])
        x[i] = (ub[i] + lb[i]) / 2.0;
    }
  }

  best_x = x;
  progress = -3;

  std::vector<double> artificial_lower_limits(lb.size());

  for (uint i = 0; i < lb.size(); i++)
    if (types[i] == PIN_IK::Continuous)
      artificial_lower_limits[i] = best_x[i] - 2 * M_PI;
    else if (types[i] == PIN_IK::TransJoint)
      artificial_lower_limits[i] = lb[i];
    else
      artificial_lower_limits[i] = std::max(lb[i], best_x[i] - 2 * M_PI);

  opt.set_lower_bounds(artificial_lower_limits);

  std::vector<double> artificial_upper_limits(lb.size());

  for (uint i = 0; i < ub.size(); i++)
    if (types[i] == PIN_IK::Continuous)
      artificial_upper_limits[i] = best_x[i] + 2 * M_PI;
    else if (types[i] == PIN_IK::TransJoint)
      artificial_upper_limits[i] = ub[i];
    else
      artificial_upper_limits[i] = std::min(ub[i], best_x[i] + 2 * M_PI);

  opt.set_upper_bounds(artificial_upper_limits);

  if (q_desired.size() == 0)
  {
    des = x;
  }
  else
  {
    des.resize(x.size());
    for (uint i = 0; i < des.size(); i++)
      des[i] = q_desired(i);
  }

  try
  {
    opt.optimize(x, minf);
  }
  catch (...)
  {
  }

  if (progress == -1)
    progress = -3;

  if (!aborted && progress < 0)
  {
    auto diff = std::chrono::steady_clock::now() - start_time;
    auto time_left = maxtime - std::chrono::duration<double>(diff).count();

    while (time_left > 0 && !aborted && progress < 0)
    {
      for (uint i = 0; i < x.size(); i++)
        x[i] = fRand(artificial_lower_limits[i], artificial_upper_limits[i]);

      opt.set_maxtime(time_left);

      try
      {
        opt.optimize(x, minf);
      }
      catch (...) {}

      if (progress == -1)
        progress = -3;

      auto diff = std::chrono::steady_clock::now() - start_time;
      time_left = maxtime - std::chrono::duration<double>(diff).count();
    }
  }

  for (uint i = 0; i < x.size(); i++)
  {
    q_out(i) = best_x[i];
  }

  return progress;
}

}
