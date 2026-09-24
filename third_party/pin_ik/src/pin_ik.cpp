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

#include <pin_ik/pin_ik.hpp>
#include <Eigen/Geometry>
#include <chrono>
#include <atomic>
#include <cstdio>
#include <iostream>
#include <limits>
#include <pin_ik/urdf.hpp>
#include <stdexcept>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace PIN_IK
{

PIN_IK::PIN_IK(const std::string& base, const std::string& tip, const std::string& xml,
                 double timeout, double tolerance, SolveType type) :
  initialized(false), eps(tolerance), maxtime(timeout), solvetype(type)
{
  loadURDFModel(xml, base, tip, model, lb, ub, tip_frame_id);
  data.reset(new pinocchio::Data(model));
  initialize();
}

PIN_IK::PIN_IK(const pinocchio::Model& _model, const Eigen::VectorXd& _q_min,
                 const Eigen::VectorXd& _q_max, pinocchio::FrameIndex _tip_frame_id,
                 double _maxtime, double _eps, SolveType _type):
  initialized(false),
  model(_model),
  tip_frame_id(_tip_frame_id),
  lb(_q_min),
  ub(_q_max),
  eps(_eps),
  maxtime(_maxtime),
  solvetype(_type)
{
  data.reset(new pinocchio::Data(model));
  initialize();
}

void PIN_IK::initialize()
{
  if (!std::isfinite(maxtime) || maxtime <= 0 || !std::isfinite(eps) || eps <= 0)
    throw std::invalid_argument("Timeout and epsilon must be finite and positive");

  if (model.nv == 0 || model.nv != lb.size() || lb.size() != ub.size())
    throw std::invalid_argument("Invalid model or joint limit dimensions");

  validateLimits(model, lb, ub);
  if (tip_frame_id >= model.frames.size())
    throw std::invalid_argument("Invalid tip frame");

  resetSolvers();

  types = ::PIN_IK::jointTypes(model);

  initialized = true;
}

bool PIN_IK::unique_solution(const Eigen::VectorXd& sol)
{
  for (uint i = 0; i < solutions.size(); i++)
    if (myEqual(sol, solutions[i]))
      return false;
  return true;
}

inline void normalizeAngle(double& val, const double& min, const double& max)
{
  if (val > max)
  {
    double diffangle = fmod(val - max, 2 * M_PI);
    val = max + diffangle - 2 * M_PI;
  }

  if (val < min)
  {
    double diffangle = fmod(min - val, 2 * M_PI);
    val = min - diffangle + 2 * M_PI;
  }
}

inline void normalizeAngle(double& val, const double& target)
{
  normalizeAngle(val, target - M_PI, target + M_PI);
}

template<typename T1, typename T2>
bool PIN_IK::runSolver(T1& solver, T2& other_solver,
                        const Eigen::VectorXd &q_init,
                        const pinocchio::SE3 &p_in)
{
  Eigen::VectorXd q_out;

  double fulltime = maxtime;
  Eigen::VectorXd seed = q_init;

  while (true)
  {
    auto timediff = std::chrono::steady_clock::now() - start_time;
    auto time_left = fulltime - std::chrono::duration<double>(timediff).count();

    if (time_left <= 0)
      break;

    solver.setMaxtime(time_left);

    int RC = solver.CartToJnt(seed, p_in, q_out, bounds);
    if (RC >= 0)
    {
      switch (solvetype)
      {
      case Manip1:
      case Manip2:
      case Manip3:
        normalize_limits(q_init, q_out);
        break;
      default:
        normalize_seed(q_init, q_out);
        break;
      }
      mtx_.lock();
      if (unique_solution(q_out))
      {
        solutions.push_back(q_out);
        uint curr_size = solutions.size();
        errors.resize(curr_size);
        double err, penalty, manip_value;
        switch (solvetype)
        {
        case Manip1:
          penalty = manipPenalty(q_out);
          manip_value = PIN_IK::manipValue1(q_out);
          err = penalty * manip_value;
          break;
        case Manip2:
          penalty = manipPenalty(q_out);
          manip_value = PIN_IK::manipValue2(q_out);
          err = penalty * manip_value;
          break;
        case Manip3:
          penalty = manipPenalty(q_out);
          manip_value = PIN_IK::manipValue3(q_out);
          err = penalty * manip_value;
          break;
        default:
          err = PIN_IK::JointErr(q_init, q_out);
          break;
        }
        errors[curr_size - 1] = std::make_pair(err, curr_size - 1);
      }
      mtx_.unlock();
    }

    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!solutions.empty() && solvetype == Speed) break;
    }

    for (int j = 0; j < seed.size(); j++)
      if (types[j] == Continuous)
        seed(j) = fRand(q_init(j) - 2 * M_PI, q_init(j) + 2 * M_PI);
      else
        seed(j) = fRand(lb(j), ub(j));
  }
  other_solver.abort();

  solver.setMaxtime(fulltime);

  return true;
}

void PIN_IK::normalize_seed(const Eigen::VectorXd& seed, Eigen::VectorXd& solution)
{
  for (int i = 0; i < lb.size(); i++)
  {
    if (types[i] == TransJoint)
      continue;

    double target = seed(i);
    double val = solution(i);

    normalizeAngle(val, target);

    if (types[i] == Continuous)
    {
      solution(i) = val;
      continue;
    }

    normalizeAngle(val, lb(i), ub(i));

    solution(i) = val;
  }
}

void PIN_IK::normalize_limits(const Eigen::VectorXd& seed, Eigen::VectorXd& solution)
{
  for (int i = 0; i < lb.size(); i++)
  {
    if (types[i] == TransJoint)
      continue;

    double target = seed(i);

    if (types[i] == RotJoint && types[i] != Continuous)
      target = (ub(i) + lb(i)) / 2.0;

    double val = solution(i);

    normalizeAngle(val, target);

    if (types[i] == Continuous)
    {
      solution(i) = val;
      continue;
    }

    normalizeAngle(val, lb(i), ub(i));

    solution(i) = val;
  }
}

double PIN_IK::manipPenalty(const Eigen::VectorXd& arr)
{
  double penalty = 1.0;
  for (int i = 0; i < arr.size(); i++)
  {
    if (types[i] == Continuous)
      continue;
    double range = ub(i) - lb(i);
    penalty *= ((arr(i) - lb(i)) * (ub(i) - arr(i)) / (range * range));
  }
  return std::max(0.0, 1.0 - exp(-1 * penalty));
}

double PIN_IK::manipValue1(const Eigen::VectorXd& arr)
{
  Eigen::VectorXd singular_values = computeSingularValues(arr);

  double error = 1.0;
  for (int i = 0; i < singular_values.size(); ++i)
    error *= singular_values(i);
  return error;
}

double PIN_IK::manipValue2(const Eigen::VectorXd& arr)
{
  Eigen::VectorXd singular_values = computeSingularValues(arr);
  return singular_values.minCoeff() / singular_values.maxCoeff();
}

double PIN_IK::manipValue3(const Eigen::VectorXd& arr)
{
  Eigen::VectorXd singular_values = computeSingularValues(arr);
  return singular_values.minCoeff();
}

Eigen::VectorXd PIN_IK::computeSingularValues(const Eigen::VectorXd& arr)
{
  pinocchio::Data::Matrix6x J(6, model.nv);
  J.setZero();

  pinocchio::forwardKinematics(model, *data, ::PIN_IK::toPinocchioConfiguration(model, arr));
  pinocchio::computeFrameJacobian(model, *data, ::PIN_IK::toPinocchioConfiguration(model, arr), tip_frame_id,
                                   pinocchio::LOCAL_WORLD_ALIGNED, J);

  Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(J);
  return svdsolver.singularValues();
}

int PIN_IK::CartToJnt(const Eigen::VectorXd &q_init, const pinocchio::SE3 &p_in,
                       Eigen::VectorXd &q_out, const pinocchio::Motion& _bounds)
{
  if (q_init.size() != model.nv || !q_init.allFinite()) return -1;

  if (!initialized)
  {
    std::fprintf(stderr, "PIN-IK was not properly initialized with a valid model or limits.  IK cannot proceed");
    return -1;
  }

  start_time = std::chrono::steady_clock::now();

  nl_solver->reset();
  iksolver->reset();

  solutions.clear();
  errors.clear();

  bounds = _bounds;

  task1 = std::thread(&PIN_IK::runPinocchioIK, this, q_init, p_in);
  task2 = std::thread(&PIN_IK::runNLOPT, this, q_init, p_in);

  if (task1.joinable())
      task1.join();
  if (task2.joinable())
      task2.join();

  if (solutions.empty())
  {
    q_out = q_init;
    return -3;
  }

  switch (solvetype)
  {
  case Manip1:
  case Manip2:
  case Manip3:
    std::sort(errors.rbegin(), errors.rend());
    break;
  default:
    std::sort(errors.begin(), errors.end());
    break;
  }

  q_out = solutions[errors[0].second];

  return solutions.size();
}

PIN_IK::~PIN_IK()
{
  if (initialized)
  {
    iksolver->abort();
    nl_solver->abort();
  }
  if (task1.joinable())
    task1.join();
  if (task2.joinable())
    task2.join();
}

}
