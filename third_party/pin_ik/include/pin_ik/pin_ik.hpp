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

#ifndef PIN_IK_HPP
#define PIN_IK_HPP

#include <pin_ik/nlopt_ik.hpp>
#include <pin_ik/pinocchio_tl.hpp>
#include <chrono>
#include <atomic>
#include <cstdio>
#include <iostream>
#include <thread>
#include <stdexcept>
#include <mutex>
#include <memory>

namespace PIN_IK
{

enum SolveType { Speed, Distance, Manip1, Manip2, Manip3 };

class PIN_IK
{
public:
  PIN_IK(const pinocchio::Model& _model, const Eigen::VectorXd& _q_min,
          const Eigen::VectorXd& _q_max, pinocchio::FrameIndex _tip_frame_id,
          double _maxtime = 0.005, double _eps = 1e-5, SolveType _type = Speed);

  PIN_IK(const std::string& base, const std::string& tip, const std::string& urdf_xml,
          double timeout = 0.005, double epsilon = 1e-5, SolveType type = Speed);

  ~PIN_IK();

  bool getModel(pinocchio::Model& model_)
  {
    model_ = model;
    return initialized;
  }

  bool getLimits(Eigen::VectorXd& lb_, Eigen::VectorXd& ub_)
  {
    lb_ = lb;
    ub_ = ub;
    return initialized;
  }

  bool getSolutions(std::vector<Eigen::VectorXd>& solutions_)
  {
    solutions_ = solutions;
    return initialized && !solutions.empty();
  }

  bool getSolutions(std::vector<Eigen::VectorXd>& solutions_, std::vector<std::pair<double, uint> >& errors_)
  {
    errors_ = errors;
    return getSolutions(solutions_);
  }

  bool setLimits(Eigen::VectorXd& lb_, Eigen::VectorXd& ub_)
  {
    validateLimits(model, lb_, ub_);
    lb = lb_;
    ub = ub_;
    types.clear();
    initialize();
    return true;
  }

  static double JointErr(const Eigen::VectorXd& arr1, const Eigen::VectorXd& arr2)
  {
    double err = 0;
    for (int i = 0; i < arr1.size(); i++)
    {
      err += pow(arr1(i) - arr2(i), 2);
    }
    return err;
  }

  // Seed/output and limits use model.nv scalars; continuous joints use radians.
  int CartToJnt(const Eigen::VectorXd &q_init, const pinocchio::SE3 &p_in, Eigen::VectorXd &q_out,
                const pinocchio::Motion& bounds = pinocchio::Motion::Zero());

  inline void SetSolveType(SolveType _type)
  {
    solvetype = _type;
  }

private:
  bool initialized;
  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;
  pinocchio::FrameIndex tip_frame_id;
  Eigen::VectorXd lb, ub;
  double eps;
  double maxtime;
  SolveType solvetype;

  std::unique_ptr<NLOPT_IK::NLOPT_IK> nl_solver;
  std::unique_ptr<ChainIkSolverPos_TL> iksolver;

  std::chrono::steady_clock::time_point start_time;

  template<typename T1, typename T2>
  bool runSolver(T1& solver, T2& other_solver,
                 const Eigen::VectorXd &q_init,
                 const pinocchio::SE3 &p_in);

  bool runPinocchioIK(const Eigen::VectorXd &q_init, const pinocchio::SE3 &p_in);
  bool runNLOPT(const Eigen::VectorXd &q_init, const pinocchio::SE3 &p_in);

  void normalize_seed(const Eigen::VectorXd& seed, Eigen::VectorXd& solution);
  void normalize_limits(const Eigen::VectorXd& seed, Eigen::VectorXd& solution);

  std::vector<BasicJointType> types;

  std::mutex mtx_;
  std::vector<Eigen::VectorXd> solutions;
  std::vector<std::pair<double, uint> >  errors;

  std::thread task1, task2;
  pinocchio::Motion bounds;

  bool unique_solution(const Eigen::VectorXd& sol);

  inline static double fRand(double min, double max)
  {
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
  }

  double manipPenalty(const Eigen::VectorXd& arr);
  double manipValue1(const Eigen::VectorXd& arr);
  double manipValue2(const Eigen::VectorXd& arr);
  double manipValue3(const Eigen::VectorXd& arr);

  Eigen::VectorXd computeSingularValues(const Eigen::VectorXd& arr);

  inline bool myEqual(const Eigen::VectorXd& a, const Eigen::VectorXd& b, const double eps=1e-4)
  {
    return (a - b).isZero(eps);
  }

  void initialize();

  void resetSolvers()
  {
    nl_solver.reset(new NLOPT_IK::NLOPT_IK(model, lb, ub, tip_frame_id, maxtime, eps, NLOPT_IK::SumSq));
    iksolver.reset(new ChainIkSolverPos_TL(model, lb, ub, tip_frame_id, maxtime, eps, true, true));
  }
};

inline bool PIN_IK::runPinocchioIK(const Eigen::VectorXd &q_init, const pinocchio::SE3 &p_in)
{
  return runSolver(*iksolver.get(), *nl_solver.get(), q_init, p_in);
}

inline bool PIN_IK::runNLOPT(const Eigen::VectorXd &q_init, const pinocchio::SE3 &p_in)
{
  return runSolver(*nl_solver.get(), *iksolver.get(), q_init, p_in);
}

}

#endif
