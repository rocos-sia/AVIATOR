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

#ifndef NLOPT_IK_HPP
#define NLOPT_IK_HPP

#include <chrono>
#include <atomic>
#include <cstdio>
#include <iostream>
#include <pin_ik/pinocchio_types.hpp>
#include <nlopt.hpp>
#include <memory>

namespace PIN_IK {
  class PIN_IK;
}

namespace NLOPT_IK
{

enum OptType { Joint, DualQuat, SumSq, L2 };

class NLOPT_IK
{
  friend class PIN_IK::PIN_IK;
public:
  NLOPT_IK(const pinocchio::Model& _model, const Eigen::VectorXd& _q_min,
           const Eigen::VectorXd& _q_max, pinocchio::FrameIndex _tip_frame_id,
           double _maxtime = 0.005, double _eps = 1e-3, OptType _type = SumSq);

  ~NLOPT_IK() {};

  int CartToJnt(const Eigen::VectorXd& q_init, const pinocchio::SE3& p_in,
                Eigen::VectorXd& q_out, const pinocchio::Motion bounds = pinocchio::Motion::Zero(),
                const Eigen::VectorXd& q_desired = Eigen::VectorXd());

  double minJoints(const std::vector<double>& x, std::vector<double>& grad);
  void cartSumSquaredError(const std::vector<double>& x, double error[]);
  void cartDQError(const std::vector<double>& x, double error[]);
  void cartL2NormError(const std::vector<double>& x, double error[]);

  inline void setMaxtime(double t)
  {
    maxtime = t;
  }

private:

  inline void abort()
  {
    aborted = true;
  }

  inline void reset()
  {
    aborted = false;
  }

  std::vector<double> lb;
  std::vector<double> ub;

  const pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;
  pinocchio::FrameIndex tip_frame_id;
  std::vector<double> des;

  double maxtime;
  double eps;
  int iter_counter;
  OptType TYPE;

  pinocchio::SE3 targetPose;
  pinocchio::SE3 z_up;
  pinocchio::SE3 x_out;
  pinocchio::SE3 y_out;
  pinocchio::SE3 z_target;
  pinocchio::SE3 x_target;
  pinocchio::SE3 y_target;

  std::vector<PIN_IK::BasicJointType> types;

  nlopt::opt opt;

  pinocchio::SE3 currentPose;

  std::vector<double> best_x;
  int progress;
  std::atomic<bool> aborted{false};

  pinocchio::Motion bounds;

  inline static double fRand(double min, double max)
  {
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
  }
};

}

#endif
