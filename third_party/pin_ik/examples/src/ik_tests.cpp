/********************************************************************************
Copyright (c) 2016, TRACLabs, Inc.
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
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>

int main(int argc, char** argv) {
  if (argc < 4 || argc > 5) {
    std::cerr << "Usage: ik_tests robot.urdf base_link tip_link [samples]\n";
    return 2;
  }
  try {
    std::ifstream file(argv[1]);
    if (!file) throw std::runtime_error("Cannot open URDF file");
    std::string xml((std::istreambuf_iterator<char>(file)), {});
    const int samples = argc == 5 ? std::stoi(argv[4]) : 100;
    if (samples <= 0) throw std::invalid_argument("samples must be positive");

    PIN_IK::PIN_IK solver(argv[2], argv[3], xml, 0.05);

    pinocchio::Model model;
    Eigen::VectorXd lower, upper;
    solver.getModel(model);
    solver.getLimits(lower, upper);

    pinocchio::Data data(model);
    pinocchio::FrameIndex tip_frame_id = model.getFrameId(argv[3]);

    Eigen::VectorXd target(model.nv), seed(model.nv), result;
    std::mt19937 rng(42);
    int success = 0;

    for (int i = 0; i < samples; ++i) {
      for (int j = 0; j < model.nv; ++j) {
        const double lo = lower(j) <= std::numeric_limits<float>::lowest() ? -M_PI : lower(j);
        const double hi = upper(j) >= std::numeric_limits<float>::max() ? M_PI : upper(j);
        target(j) = std::uniform_real_distribution<double>(lo, hi)(rng);
        seed(j) = (lo + hi) / 2;
      }

      // Forward kinematics to get goal pose
      pinocchio::forwardKinematics(model, data, PIN_IK::toPinocchioConfiguration(model, target));
      pinocchio::updateFramePlacements(model, data);
      pinocchio::SE3 goal = data.oMf[tip_frame_id];

      if (solver.CartToJnt(seed, goal, result) < 0) continue;

      // Verify FK of result
      pinocchio::forwardKinematics(model, data, PIN_IK::toPinocchioConfiguration(model, result));
      pinocchio::updateFramePlacements(model, data);
      pinocchio::SE3 actual = data.oMf[tip_frame_id];

      // Check if poses are close
      if (!goal.isApprox(actual, 1e-4))
        throw std::runtime_error("FK residual exceeds tolerance");

      // Check joint limits
      for (int j = 0; j < result.size(); ++j)
        if (result(j) < lower(j)-1e-8 || result(j) > upper(j)+1e-8)
          throw std::runtime_error("Solution violates joint limits");

      ++success;
    }

    std::cout << "Solved " << success << '/' << samples
              << "; FK tolerance 1e-4; joint limits checked\n";
    return success == samples ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
