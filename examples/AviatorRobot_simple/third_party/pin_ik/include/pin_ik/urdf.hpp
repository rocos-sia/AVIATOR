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

#ifndef PIN_IK_URDF_HPP
#define PIN_IK_URDF_HPP

#include <pin_ik/pinocchio_types.hpp>
#include <string>

namespace PIN_IK {

/**
 * @brief Load a Pinocchio model from URDF string for a kinematic chain
 *
 * @param urdf_xml URDF XML string
 * @param base Base link name
 * @param tip Tip link name
 * @param model Output Pinocchio model (will contain only the chain from base to tip)
 * @param lower Output scalar lower joint limits (model.nv; continuous: -infinity)
 * @param upper Output scalar upper joint limits (model.nv; continuous: +infinity)
 * @param tip_frame_id Output frame index of the tip
 */
void loadURDFModel(const std::string& urdf_xml,
                   const std::string& base,
                   const std::string& tip,
                   Model& model,
                   Eigen::VectorXd& lower,
                   Eigen::VectorXd& upper,
                   FrameIndex& tip_frame_id);

} // namespace PIN_IK

#endif // PIN_IK_URDF_HPP
