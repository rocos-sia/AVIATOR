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

#ifndef TRAC_IK_IK_RNG_HPP
#define TRAC_IK_IK_RNG_HPP

#include <random>

namespace TRAC_IK
{
// Deterministic per-thread RNG replacing the process-global `rand()` in the three
// fRand implementations (trac_ik.hpp, kdl_tl.hpp, nlopt_ik.hpp). `rand()`/`srand()`
// are process-global and non-reproducible under multithreading (16 atlas workers
// interleave the same stream). A `thread_local` engine instead gives every OS
// thread its own independent, explicitly-seeded stream.
//
// TRAC_IK::CartToJnt spawns two solver threads (runKDL / runNLOPT) per call, so
// the seed is NOT propagated from the caller's thread; each of those spawned
// threads seeds its own thread-local engine from TRAC_IK::seed_ at the top of
// runKDL / runNLOPT. A caller therefore gets a fully reproducible CartToJnt result
// simply by calling TRAC_IK::setSeed(seed) once before the call.
inline std::mt19937 &ik_rng()
{
    static thread_local std::mt19937 rng{0u};
    return rng;
}

inline void ik_rng_seed(unsigned seed)
{
    ik_rng().seed(seed);
}

inline double ik_frand(double min, double max)
{
    std::uniform_real_distribution<double> dist(min, max);
    return dist(ik_rng());
}
}

#endif  // TRAC_IK_IK_RNG_HPP
