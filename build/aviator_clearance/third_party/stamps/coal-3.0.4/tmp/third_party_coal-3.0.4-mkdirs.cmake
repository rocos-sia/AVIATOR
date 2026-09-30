# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/rocos/sia/AVIATOR/third_party/coal-3.0.4"
  "/home/rocos/sia/AVIATOR/build/aviator_clearance/third_party/build/coal-3.0.4"
  "/home/rocos/sia/AVIATOR/build/aviator_clearance/third_party/stamps/coal-3.0.4"
  "/home/rocos/sia/AVIATOR/build/aviator_clearance/third_party/stamps/coal-3.0.4/tmp"
  "/home/rocos/sia/AVIATOR/build/aviator_clearance/third_party/stamps/coal-3.0.4/src/third_party_coal-3.0.4-stamp"
  "/home/rocos/sia/AVIATOR/build/aviator_clearance/third_party/stamps/coal-3.0.4/src"
  "/home/rocos/sia/AVIATOR/build/aviator_clearance/third_party/stamps/coal-3.0.4/src/third_party_coal-3.0.4-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/rocos/sia/AVIATOR/build/aviator_clearance/third_party/stamps/coal-3.0.4/src/third_party_coal-3.0.4-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/rocos/sia/AVIATOR/build/aviator_clearance/third_party/stamps/coal-3.0.4/src/third_party_coal-3.0.4-stamp${cfgdir}") # cfgdir has leading slash
endif()
