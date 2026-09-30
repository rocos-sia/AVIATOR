# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/rocos/sia/AVIATOR/third_party/pin_ik-2.2.0"
  "/home/rocos/sia/AVIATOR/build/aviator_clearance/third_party/build/pin_ik-2.2.0"
  "/home/rocos/sia/AVIATOR/build/aviator_clearance/third_party/stamps/pin_ik-2.2.0"
  "/home/rocos/sia/AVIATOR/build/aviator_clearance/third_party/stamps/pin_ik-2.2.0/tmp"
  "/home/rocos/sia/AVIATOR/build/aviator_clearance/third_party/stamps/pin_ik-2.2.0/src/third_party_pin_ik-2.2.0-stamp"
  "/home/rocos/sia/AVIATOR/build/aviator_clearance/third_party/stamps/pin_ik-2.2.0/src"
  "/home/rocos/sia/AVIATOR/build/aviator_clearance/third_party/stamps/pin_ik-2.2.0/src/third_party_pin_ik-2.2.0-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/rocos/sia/AVIATOR/build/aviator_clearance/third_party/stamps/pin_ik-2.2.0/src/third_party_pin_ik-2.2.0-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/rocos/sia/AVIATOR/build/aviator_clearance/third_party/stamps/pin_ik-2.2.0/src/third_party_pin_ik-2.2.0-stamp${cfgdir}") # cfgdir has leading slash
endif()
