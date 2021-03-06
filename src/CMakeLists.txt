# ============================================================================
# Copyright (c) 2014 Andrew Hundt and Alex Strickland
# All rights reserved.
#
# See COPYING file for license information.
# ============================================================================

##############################################################################
# @file  CMakeLists.txt
# @brief Build configuration of software.
##############################################################################

# ============================================================================
# library target(s)
# ============================================================================

# Add library target for each library using basis_add_library().
#
# This command can not only be used to build libraries from C/C++, but also source
# code written in other programming languages such as Java, Python, Perl,
# MATLAB, and Bash. Note that here we consider modules written in a scripting
# language which are no executables but to be included by other scripts written
# in the particular language as libraries.
#
# Note: Public modules written in a scripting language such as Python, Perl,
#       MATLAB, or Bash which are intended for use by other packages should
#       be placed in the lib/[<lang>/]CIS/ directory,
#       where <lang> is the language name in lowercase and is optional.
#       BASIS will automatically add a library build target for these modules.



# ============================================================================
# executable target(s)
# ============================================================================

# Add executable target for each executable program using basis_add_executable().
#
# This command can not only be used to build executables from C/C++, but also
# source code written in other programming languages such as Java, Python, Perl,
# MATLAB, and Bash.

basis_include_directories(${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIR})
basis_add_executable(cisHW1-2.cpp)
basis_target_link_libraries(cisHW1-2 ${PCL_LIBRARIES} ${Boost_LIBRARIES})

basis_add_executable(cisHW3-4.cpp)
basis_target_link_libraries(cisHW3-4 ${PCL_LIBRARIES} ${Boost_LIBRARIES})
