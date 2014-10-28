# ============================================================================
# Copyright (c) 2014 Andrew Hundt and Alex Strickland
# All rights reserved.
#
# See COPYING file for license information.
# ============================================================================

##############################################################################
# @file  ConfigSettings.cmake
# @brief Sets variables used in CMake package configuration.
#
# It is suggested to use @c _CONFIG as suffix for variable names that are to
# be substituted in the Config.cmake.in template file in order to distinguish
# these variables from the build configuration.
#
# @note The default BasisConfigSettings.cmake file which is part of the BASIS
#       installation is included prior to this file. Hence, the variables are
#       valid even if a custom project-specific configuration is used and
#       default values can further be overwritten in this file.
#
# @ingroup BasisSettings
##############################################################################

# ============================================================================
# build tree configuration settings
# ============================================================================

if (BUILD_CONFIG_SETTINGS)
  set (DATA_DIR_CONFIG "${PROJECT_DATA_DIR}")
  return ()
endif ()

# ============================================================================
# installation configuration settings
# ============================================================================

set (DATA_DIR_CONFIG "${INSTALL_DATA_DIR}")

# REMOVE THE FOLLOWING LINES IF IT HAS BEEN A WHILE 2014-10-26
#if(NOT CMAKE_INSTALL_PREFIX)
	# default cmake install dir
#  set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)
#endif()