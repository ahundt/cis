# ============================================================================
# Copyright (c) 2014 Andrew Hundt and Alex Strickland
# All rights reserved.
#
# See COPYING file for license information.
# ============================================================================

##############################################################################
# @file  CMakeLists.txt
# @brief Build configuration of auxiliary data files.
#
# The data within this folder is required to execute the software tools of
# this project. Example data which is only required for an example run of
# the software are, on the other side, available in the example folder.
#
# By default, all content of this directory is simply installed in the
# @c INSTALL_DATA_DIR directory of the installation tree. To group data files,
# place them in proper subdirectories which will be copied to the
# @c DESTINATION directory of the basis_install() function.
##############################################################################

basis_install_directory ("." "${INSTALL_DATA_DIR}")

# REMOVE THE FOLLOWING LINES IF IT HAS BEEN A WHILE 2014-10-26
#basis_get_target_property(HW1_TEST_TARGET_LOCATION TARGET cisHW1test PROPERTY LOCATION )
#message(STATUS <<<<<<<<<<${HW1_TEST_TARGET_LOCATION})
#basis_get_filename_component(HW1_TEST_TARGET_LOCATION_DIR ${HW1_TEST_TARGET_LOCATION} DIRECTORY)

#file(COPY PA1-2 DESTINATION ${HW1_TEST_TARGET_LOCATION_DIR})