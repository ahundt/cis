Introduction
============


The purpose of the assignment was to develop an algorithm for a 3D point set to 3D point set registration and a pivot calibration.  The problem involved a stereotactic navigation system and an electromagnetic positional tracking device.  Tracking markers were placed on objects so the optical tracking device and an electromagnetic tracking device could measure the 3D positions of objects in space relative to measuring base units.  These objects were then registered so that they could be related in the same coordinate frames.  Pivot calibration posts were placed in the system so pivot calibration could be performed and the 3D position of two different probes could be tracked throughout the system. Resources utilized include \[1].

* \[1] Horn, Closed-form solution of absolute orientation using unit quaternions, Optical Society of America (1987)




License
=======

Copyright (c) 2014 Andrew Hundt and Alex Strickland <br />

See COPYING file for license information.



Installation
============

See build and installation instructions given in the [INSTALL](/INSTALL.md) file.



Documentation
=============

See the software manual for details on the software including a demonstration
of how to apply the software tools provided by this package.



Package Content
===============

Path                    | Content description
----------------------- | ----------------------------------------------------------
[BasisProject.cmake][1] | Meta-data used for the build configuration.
[CMakeLists.txt]    [2] | Root CMake configuration file.
[config/]           [3] | Package configuration files.
[data/]             [4] | Data files required by this software.
[doc/]              [5] | Documentation source files.
[example/]          [6] | Example files used for demonstration.
[include/]          [7] | Public header files.
[src/]              [8] | Source code files.
[test/]             [9] | Regression and unit tests.






<!-- --------------------------------------------------------------------------------- -->

<!-- Links to GitHub, see the local directory if you have downloaded the files already -->
[1]: /BasisProject.cmake
[2]: /CMakeLists.txt
[3]: /config
[4]: /data
[5]: /doc
[6]: /example
[7]: /include
[8]: /src
[9]: /test
