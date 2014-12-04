Introduction
============

This software contains Andrew Hundt and Alex Strickland's implementation of the Programming Assignments (PA) for [Computer Integrated Surgery](http://www.cs.jhu.edu/cista/445/) aka CIS1 at Johns Hopkins University 2014, taught by Dr Russell Taylor. This readme is organizes information about the course for future reference.

Assignment Groupings:

**HW1**

Computer integrated surgical device proposal document

**PA1-2** and **HW2-3** 

Point cloud to point cloud transforms, distortion correction
 
**PA3-4** and **HW4**

Iterative Closest Point, matching sensor point clouds to a mesh

Getting Started
---------------

To get your bearings look at the documentation in the `cis/doc/` folder. The most important information can be found at:

- cis/doc/[CIS_Software_Manual.pdf](https://github.com/ahundt/cis/raw/master/doc/CIS_Software_Manual.pdf)
  - Documentation of the completed assignments
- `cis/doc/referenceSources/`
  - Reference PDF files from lectures and other sources 
    that contain much the information needed to understand 
    and implement the algorithms discussed in the class.
  - Homework Assignment PDFs for HW1-4
  - Programming Assignment PDFs for PA1-5
- [Homework 1 Intermedullary Nailing](https://docs.google.com/document/d/1b3C5iIiOoMnBccvYPlKRPkH8IMZ_uXS1uY1EbEmuVKY/edit?usp=sharing)
  - Surgery used to repair fractured, or broken bones
  - Assignment was to design computer integrated devices 
    to improve the surgical procedure and essentially create
    a basic device, system, and business plan proposal. 

Most important Code
===================

Search the files for the following function names to find the most important function implementations for the class. More details are available in the PDF manual and in the doxygen documentation.

PA1-2
-----

**PA1**

PA1 was completed late, but our output passed all of the tests during the course of PA2 so these components should be fairly reliable.

**hornRegistration()**

point cloud to point cloud transformations. If two point clouds contain the same set of points in different reference frames, this function figures out the 4x4 homogenous transform matrix between the two clouds.

**SVDSolve()**

Singluar Value Decomposition matrix algorithm. Used for things such as point cloud to point cloud transform detection in hornRegistration. This is a broadly useful tool, we used the eigen library's implementation here.  

**pivotCalibration()** 

Calibrates detected medical devices.
  
**PA2**

PA2 final output had a couple bugs, so not everything for PA2 works 100%. This does not affect PA3-4.

We have encountered errors in our software that we have narrowed down to points after the EM distortion calibration steps, because we have been able to verify our Bernstein functions using unit tests and debug data. However, a bug remains in either the steps for calculating Freg or finding each of the CT fiducial. Since the underlying components are largely well tested, we expect the bug to be in the transform or data flow steps of the generateOutputFile() function in cisHW1-2.cpp or the function definitions in PA2.hpp.
**CorrectDistortion()**
Likely ok, but may have serious bugs. Correct distortions in one point cloud by utilizing distorted and undistorted versions of a second point cloud. Bernstein Polynomials are utilized to perform the correction.
**BernsteinPolynomial()**
Likely ok, but may have serious bugs. Find the solution to the Bernstein polynomial when at varying degrees and points depending on the input. Used for distortion correction in this assignment.
**Fmatrix()**
May have serious bugs. Multiplies the Bernstein polynomial into a matrix so that a function of every degree of i, j, and k are found and a distortion calibration can be done using the matrix.

PA3-4
-----

Iterative Closest Point

**ICPwithSimpleSearchStep()**
Primary function implementing the first iteration of the simple search ICP algorithm.**ICPwithSpatialIndexStep()**
Primary function implementing the first iteration of the spatial index ICP algorithm to increase efficiency.

**FindClosestPoint()**
Finds the closest point on the triangle to a point in space. If the closest point lies with in triangle, then the function finds the nearest point internally. Else if the closest point lies on an edge or vertex, the function OutsideOfTriangle() is called to find the nearest point.
**TerminationCriteria**
TerminationCriteria is a C++ Class implemented to evaluate if the current iterations of the ICP algorithm is within our desired error threshold, and to collect statistics on the error as ICP progresses.

PA5
---

PA5 was optional and not completed.

License
=======

Copyright (c) 2014 Andrew Hundt and Alex Strickland <br />

See COPYING file for license information.



Installation
============

See build and installation instructions given in the [INSTALL](/INSTALL.md) file.




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
