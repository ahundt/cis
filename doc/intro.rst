
.. meta::
    :description: Andrew Hundt and Alex Strickland Computer Integrated Surgery 600.445 Coursework Repository

.. raw:: latex

    \pagebreak


========
Overview
========

Andrew Hundt and Alex Strickland Computer Integrated Surgery 600.445 Coursework Repository

Introduction
============

The purpose of the assignment was to develop an algorithm for a 3D point set to 3D point set registration and a pivot calibration.  The problem involved a stereotactic navigation system and an electromagnetic positional tracking device.  Tracking markers were placed on objects so the optical tracking device and an electromagnetic tracking device could measure the 3D positions of objects in space relative to measuring base units.  These objects were then registered so that they could be related in the same coordinate frames.  Pivot calibration posts were placed in the system so pivot calibration could be performed and the 3D position of two different probes could be tracked throughout the system.  The diagram below from the assignment document gives a visual description of the system.

.. image:: static/CIS_PA1_AssignmentPicture.png

Mathematical Approach
=====================

Point Cloud Registration
------------------------

A number of least squares methods could be used to determine a transformation matrix for a 3D point set registration.  We selected to use Horn’s method because a rotation matrix is always found and no iterative approximation is involved.  The first step is to find the centroid of the point clouds in the two different coordinate systems.  Then the centroid is subtracted from each point measurement of the separate point clouds so the points will be relative to the centroid.  Next an H matrix is created which is the sum of the products of each corresponding point in the two frames.  A real symmetric G matrix is then created from the sums and differences of the elements in the H matrix which was previously created.  Next, the eigenvalues and corresponding eigenvectors of the G matrix were calculated.  The eigenvector corresponding to the most positive eigenvalue represents the unit quaternion of the matrix.  Once the quaternion is known, the rotation matrix can be found using Rodriguez’s formula.  The translation between the two coordinate systems is next found from the difference between the centroid of the known point cloud and the scaled centroid of the unknown point cloud.  Finally a homogeneous transformation matrix could be made to know the frame transformation between the two point clouds.

Pivot Calibration
-----------------

For the pivot calibration, singular value decomposition was used to estimate the orientation of the probe by finding the positions of the centroid of the tracked markers on the probe and the tip of the probe.  First, a matrix was created which consisted of the rotation matrices calculated in each frame and the negative identity matrix found using Horn’s method.\[1]  Next, a vector was created which consisted of the stack of the translation vectors in each frame also found using Horn’s method.  The singular value decomposition of the matrix was performed to split the matrix into the matrices containing the singular values, the left-singular vectors, and the right singular vectors.  Once this was done, the vector between the centroid of the tracked markers on the probe and the probe tip could be approximated using the SVD matrices and the translation vectors of each frame.

 * \[1] Horn, Closed-form solution of absolute orientation using unit quaternions, Optical Society of America (1987)


Algorithmic Approach:
=====================

Parsing
-------

We developed our algorithm using C++.  The Eigen library was used as a Cartesian math package for 3D points, rotations, and frame transformations.  The Boost library was also used to write a parser file and develop various aspects of our algorithms.  The first step was to write parser code that could interpret the given data.  The parser needed to interpret which data set was being entered, the number of frames in each data set, and which markers were being tracked in the data set.  The parser would store the data as Eigen matrices to be easily used for our algorithms.  A diagram below shows how the parser function interpreted and stored the data.


Transforms
----------

Once the data was parsed, two matrices containing marker positions in different coordinate frames was put in the function hornRegistration to determine the corresponding transformation matrix between the two frames.  The first step of the hornRegistration was to find the two centroids of two 3D marker positions and subtract it from each marker position using functions in the Eigen library.  The next step was to put these values in a function that would create a 3x3 H matrix.  Once this was done, the H matrix could be put in a separate function that would calculate the 4x4 G matrix.  The eigenvalues and the corresponding eigenvectors of the G matrix were next calculated by using functions of the Eigen library.  A vector of each eigenvalue and the corresponding eigenvector was then created so that the eigenvalues could be sorted to find the most positive eigenvalue and its corresponding eigenvector which represented the unit quaternion of the rotation.  Next, the 3x3 rotation matrix was created by an Eigen function that converted a unit quaternion into the corresponding rotation matrix.  Finally, the translation vector between the two centroids was calculated and a 4x4 homogeneous transformation matrix was created by using another function that takes a rotation matrix and a translation vector and outputs the corresponding transformation matrix.

Pivot Calibration
-----------------

Next a pivot calibration algorithm was created which used both the parser and hornRegistration algorithms mentioned above.  First, the tracker data was parsed into separate matrices which corresponded to each frame of tracked data.  Each matrix of frame data was compared to the base matrix frame using the hornRegistration function described above and the corresponding homogeneous transformation from the base frame to the current frame was found.  The rotational component of each frame was put into an Eigen matrix and the translational component of each frame was put into an Eigen vector with the form described in the mathematical approach above.  The function of JacobiSVD of the Eigen library was then used to solve the least squares vector between the rotational matrix and translation vectors.  The least squares vector contained approximated orientation of the probe and the position of the probe tip.

Structure of the Program
========================

The most important files include:

========================   ====================================================================================
File name                  Description
========================   ====================================================================================
**hornRegistration.hpp**   Functions for implemention Horn's method of Point Cloud to Point Cloud registration.
**PivotCalibration.hpp**   Functions for implementing Pivot Calibration.
**cisHW1test.cpp**         An extensive set of unit tests for the library.
**cisHW1main.cpp**         Main executable source, contains cmdline parsing code and produces output data.
**parseCSV...**            File parsing functions are in **parseCSV_CIS_pointCloud.hpp**.
========================   ====================================================================================

The software is structured as a set of header only libraries in the include folder, which are utilized by
the unit tests, main, and any external libraries that choose to use these utilities.

Each function includes substantial doxygen documentation explaining its purpose and usage. This documentation
can be viewed inline with the source code, or via a generated html sphinx + doxygen website generated using CMake.  Here is a list of the most important functions used in the program is a brief description of each of them.

========================   ====================================================================================
Function name                  Description
========================   ====================================================================================
Hmatrix   				   Computes a sum of the products H matrix given a set of two cloud points
Gmatrix					   Computes a sum of the differences of the given H matrix
EigenMatrix         	   Computes the eigenvalues and corresponding eigenvectors from a given G matrix.  It 
						   outputs a rotation matrix corresponding to the unit quaternion of the largest 
						   positive eigenvalue
homogeneousmatrix          Creates a 4x4 homogeneous matrix from a derived rotational matrix and translational 
						   vector
hornRegistration           Computes the homogeneous transformation matrix F given a set of two cloud points.  
						   It is comprised of the various functions listed above
homogeneousInverse		   Computes the inverse of a given homogeneous matrix 
registrationToFirstCloud   Parses the data and runs the hornRegistration function for pivot calibration
transformToRandMinusIan-   Creates the A and b components of the form Ax=b for singular value decomposition.
dPMatrices                 A is of the form [R|-I] while b is of the form [-p] where R is the stack of 
						   rotational matrices of the F transformation matrices, I is stack of 3x3 identity 
						   matrices, and p is the stack of the translational vectors of the F transformation 
						   matrices.
SVDSolve				   Computes the x of the least squares problem Ax=b using singular value decomposition
						   when the stack of matrices in given.
========================   ====================================================================================


Results and Discussion
======================

We implemented a battery of unit tests to verify the basic functions and ensure they are running correctly. We have been able to ensure that point cloud to point cloud registration is working correctly by finding the transformation of one point cloud to another and then the opposite.  Multiplying these two transformation matrices together resulted in an identity matrix which would be expected.  We tested the input data set as well, ensuring that we were within the given tolerance range.  Our program outputted almost exact results when the data was run with no error.  When error such as EM distortion, EM noise, and OT jiggle, were introduced in the data, our results were still very close to the expected results and were well within our tolerance range.  This shows the strength of Horn’s method and since it requires no special case exceptions for a solution, we concluded it was the best method of the one's taught in class.  The position of the tip of the probe when calibrate by EM also gave us results well with in out tolerance levels.  Again, it could be seen that our results were less accurate when error was introduced, but not to an unreasonable degree.  We did encounter a bug when trying to calibrate the optical probe in the EM coordinate system.  Our results for the x and y values of the pivot point were in the tolerance range of the actual pivot point.  However, we ran into a large systematic shift in the negative z-direction of our output data.  We believe that there was a possible ordering or directional issue in our algorithmic transforms.  Since the shift was systematic throughout all the test data, the value of the error was well know and could accounted for if this were a real system until the bug was found.

Status of results
======================



Andrew and Alex spent approximately equal time on the assignment, with significant amounts of time spent pair programming. Both contributed equally to the implementation and debugging of funcitons.

Additional Information
======================
