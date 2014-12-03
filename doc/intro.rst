
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



PA1
---
 
The purpose of PA1 was to develop an algorithm for a 3D point set to 3D point set registration and a pivot
calibration. The problem involved a stereo tactic navigation system and an electromagnetic positional tracking
device. Tracking markers were placed on objects so the optical tracking device and an electromagnetic tracking
device could measure the 3D positions of objects in space relative to measuring base units. These objects were
then registered so that they could be related in the same coordinate frames. Pivot calibration posts were placed
in the system so pivot calibration could be performed and the 3D position of two different probes could be
tracked throughout the system. The diagram below from the assignment document gives a visual description of the
system.


.. image:: static/CIS_PA1_AssignmentPicture.png


PA2
---

In addition to all of the steps outlined in PA1, the core purpose of PA2 was to develop an algorithm for
distortion correction and to implement it for use with the stereo tactic navigation system of PA1. In addition to
distortion correction, we performed registration of the device coordinate frames to prior CT coordinate frames.


PA3
---

The purpose of PA3 was to develop the find closest point step of an iterative-closest point registration 
algorithm. The problem involved a 3D triangular surface mesh of a bone found in CT coordinates and two rigid 
bodies. One rigid body is rigidly attached to the bone and one to be used as a pointer. LED markers were attached
to the two rigid bodies so that the coordinates could be determined in optical coordinates. The closest point on 
the triangular mesh to a number of points where the tip of the pointer contacted the bone was found using our find
closest point algorithm. The diagram below from the assignment document gives a visual description of the system.


.. image:: static/PA3_Problem_Drawing.png


PA4
---

The purpose of PA4 was to build off the find closest points algorithm from PA3 and implement an iterative-closest
point registration (ICP) algorithm. The closest point on the triangular mesh to the points where the tip of the 
pointer contacted the bone was found iteratively until the error threshold was reached. In this way, the error
between these points could be minimized.  


Mathematical Approach
=====================


Distortion Correction
---------------------

A number of distortion correction methods are available to correct for inaccuracies among various sensor
coordinate systems and the real physical dimensions of the world. We selected Bernstein polynomials for our
implementation due to their numerical stability and accuracy for the specific electromagnetic distortion problem
we encounter. The basic idea in this case is to construct a 3D polynomial representing the spatial flexing
caused by distortions in measurements.


To reap the best of the numerical stability properties of the Bernstein polynomial we scale the input values to
the range from 0 to 1. Therefore we scale the values to within the range [0,1] in each dimension, utilizing the
minimum bounding rectangle (MBR) to determine the scale factor. Then, we construct a 5th degree Bernstein
polynomial for each point using the polynomial function outlined in slide 18 of the InterpolationReview.pdf
lecture notes pictured below.


.. image:: static/bernstein_polynomial_clean.png

We then stack the polynomials to form the F Matrix, although this polynomial can be increased for higher
precision or decreased for higher performance as needed. Once we have these polynomials stacked as a large
matrix we solve the least squares problem utilizing SVD against the ground truth data, as outlined on slide 43
of the lecture notes pictured below.


.. image:: static/bernstein_polynomial_SVD_3D_calibration.png

The output of that equation is the calibration coefficient matrix which can then be multiplied by the stacked F
matrix of a distorted point set to generate the final corrected and undistorted point set. Multiplication of a
stacked matrix is a more efficient alternative to the loop of sums from the slides.


Tradeoffs
~~~~~~~~~

One of the particular advantages of Bernstein polynomials is the ability to select the degree of the polynomial.
The polynomial degree presents an interesting trade off, because a higher degree polynomial allows more precise
representation of distortions and lower error. This benefit comes at the cost of an exponential increase in
computation time for each additional polynomial degree.


Point Cloud Registration
------------------------

A number of least squares methods could be used to determine a transformation matrix for a 3D point set
registration. We selected Horn’s method because a rotation matrix is always found and no iterative approximation
is involved. The first step is to find the centroid of the point clouds in the two different coordinate systems.
Then the centroid is subtracted from each point measurement of the separate point clouds so the points will be
relative to the centroid. Next an H matrix is created which is the sum of the products of each corresponding
point in the two frames. A real symmetric G matrix is then created from the sums and differences of the elements
in the H matrix which was previously created. Next, the eigenvalues and corresponding eigenvectors of the G
matrix were calculated. The eigenvector corresponding to the most positive eigenvalue represents the unit
quaternion of the matrix. Once the quaternion is known, the rotation matrix can be found using Rodriguez’s
formula. The translation between the two coordinate systems is next found from the difference between the
centroid of the known point cloud and the scaled centroid of the unknown point cloud. Finally a homogeneous
transformation matrix could be made to know the frame transformation between the two point clouds.


Pivot Calibration
-----------------

For the pivot calibration, singular value decomposition was used to estimate the orientation of the probe by
finding the positions of the centroid of the tracked markers on the probe and the tip of the probe. First, a
matrix was created which consisted of the rotation matrices calculated in each frame and the negative identity
matrix found using Horn’s method.\[1] Next, a vector was created which consisted of the stack of the translation
vectors in each frame also found using Horn’s method. The singular value decomposition of the matrix was
performed to split the matrix into the matrices containing the singular values, the left-singular vectors, and
the right singular vectors. Once this was done, the vector between the centroid of the tracked markers on the
probe and the probe tip could be approximated using the SVD matrices and the translation vectors of each frame.


 * \[1] Horn, Closed-form solution of absolute orientation using unit quaternions, Optical Society of America (1987)

 
Finding the Closest Point on a Triangle
---------------------------------------
 
If the vertices of a triangle are know and there is a point in space, then the closest point that lies on the triangle to 
the point in space can be found. This is done by using the equations given below:\[2]

.. image:: static/PA3_GraphicsApproach_Eq1.png

.. image:: static/PA3_GraphicsApproach_Eq2.png

.. image:: static/PA3_GraphicsApproach_Eq3.png

.. image:: static/PA3_GraphicsApproach_Eq4.png

Where P is the point in space, P1, P2, and P3 are the vertices of the triangles. Then the closest point on the triangle P'
can be found using Barycentric coordinates listed below:\[2]
 
.. image:: static/PA3_GraphicsApproach_Eq5.png

.. image:: static/PA3_GraphicsApproach_Eq6.png

.. image:: static/PA3_GraphicsApproach_Eq7.png

.. image:: static/PA3_GraphicsApproach_Eq8.png
 
If alpha, beta, and gamma are all between zero and one, then the P' is the closest point and lies within the triangle's 
boundaries. If not, the closest point lies on the boundaries of the triangle and the point P must be projected onto every 
side of the triangle. The equations (from the Finding point-pairs lecture page 8) below how this is implemented:

.. image:: static/PA3_GraphicsApproach_Eq9.png

.. image:: static/PA3_GraphicsApproach_Eq10.png

Where X and Y are the two end points (or vertices) of the line segment, P is the point to be projected on the line segment,
P* is the projected point on the line segment, and lambda* is the ratio of normalized length from X to P*. If two of the 
three P* projections lie on the same vertex, then the closest point on the triangle is that vertex. Otherwise, the closest 
point will be the P* projection on the side whose value for lambda* satisfies the conditions of being between one and zero. 
Then the equation (from the Finding point-pairs lecture page 8) below is implemented to find the closest point P*:

.. image:: static/PA3_GraphicsApproach_Eq11.png

* \[2] W. Heidrich, Journal of Graphics, GPU, and Game Tools,Volume 10, Issue 3, 2005.


ICP
---

For every point on the bone where the probe tip was placed, the find the closest point method was implemented for every 
triangle on the mesh. Once the closest point on the mesh was found, the error between the two points was computed by taking 
the norm. Then the point on the triangle with the smallest error was said to be the closest point on the mesh. Next, a new
transformation between the points on the bone and the point on the mesh is found using the Point Cloud Registration method.
Then the process is repeated using new points from the new transformation to minimize the error between the points on the 
bone and the points on the mesh.

 
 
Algorithmic Approach:
=====================


Parsing
-------

We developed our algorithm using C++. The Eigen library was used as a Cartesian math package for 3D points,
rotations, and frame transformations. The Boost library was also used to write a parser file and develop various
aspects of our algorithms. The first step was to write parser code that could interpret the given data. The
parser needed to interpret which data set was being entered, the number of frames in each data set, and which
markers were being tracked in the data set. The parser would store the data as Eigen matrices to be easily used
for our algorithms.


Transforms
----------

Once the data was parsed, two matrices containing marker positions in different coordinate frames was put in the
function hornRegistration to determine the corresponding transformation matrix between the two frames. The first
step of the hornRegistration was to find the two centroids of two 3D marker positions and subtract it from each
marker position using functions in the Eigen library. The next step was to put these values in a function that
would create a 3x3 H matrix. Once this was done, the H matrix could be put in a separate function that would
calculate the 4x4 G matrix. The eigenvalues and the corresponding eigenvectors of the G matrix were next
calculated by using functions of the Eigen library. A vector of each eigenvalue and the corresponding
eigenvector was then created so that the eigenvalues could be sorted to find the most positive eigenvalue and
its corresponding eigenvector which represented the unit quaternion of the rotation. Next, the 3x3 rotation
matrix was created by an Eigen function that converted a unit quaternion into the corresponding rotation matrix.
Finally, the translation vector between the two centroids was calculated and a 4x4 homogeneous transformation
matrix was created by using another function that takes a rotation matrix and a translation vector and outputs
the corresponding transformation matrix.


Pivot Calibration
-----------------

Next a pivot calibration algorithm was created which used both the parser and hornRegistration algorithms
mentioned above. First, the tracker data was parsed into separate matrices which corresponded to each frame of
tracked data. Each matrix of frame data was compared to the base matrix frame using the hornRegistration
function described above and the corresponding homogeneous transformation from the base frame to the current
frame was found. The rotational component of each frame was put into an Eigen matrix and the translational
component of each frame was put into an Eigen vector with the form described in the mathematical approach above.
The function of JacobiSVD of the Eigen library was then used to solve the least squares vector between the
rotational matrix and translation vectors. The least squares vector contained approximated orientation of the
probe and the position of the probe tip.


Distortion Calibration
----------------------

Next we create a distortion calibration algorithm, which followed the mathematical procedure outlined above. First,
the data was parsed and stored in a large vector so the the maximum and minimum values could be obtained in the X, Y,
and Z dimensions of the data set. Then the values of the data set were scaled to between [0 1] to create a minimum 
bounding box. We calculate Bernstein polynomials for each point and stack them into the F matrix. The Eigen library
is utilized to calculate the SVD of Fc=p, where F is the F matrix of Bernstein Polynomials, c is the calibration
coefficient matrix, and p is the undistorted points matrix that you compare the distorted points to. A separate
set of points can be scaled according to the same distortion parameters from above and the distortion associated with
their measurement can be corrected. 

Once this is done, an the same Fmatrix calculated above can be calculated for this new set of points. Then the c
calibration coefficient matrix was multiplied by the Fmatrix to find the undistorted points in the new coordinate system.
Once the data was undistorted, we were able to run a new pivot calibration with the undistorted points to see how well
our undistortion works. The next step was to use the distortion matrix to find CT fiducial in the EM frame. The new data
was scaled by the same scaling function as above and put into a new Fmatrix. In this way, a new Fmatrix could be found
and the measured points could be undistorted. Then these values were used with known values of points measured in the CT
frame to find a transformation matrix Freg that would take you from the EM frame to the CT frame. Finally the tip of the
EM probe could be measured in the CT frame.

 
Find Nearest Point
------------------

Next a Find Nearest Point algorithm was created which used both the parser and hornRegistration algorithms
mentioned above. First, each point of tracker data was parsed into Eigen vectors of (x,y,z) coordinates which 
corresponded to the position of the trackers attached to the rigid bodies, A and B, in optical coordinates. Next, another 
set of tracker data was parsed into Eigen vectors of (x,y,z) coordinates which corresponded to the position of the 
trackers attached to the rigid bodies in their body coordinates. The transformation matrix from the body frame to the 
optical tracker frame was then computed using the hornRegistration function described above. Then the coordinates tip of 
the rigid body A with respect to rigid body B was found by multiplying the vector of the tip in body A coordinates by the 
transformations previously found. 

Next, the mesh data was parsed so the vertices of each triangle was known. Then ICP registration could be used to find
the point on the mesh that was closest to the tip of rigid body A. First, the transformation from the CT mesh coordinates
to the rigid body B coordinates was assumed to be the identity matrix. Once this assumption was made, sample points were
found by multiply the transformation from CT mesh coordinates to the rigid body B coordinates by the tip of the pointer A
in rigid body A coordinates. Now these sample points were used to which points on the CT mesh they were closest to with
the given transformation. The simplest FindNearestPoint function was implemented in which the the nearest point to the
sample points on the CT mesh was calculated for every triangle in the mesh. The error between the two points for each
triangle was calculated by taking the norm between the points and the smallest error corresponded to the nearest point on
the mesh to the pointer tip A.


ICP
---

The ICP algorithm continuously ran the Find Nearest point algorithm until the error criteria was reached. On the first
iteration, the transformation from the CT mesh coordinates and the rigid body B coordinates was assumed to be the identity
matrix. Then on subsequent iterations, a new estimate of the transformation was found using a hornRegistration from the
rigid body B coordinates to the closest points found on the CT mesh found in the previous iteration. Once the transformation
was found, the sample points were computed the same way as in the Find Nearest Point Algorithm and the steps were continued
until new closest points on the mesh were found. The error calculated in each iteration was compared to the error criteria
and if the error calculated was below the criteria, then ICP algorithm was stopped and the new closest points and transformation
were assumed to be a good estimate.


Spatial Indexing
----------------

Additionally, an alternative ICP algorithm was implemented using the `Boost.Geometry Spatial Index library <http://www.boost.org/doc/libs/1_57_0/libs/geometry/doc/html/geometry/spatial_indexes/introduction.html>`_ , which contains
an r-tree implementation. The preferred mechanism to accelerate search using a SpatialIndex is to insert each triangle into
the index, then query the r-tree for the nearest triangle to a given point. This substantially speed up all data access by
reducing access time for an individual element from O(n) to O(M log_M(n)), where M is the number of entries in a node.

However, this data structure does not yet have direct polygon insertion implemented. Instead the minimum bounding rectangle,
or bounding box, of each triangle is inserted into the index with a reference to the underlying triangle. From this, the
nearest bounding boxes are queried and visited in order from nearest to furthest, and the underlying polygon distance is
checked. As soon as a bounding box distance is reached that is entirely further away than the nearest polygon, the search is
stopped and the ICP algorithm proceeds as normal. This is necessary because the bounding boxes of different triangles can
overlap, returning results that are near to but not the actual closest triangle to the point. ultimately this is a
substantially accelerated lookup of the closest point on the mesh.




Structure of the Program
========================

The software is structured as a set of header only libraries in the include folder, which are utilized by
the unit tests, main, and any external libraries that choose to use these utilities.


The most important files include:


=============================   ===============================================================================
File name                       Description
=============================   ===============================================================================
**IterativeClosestPoint.hpp**   Algorithm for finding ICP registration.
**hornRegistration.hpp**        Horn's method of Point Cloud to Point Cloud registration.
**DistortionCalibration.hpp**   Bernstein Polynomial method of distortion correction.
**PivotCalibration.hpp**        Pivot Calibration.
**PA2.hpp**                     **fiducialPointInEMFrame()** and **probeTipPointinCTFrame()** PA2 #4,6
**cisHW1test.cpp**              An extensive set of unit tests for the library relevant to PA1.
**cisHW2test.cpp**              An extensive set of unit tests for the library relevant to PA2.
**cisHW3test.cpp**              An extensive set of unit tests for the library relevant to PA3.
**cisHW1-2.cpp**                Main executable source, contains cmdline parsing code and produces output data.
**cisHW3-4.cpp**                Main executable source, contains cmdline parsing code and produces output data.
**parseCSV...**                 File parsing functions are in **parseCSV_CIS_pointCloud.hpp**.
=============================   ===============================================================================






Important Functions and Descriptions
------------------------------------

Each function includes substantial doxygen documentation explaining its purpose and usage. This documentation
can be viewed inline with the source code, or via a generated html sphinx + doxygen website generated using CMake. 
Here is a list of the most important functions used in the program is a brief description of each of them.

PA 1
~~~~

**EigenMatrix()**         	   

Computes the eigenvalues and corresponding eigenvectors from a given G matrix. It 
outputs a rotation matrix corresponding to the unit quaternion of the largest 
positive eigenvalue

**homogeneousmatrix()**          

Creates a 4x4 homogeneous matrix from a derived rotational matrix and translational vector

**hornRegistration()**

Computes the homogeneous transformation matrix F given a set of two cloud points. 
It is comprised of the various functions listed above

**homogeneousInverse()**		   

Computes the inverse of a given homogeneous matrix 

**registrationToFirstCloud()**   

Parses the data and runs the hornRegistration function for pivot calibration

**transformToRandMinusIandPMatrices()**   

Creates the A and b components of the form Ax=b for singular value decomposition.
A is of the form [R|-I] while b is of the form [-p] where R is the stack of 
rotational matrices of the F transformation matrices, I is stack of 3x3 identity 
matrices, and p is the stack of the translational vectors of the F transformation 
matrices.

**SVDSolve()**				   

Computes the x of the least squares problem Ax=b using singular value decomposition
when the stack of matrices in given


**Hmatrix()**   			
	
Computes a sum of the products H matrix given a set of two cloud points

**Gmatrix()**					  
 
Computes a sum of the differences of the given H matrix

**pivotCalibration()**

Computes the pivot point position from tracking data using the SVDSolve(), 
registrationToFirstCloud(), and transformToRandMinusIandPMatrices() functions


PA 2
~~~~

**CorrectDistortion()**

Correct distortions in one point cloud by utilizing distorted and undistorted versions of a second point cloud.
Bernstein Polynomials are utilized to perform the correction.

**BernsteinPolynomial()**

Find the solution to the Bernstein polynomial when at varying degrees and points depending on the input.

**Fmatrix()**

Multiplies the Bernstein polynomial into a matrix so that a function of every degree of i, j, and k are found
and a distortion calibration can be done using the matrix.

**ScaleToUnitBox()**

Calculates maximum and minimum values in the X,Y, and z coordinates of a point cloud and then normalizes the 
value of every single point.

**probeTipPointinCTF()**

Uses measured positions of EM tracker points on the EM probe in the EM frame when the tip is in a CT fiducial
and returns the point of the fiducial dimple (solves problem 5).

**fiducialPointInEMFrame()**

Uses measured positions of EM tracker points on the EM probe in the EM frame when the tip is in a CT fiducial
and returns points of the CT fiducial locations in EM frame.


PA 3
~~~~

**ICPwithSimpleSearchStep()**

Primary function implementing the first iteration of the simple search ICP algorithm.

**ICPwithSpatialIndexStep()**

Primary function implementing the first iteration of the spatial index ICP algorithm to increase efficiency.

**dkKnownMeshPointsBaseFrame()**

Finds the location of of Atip in fiducial body B coordinates (dk) using hornRegistration.

**FindClosestPoint()**

Finds the closest point on the triangle to a point in space. If the closest point lies with in triangle, then the
function finds the nearest point internally. Else if the closest point lies on an edge or vertex, the function
OutsideOfTriangle() is called to find the nearest point.

**OutsideOfTriangle()**

Finds the closest point on the triangle to a point in space if the closest point lies on an edge or vertex

**ProjectOnSegment()**

Finds the nearest point on a line segment to a point in space. Called by the function OutsideOfTriangle() to determine
where the nearest point is to each side of the triangle.

**PointEqualityCheck()**

Determines if two points are equal. Used by the function OutsideOfTriangle to determine if the nearest point on the
triangle lies on a vertex


PA 4
~~~~

**ICPwithSimpleSearch()**

ICP algorithm that iterates the simple search of the FindClosestPoint algorithm until an error threshold is reached.

**ICPwithSpatialIndex()**

ICP algorithm that iterates the spatial index of the FindClosestPoint algorithm until an error threshold is reached.
The spatial index is used to increase efficiency of the ICP algorithm.

**TerminationCriteria**

TerminationCriteria is a C++ Class implemented to evaluate if the current iterations of the ICP algorithm is within 
our desired error threshold, and to collect statistics on the error as ICP progresses.



Results and Discussion
======================

Validation
----------

We took several approaches to the validation of our software. These include manual and automatic execution of
the supplied test data, the implementation of unit tests to verify the data, and initial integration of
continuous integration software to catch errors early. We implemented a battery of unit tests to verify the
basic functions and ensure they are running correctly.


Point Cloud Registration
~~~~~~~~~~~~~~~~~~~~~~~~

We have been able to ensure that point cloud to point cloud registration is working correctly by finding the
transformation of one point cloud to another and then the opposite. Multiplying these two transformation
matrices together resulted in an identity matrix which would be expected. We tested the input data set as well,
ensuring that we were within the given tolerance range. This shows the strength of Horn’s method and since it requires no
special case exceptions for a solution, we concluded it was the best method of the one's taught in class.

Calibration
~~~~~~~~~~~

The position of the tip of the probe when calibrate by EM also gave us results well within our tolerance
levels. Our results were less accurate when error was introduced, but not to an unreasonable degree.

Finding the Closest Point on a Triangle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We have also been able to ensure that finding the closest point on a triangle algorithm is working correctly by assigning
vertices to an arbitrary triangle and then testing points in space where we knew what the closest point on the triangle
was. We tested the different special cases of the problem as the closest point lying within the boundaries of the
triangle, on one of the sides of the triangle, and on one of the vertices of the triangle. Our algorithm was able to
return the nearest point for every case.


Status of results
=================

PA 2
----

We have encountered errors in our software that we have narrowed down to points after the EM distortion calibration
steps, because we have been able to verify our Bernstein functions using unit tests and debug data. However, a bug
remains in either the steps for calculating Freg or finding each of the CT fiducial. Since the underlying components are
largely well tested, we expect the bug to be in the transform or data flow steps of the generateOutputFile() function in
cisHW1-2.cpp or the function definitions in PA2.hpp.

PA 3
----

Our initial algorithm passed our unit tests and gave us similar errors to the output data files. We concluded that we
were able to successfully implement the first iteration of an ICP registration. Although the simplest linear method was
implemented, our program was able to run the program of all debug and unknown data sets in less than thirty seconds even
in debug mode. Adding threads and running in release mode made our program run even faster. A data structure was not
needed for this assignment because we concluded that there was no problem with the speed of the program. For PA 4, we will
re-evaluate this conclusion as the ICP will need to iterate multiple times, greatly increasing our runtime.

Tabular Summary of PA 3 Unknown Data Results
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

=================   ===============   ===============   ===============
Point Number        Error in G Data   Error in H Data   Error in I Data
=================   ===============   ===============   ===============
       1               0.14811            0.77494            2.84084   
       2               0.60432            0.96266            0.46215      
       3               2.24293            0.67760            4.43498      
       4               0.76945            0.24216            1.77055      
       5               1.45001            0.51152            0.40509      
       6               0.80641            1.54260            0.10251      
       7               0.86017            0.24527            0.30031      
       8               2.61080            0.72909            0.79210      
       9               2.10517            0.48146            2.54912      
       10              0.96220            0.30248            2.07283      
       11              0.00198            1.43465            5.54007      
       12              2.45932            0.01729            1.06041
       13              1.47099            1.38414            2.93195
       14              1.34390            0.42595            1.44610
       15              0.41964            2.43422            1.83156
       16              1.44879            0.13935            0.57452
       17              0.01526            0.29749            0.47596
       18              1.96765            0.88617            0.87031
       19              1.30039            0.65453            3.82845
       20              1.02220            0.18214            1.17058      
**Average Error**    **1.20048**        **0.71629**        **1.77302**
  **Max Error**      **2.61080**        **2.43422**        **5.54007**
=================   ===============   ===============   ===============

The error in debug data sets is consistent with the error in the output files. On the unknown output, the overall error
bounds seem reasonable when compared to the debug data sets. We expect our algorithms are not the most substantial source
of error in these results and instead are attributed to noise or other sources of error.
 

PA 4
----

Performance
~~~~~~~~~~~

We took several approaches to performance optimization. First, we ensured there are instructions for building and executing
the code in release mode with high performance settings for fast execution. We also ensured all of our functions were
implemented with high performance in mind, utilizing the Eigen C++ library functions and other cases where vectorization of
data allows higher performance. Then, we implemented threading so that all data sets can be executed simultaneously, 
which resulted in an approximately 8 fold speed up for computers with multiple processors. These performance criteria
allow the application to execute all the data sets in 35.5 seconds on a 2014 Intel Core i7 processor at 2.5 Ghz. This performance 
was achieved with simple search. Spatial Indexing provides a substantial additional performance boost on top of optimized 
simple search, executing all the data sets in 2.41s on a 2014 Intel Core i7 processor at 2.5 Ghz.

Stopping Error Criteria
~~~~~~~~~~~~~~~~~~~~~~~
We implemented several different stopping criteria for our ICP algorithm. The first was a minimum iteration so that the
ICP had to run a minimum number of iterations before it could be stopped. The second was a mean error criteria so that the
ICP would not stop iterating unless the mean error was sufficiently low. The third was a max error criteria so that the
ICP would not stop iterating unless the maximum error criteria was sufficiently low. The next criteria computes a rolling
variance in the mean error at each iteration. Effectively, this allows the algorithm to stop correctly for a variety of
data sets when error improvements at each iteration have diminished, regarless of what ideal minimum is possible. This
criteria produced the best results for stopping correctly across a variety of data sets. The final criteria was a maximum
iteration so that the ICP would not just keep running forever if the error never met any of the previous criteria. Each
error criteria is checked after each iteration by the TerminationCriteria object to determine if the ICP should be stopped.


Tabular Summary of PA 4 Results
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

=================   ===============   ===============   ===============
File                Error Mean        Error Max         Error Variance
=================   ===============   ===============   ===============
PA4-A-Debug         0.00197113        0.00717058        2.23062e-06 
PA4-B-Debug         0.00186124        0.00787022        2.47282e-06
PA4-C-Debug         0.00176908        0.0069919         2.21001e-06 
PA4-D-Debug         0.00324727        0.0133826         6.58918e-06 
PA4-E-Debug         0.0668846         0.275337          0.0031848 
PA4-F-Debug         0.0593784         0.256141          0.00257083 
PA4-G-Unknown       0.00332878        0.0149228         7.39938e-06 
PA4-H-Unknown       0.00364877        0.0118835         7.24306e-06 
PA4-J-Unknown       0.0650882         0.337438          0.00308615 
PA4-K-Unknown       0.0665567         0.259723          0.00288765
=================   ===============   ===============   ===============

The mean, max, and variance of the errors found at the end of the ICP algorithm were low for all data sets. Also our
error results from the debug data sets were similar to those of Dr. Taylor's output files. From these observations, 
we concluded that our final Freg found was a very reasonable transformation from the points on the bone to the points 
on the mesh and that we chose our error stopping criteria well.

In the tabular summary above, it can be seen that the mean and max error is an order of magnitude higher in data sets 
E, F, J, and K than in the rest of the data sets. This is due to error propagation in these data sets which are 
discussed below. In the two debug sets E and F, our error results were still similar to Dr. Taylor's output files.


Error Propagation
-----------------

Barring errors due to software bugs, error propagation can occur based on several sources. If there is systemic biased
measurement in a single direction, this can offset error and cause it to propagate along transform chains and even
amplify error.

PA 2
~~~~

Error sources and propagation can come from a variety of sources, including EM distortion, EM Noise, and OT jiggle. 
We were able to account for the EM distortion through our distortion calibration functions. It is expected that some
amount of EM Noise, distortion, and jiggle will be propagated throughout the system that we are unable to account for.

One example of how error can propagate is if both the optical tracker and EM tracker are off with a common distortion
component, it is possible for this information to cause the Bernstein curve to misestimate the actual curve, and
consequently cause the registration between the CT scan and the other sensors to have a higher error. In this way errors
can propagate through the whole system. This particular example can be mitigated through the use of fixed physical
structures that are known in advance that can be used to estimate and account for such systemic errors.

Additionally, inaccurate sensors due to large random variation are an example of error which cannot be removed through 
distortion calibration. 

PA 3 and 4
~~~~~~~~~~

Possible error sources and propagation were due to simulated noise. The early debug sets had the least amount of
simulated noise corresponding to very low error while the later debug sets and unknown sets had a greater simulated noise
corresponding to a slightly larger error. The simulated noise could have been attributed to a number of sources including
Optical distortion, Optical Noise, Optical jiggle, CT distortion, or CT Noise.


Results Metric
--------------

PA 2
~~~~

We know that our distortion is correct and we can measure its accuracy because we can compare the old values 
of EM pivot to the newly undistorted values that we encounter. By comparing to prior ground truth values we 
can assess the accuracy of our calibration.

Our metric for error is the distance difference between our calculations and the debug outputs. This can be measured
as an average, or with other statistical tools. We can also detect certain sources of error by specifying our own test
functions. We also utilize the **BOOST_VERIFY** macro and the checkWitinTolerances() function to verify that functions
are being called and returning values that or correct to within certain tolerances, considering the limits of the
particular algorithms we are using. 

PA 3
~~~~

Our metric for error is the norm between the sample points and the nearest points on the CT mesh. The norm was small in
most cases so we concluded that our implementation of the first step of ICP registration was successful. In PA 4, we will 
expand on our current ICP by iterating and setting an error bound to obtain more accurate results.

PA 4
~~~~

Our metric for error is the norm between the sample points and the nearest points on the CT mesh. An ICP was run until
the error was minimized below our error criteria. Our ICP algorithm produced low errors so we concluded that our
transformation matrix, Freg, was a good representation of mapping points on the bone to points on the CT mesh. In addition
to our low errors, our ICP also ran quickly so we concluded that our error bounds used as stopping criteria was a good 
choices for our program.


Andrew and Alex spent approximately equal time on the assignment, with significant amounts of time spent pair
programming. Both contributed equally to the implementation and debugging of functions.



Additional Information
======================
