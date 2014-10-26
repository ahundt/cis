
.. meta::
    :description: Andrew Hundt and Alex Strickland Computer Integrated Surgery 600.445 Coursework Repository

.. raw:: latex

    \pagebreak


========
Overview
========

Andrew Hundt and Alex Strickland Computer Integrated Surgery 600.445 Coursework Repository

A narrative report (typically about 5-8 pages long) summarizing
•	The mathematical approach taken
•	The algorithmic steps followed
•	An overview of the structure of the computer program, sufficient to enable someone with reasonable skill (the grader) to understand your approach and follow your code.
•	The steps taken to verify that the program is working correctly. Typically, this would take the form of a discussion of the results using the debugging examples.
•	A tabular summary of the results obtained for unknown data
•	A short discussion for the results of running your program. This certainly includes  the tabular 	summary above, but may also include a discussion of convergence if you adopt an iterative process or of difficulties if you suspect that your answer is wrong.
•	A short statement of who did what.


Introduction:

The purpose of the assignment was to develop an algorithm for a 3D point set to 3D point set registration and a pivot calibration.  The problem involved a stereotactic navigation system and an electromagnetic positional tracking device.  Tracking markers were placed on objects so the optical tracking device and an electromagnetic tracking device could measure the 3D positions of objects in space relative to measuring base units.  These objects were then registered so that they could be related in the same coordinate frames.  Pivot calibration posts were placed in the system so pivot calibration could be performed and the 3D position of two different probes could be tracked throughout the system.  The diagram below gives a visual description of the system.

Insert Picture Here

Mathematical Approach:

A number of least squares methods could be used to determine a transformation matrix for a 3D point set registration.  We selected to use Horn’s method because a rotation matrix is always found and no iterative approximation is involved.  The first step is to find the centroid of the point clouds in the two different coordinate systems.  Then the centroid is subtracted from each point measurement of the separate point clouds so the points will be relative to the centroid.  Next an H matrix is created which is the sum of the products of each corresponding point in the two frames.  A real symmetric G matrix is then created from the sums and differences of the elements in the H matrix which was previously created.  Next, the eigenvalues and corresponding eigenvectors of the G matrix were calculated.  The eigenvector corresponding to the most positive eigenvalue represents the unit quaternion of the matrix.  Once the quaternion is known, the rotation matrix can be found using Rodriguez’s formula.  The translation between the two coordinate systems is next found from the difference between the centroid of the known point cloud and the scaled centroid of the unknown point cloud.  Finally a homogeneous transformation matrix could be made to know the frame transformation between the two point clouds.

For the pivot calibration, singular value decomposition was used to find the transformation between the centroid of the tracked markers on the probe and the tip of the probe.  A matrix was created which consisted of the rotation matrices calculated in each frame and the negative identity matrix.  The singular value decomposition of the matrix was performed to split the matrix into matrices containing the singular values, the left-singular vectors, and the right singular vectors.  Once this was done, the vector between the centroid of the tracked markers on the probe and the probe tip could be determined using the split matrix and the tracked positions of markers.

Algorithmic Approach:

We developed our algorithm using C++.  The Eigen library was used as a Cartesian math package for 3D points, rotations, and frame transformations.  The Boost library was also used to write a parser file and the Point Cloud Library was used to store the 3D positions of each tracker.  The first step was to write parser code that could interpret the given data.  The number of trackers and frames was first interpreted so the 3D marker positions could be put into a matrix corresponding to which object they were on and which frame they were tracked in.  A diagram below shows how the parser function interpreted and stored the data.

INSERT GRAPHIC

Once the data was parsed, two matrices containing marker positions in different coordinate frames was put in the function hornRegistration to determine the corresponding transformation matrix between the two frames.  The first step of the hornRegistration was to find the two centroids of two 3D marker positions and subtract it from each marker position using functions in the Eigen library.  The next step was to put these values in a function that would create a 3x3 H matrix.  Once this was done, the H matrix could be put in a separate function that would calculate the 4x4 G matrix.  The eigenvalues and the corresponding eigenvectors of the G matrix were next calculated by using functions of the Eigen library.  A vector of each eigenvalue and the corresponding eigenvector was then created so that the eigenvalues could be sorted to find the most positive eigenvalue and its corresponding eigenvector which represented the unit quaternion of the rotation.  Next, the 3x3 rotation matrix was created by function that converted a unit quaternion into the corresponding rotation matrix.  Finally, the translation vector between the two centroids was calculated and a 4x4 homogeneous transformation matrix was created by using another function that takes a rotation matrix and a translation vector and outputs the corresponding transformation matrix.
