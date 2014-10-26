
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

Mathematical Approach:

A number of least squares methods could be used to determine a transformation matrix for a 3D point set registration.  We selected to use Horn’s method because a rotation matrix is always found and no iterative approximation is involved.  The first step is to find the centroid of the point clouds in the two different coordinate systems.  Then the centroid is subtracted from each point measurement of the separate point clouds so the points will be relative to the centroid.  Next an H matrix is created which is the sum of the products of each corresponding point in the two frames.  A real symmetric G matrix is then created from the sums and differences of the elements in the H matrix which was previously created.  Next, the eigenvalues and corresponding eigenvectors of the G matrix were calculated.  The eigenvector corresponding to the most positive eigenvalue represents the unit quaternion of the matrix.  Once the quaternion is known, the rotation matrix can be found using Rodriguez’s formula.  The translation between the two coordinate systems is next found from the difference between the centroid of the known point cloud and the scaled centroid of the unknown point cloud.  Finally a homogeneous transformation matrix could be made to know the frame transformation between the two point clouds.

For the pivot calibration, singular value decomposition was used to find the transformation between the centroid of the tracked markers on the probe and the tip of the probe.  A matrix was created which consisted of the rotation matrices calculated in each frame and the negative identity matrix.  The singular value decomposition of the matrix was performed to split the matrix into matrices containing the singular values, the left-singular vectors, and the right singular vectors.  Once this was done, the vector between the centroid of the tracked markers on the probe and the probe tip could be determined using the split matrix and the tracked positions of markers.

Algorithmic Approach: