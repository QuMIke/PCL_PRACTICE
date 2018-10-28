/*
The idea of the feature extraction method is as follows.
Having a mesh and a set of points for which feature must be computed we perform some simple steps. 
First of all for a given point of interest the local surface is cropped. 
Local surface consists of the points and triangles that are within the given support radius. 
For the given local surface LRF (Local Reference Frame) is computed. 
LRF is simply a triplet of vectors, the comprehensive information about how these vectors are computed you can find in the article.
What is really important is that using these vectors we can provide the invariance to the rotation of the cloud.
To do that, we simply translate points of the local surface in such way that point of interest became the origin,
after that we rotate local surface so that the LRF vectors were aligned with the Ox, Oy and Oz axes.
Having this done, we then start the feature extraction.
For every axis Ox, Oy and Oz the following steps are performed, we will refer to these axes as current axis:
  local surface is rotated around the current axis by a given angle;
  points of the rotated local surface are projected onto three planes XY, XZ and YZ;
  for each projection distribution matrix is built, this matrix simply shows how much points fall onto each bin. Number of bins represents the matrix dimension and is the parameter of the algorithm, as well as the support radius;
  for each distribution matrix central moments are calculated: M11, M12, M21, M22, E. Here E is the Shannon entropy;
  calculated values are then concatenated to form the sub-feature.
  We iterate through these steps several times. Number of iterations depends on the given number of rotations. Sub-features for different axes are concatenated to form the final RoPS descriptor.
  
For this tutorial we will use the model from the Queen’s Dataset. You can choose any other point cloud, 
but in order to make the code work you will need to use the triangulation algorithm in order to obtain polygons. 
*/

