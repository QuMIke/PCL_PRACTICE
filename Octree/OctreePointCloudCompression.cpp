/*
Point clouds consist of huge data sets describing three dimensional points associated with additional information
such as distance, color, normals, etc. Additionally, they can be created at high rate 
and therefore occupy a significant amount of memory resources.
Once point clouds have to be stored or transmitted over rate-limited communication channels,
methods for compressing this kind of data become highly interesting. 
The Point Cloud Library provides point cloud compression functionality. 
It allows for encoding all kinds of point clouds including “unorganized” point clouds that are characterized 
by non-existing point references, varying point size, resolution, density and/or point ordering. Furthermore, 
the underlying octree data structure enables to efficiently merge point cloud data from several sources.
*/
