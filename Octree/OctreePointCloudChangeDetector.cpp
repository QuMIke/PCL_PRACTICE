/*
An octree is a tree-based data structure for organizing sparse 3-D data. 
In this tutorial we will learn how to use the octree implementation
for detecting spatial changes between multiple unorganized point clouds which could vary in size, resolution, density and point ordering.
By recursively comparing the tree structures of octrees, 
spatial changes represented by differences in voxel configuration can be identified. Additionally,
we explain how to use the pcl octree “double buffering” technique allows us to efficiently process multiple point clouds over time.
*/

