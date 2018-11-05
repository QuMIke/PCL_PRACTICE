/*
This document demonstrates using the Iterative Closest Point algorithm in order to incrementally register a series of point clouds two by two.

The idea is to transform all the clouds in the first cloud’s frame.
This is done by finding the best transform between each consecutive cloud, and accumulating these transforms over the whole set of clouds.
Your data set should consist of clouds that have been roughly pre-aligned in a common frame (e.g. in a robot’s odometry or map frame) and overlap with one another.
*/

