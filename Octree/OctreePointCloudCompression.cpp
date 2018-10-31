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

#include "stdafx.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <stdio.h>
#include <sstream>
#include <stdlib.h>

#include "KinnectGrabber.h"

int main()
{
	pcl::io::OpenNI2Grabber grabber;
	KinnectGrabber<pcl::PointXYZRGBA> v(grabber);
	v.run();

	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	cloud = v.getLatestCloud();

	if (cloud == nullptr)
	{
		std::cout << "Get cloud failed!" << std::endl;
	}
	else
	{
		std::stringstream compressedData;
		pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
		boost::shared_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>> pointCloudEncoder(
			new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, true));
		pointCloudEncoder->encodePointCloud(cloud, compressedData);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>());
		boost::shared_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>> pointCloudDecoder(
			new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>());
		pointCloudDecoder->decodePointCloud(compressedData, cloudOut);

		if (cloudOut == nullptr)
		{
			std::cout << "Decode compressed data failed!\n";
		}
		else
		{
			pcl::visualization::CloudViewer viewer("Compressed point cloud");
			viewer.showCloud(cloudOut);
			while (!viewer.wasStopped())
			{
				Sleep(1000);
			}
		}

	}

	return 0;

}
