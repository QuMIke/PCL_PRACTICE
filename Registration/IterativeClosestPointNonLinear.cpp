/*
This document demonstrates using the Iterative Closest Point algorithm in order to incrementally register a series of point clouds two by two.

The idea is to transform all the clouds in the first cloud’s frame.
This is done by finding the best transform between each consecutive cloud, and accumulating these transforms over the whole set of clouds.
Your data set should consist of clouds that have been roughly pre-aligned in a common frame (e.g. in a robot’s odometry or map frame) and overlap with one another.
*/

#include "stdafx.h"
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// visulizer
pcl::visualization::PCLVisualizer *p;
// its left and right viewports
int vp_1, vp_2;

// convenint structure to handle our pointclouds
struct PCD
{
	PointCloud::Ptr cloud;
	std::string f_name;

	PCD() :
		cloud(new PointCloud) {};
};

struct PCDComparator
{
	bool operator () (const PCD& p1, const PCD& p2)
	{
		return (p1.f_name < p2.f_name);
	}
};

// define a new point representation for <x, y, curvature>
class MyPointRepresentation : public pcl::PointRepresentation<PointNormalT>
{
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
	MyPointRepresentation()
	{
		// define the number of dimentions
		nr_dimensions_ = 4;
	}

	// override the copyToFloatArray method to define our featur vector
	virtual void copyToFloatArray(const PointNormalT &p, float *out) const
	{
		// <x, y, z ,curvature>
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};


// brief display source and target on the first viewport of the visualzier
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
	p->removePointCloud("vp1_target");
	p->removePointCloud("vp2_source");

	PointCloudColorHandlerCustom<PointT> tgt_h(cloud_target, 0, 255, 0);
	PointCloudColorHandlerCustom<PointT> src_h(cloud_source, 255, 0, 0);
	p->addPointCloud(cloud_target, "vp1_target", vp_1);
	p->addPointCloud(cloud_source, "vp2_source", vp_2);

	PCL_INFO("Press q to begin the registration.\n");
	p->spin();

}

// brief display sorcee and target on the second viewport of the visualzier
void showPointCLoudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
	p->removePointCloud("source");
	p->removePointCloud("target");

	PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(cloud_target, "curvature");
	if (!tgt_color_handler.isCapable())
	{
		PCL_WARN("Cannot create curvature color handler!");
	}

	PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(cloud_source, "curvature");
	if (!src_color_handler.isCapable())
		PCL_WARN("Cannot create curvature color handler!");

	p->addPointCloud(cloud_target, tgt_color_handler, "target", vp_2);
	p->addPointCloud(cloud_source, src_color_handler, "source", vp_2);

	p->spinOnce();
}

/**\brief Load a set of PCD files that we want to register together
  *\param argc the number of arguments (pass from main())
  *\param argv the actual conmmand line arguments (pass from main())
  *\param models the resultant vector of point cloud datasets
  */
void loadData(int argc, char** argv, std::vector<PCD, Eigen::aligned_allocator<PCD>> &models)
{
	std::string extension(".pcd");
	// suppose the first argument is the actual test model
	for (int i = 1; i < argc; i++)
	{
		std::string fname = std::string(argv[i]);
		// need to be at least 5: .plot
		if (fname.size() <= extension.size())
			continue;
		std::transform(fname.begin(), fname.end(), fname.begin(), (int(*)(int))tolower);

		// check that the argument is a pcd file
		if (fname.compare(fname.size - extension.size(), extension.size(), extension) == 0)
		{
			// load the cloud and saves it into the global list of models
			PCD m;
			m.f_name = argv[i];
			pcl::io::loadPCDFile(argv[i], *m.cloud);
			// remove NAN points from the cloud
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);

			models.push_back(m);
		}
	}
}

/** \brief Align a paire of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_tanrsform the resultant transform between source and target
  */
void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
	// Downsample for consistency and speed
	// \note enable this for large datasets
	PointCloud::Ptr src(new PointCloud);
	PointCloud::Ptr tgt(new PointCloud);
	pcl::VoxelGrid<PointT> grid;
	if (downsample)
	{
		grid.setLeafSize(0.05, 0.05, 0.05);
		grid.setInputCloud(cloud_src);
		grid.filter(*src);

		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
	}

	// compute surface normals and curvature


}
