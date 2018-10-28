/*
  For this tutorial we will use GreedyProjectioTriangulation to generate polygonmesh.
*/

#include "stdafx.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include "KinnectGrabber.h"

int main()
{
	pcl::io::OpenNI2Grabber grabber;
	KinnectGrabber<pcl::PointXYZ> v(grabber);

	v.run();
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud = nullptr;
	cloud = v.getLatestCloud();
	if (cloud == nullptr)
	{
		std::cout << "Get cloud failed!" << std::endl;
	}
	else
	{
		// estimate normals
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
		pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>);
		normal_estimator.setInputCloud(cloud);
		normal_estimator.setRadiusSearch(0.01);
		normal_estimator.setSearchMethod(tree_n);
		normal_estimator.compute(*normal);

		// concatenate cloud and normal
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields(*cloud, *normal, *cloud_normal);

		// create polygonmesh to store output
		pcl::PolygonMesh triangles;
		// create search method kdtree
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree_c_n(new pcl::search::KdTree<pcl::PointNormal>);

		// create greedy projection triangulation object and paras
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		gp3.setSearchRadius(1.5f);
		gp3.setMu(2.5f);
		gp3.setMaximumNearestNeighbors(1000);
		gp3.setMaximumSurfaceAngle(M_PI / 4.0);
		gp3.setMinimumAngle(M_PI / 18);
		gp3.setMaximumAngle(2 * M_PI / 3);
		gp3.setNormalConsistency(false);
		gp3.setSearchMethod(tree_c_n);

		// input cloud with normal and reconstruct
		gp3.setInputCloud(cloud_normal);
		gp3.reconstruct(triangles);

		// display result
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
		viewer->setCameraFieldOfView(1.02259994f);
		viewer->setPosition(0, 0);
		viewer->setSize(cloud->width, cloud->height);
		viewer->resetCameraViewpoint("3D viewer");
		viewer->setCameraPosition(
			0, 0, 0,		// Position
			0, 0, 1,		// Viewpoint
			0, -1, 0);	// Up
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPolygonMesh(triangles);
		viewer->addCoordinateSystem(1.0);
		//viewer->initCameraParameters();

		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(1000));
		}

	}

	return 0;
}
