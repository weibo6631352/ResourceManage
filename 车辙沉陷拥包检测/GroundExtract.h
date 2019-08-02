#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

class GroundExtract
{
public:
	GroundExtract();
	~GroundExtract();
	void static FindGroundIndices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndicesPtr cloud_indices, pcl::PointIndicesPtr ground_indices);
};

