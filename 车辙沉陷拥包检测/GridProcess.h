#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

class GridProcess
{
public:
	GridProcess();
	~GridProcess();
	pcl::PointXYZRGB static GetGridMid(pcl::PointXYZRGB min, int row, int col, double gridSize, boost::shared_ptr<int*> &pointsArr);
	int static GetRow(const pcl::PointXYZRGB &min, const pcl::PointXYZRGB &pt, const double gridSize);
	int static GetCol(const pcl::PointXYZRGB &min, const pcl::PointXYZRGB &pt, const double gridSize);
	void static PutPointCloud2Arr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr ground_indices
		, boost::shared_ptr<double> &pointsArr, int row, int col, pcl::PointXYZRGB min, double gridSize);
	void GenerateGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndicesPtr ground_indices);


	boost::shared_ptr<double> pointArr_;
	int row_;
	int col_;
	double size_;
};
