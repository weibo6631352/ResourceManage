#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>

#include <boost/shared_ptr.hpp>

class Common
{
public:
	Common();
	~Common();

	bool static FloatComp(const double &a, const double &b);
	void static GenerateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndicesPtr inices);
	void static ExtractCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndicesPtr inices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud);
	void static ExtractCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices inices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud);
	void static ExtractColorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud);
	void static ExtractColorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
	void static GetMinMax3D(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const pcl::PointIndices &indices, pcl::PointXYZRGB &min_pt, pcl::PointXYZRGB &max_pt);
	void static Sample(pcl::PointCloud<pcl::PointXYZRGB> &cloud, double gridsize);
	void static Grid2Cloud(boost::shared_ptr<double> grid, int row, int col, double gd_size, pcl::PointCloud<pcl::PointXYZRGB> &cloud);
	void static OutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double radius /*= 10*/, double count /*= 10*/, pcl::PointIndicesPtr cloud_indices = nullptr);
	void static JuLeiFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, unsigned int threshold);
};

