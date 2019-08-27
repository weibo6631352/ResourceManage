#include "Common.h"
#include <pcl/common/common.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include "Setting.h"
Common::Common()
{


}


Common::~Common()
{
}

bool Common::FloatComp(const double &a, const double &b)
{
	const static double EPS = 1e-6;
	return abs(a - b) < EPS;
}

void Common::ExtractCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndicesPtr inices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud)
{
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inices);
	extract.filter(*out_cloud);
}

void Common::ExtractCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices inices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud)
{
	pcl::PointIndicesPtr inicesptr(new pcl::PointIndices(inices));
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inicesptr);
	extract.filter(*out_cloud);
}

void Common::ExtractColorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud)
{
	out_cloud->reserve(cloud->size());
	pcl::PointXYZRGB *trvPt;
	for (int i = 0; i < cloud->size(); ++i)
	{
		trvPt = &cloud->at(i);
		if (trvPt->r == 255 && trvPt->g == 255 && trvPt->b == 255)
		{
			out_cloud->push_back(*trvPt);
		}
	}
}

void Common::ExtractColorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
	std::cout << "begin ExtractColorCloud..." << cloud->size() << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	out_cloud->reserve(cloud->size());
	pcl::PointXYZRGB *trvPt;

// 	int r, g, b;
// 	r = g = b = -1;
	for (int i = 0; i < cloud->size(); ++i)
	{
		trvPt = &cloud->at(i);
		

// 		if (trvPt->r != r || trvPt->g != g || trvPt->b != b)
// 		{
// 			std::cout << (int)trvPt->r << " " << (int)trvPt->g << " " << (int)trvPt->b << std::endl;
// 		}
// 		r = trvPt->r;
// 		g = trvPt->g;
// 		b = trvPt->b;

		if (trvPt->r == 254 && trvPt->g == 254 && trvPt->b == 254)
		{
			out_cloud->push_back(*trvPt);
		}
	}
	cloud = out_cloud;

	std::cout << "end ExtractColorCloud." << cloud->size() << std::endl;
}

void Common::GenerateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndicesPtr inices)
{
	//for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->begin(); it != cloud->end(); ++it)
	inices->indices.clear();
	inices->indices.reserve(cloud->size());
	for (int i = 0; i < cloud->size(); ++i)
	{
		inices->indices.push_back(i);
	}
}

void Common::GetMinMax3D(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
	const pcl::PointIndices &indices, pcl::PointXYZRGB &min_pt, pcl::PointXYZRGB &max_pt)
{
	Eigen::Vector4f fmin_pt;
	Eigen::Vector4f fmax_pt;
	pcl::getMinMax3D(cloud, indices, fmin_pt, fmax_pt);
	min_pt.x = fmin_pt.x();
	min_pt.y = fmin_pt.y();
	min_pt.z = fmin_pt.z();

	max_pt.x = fmax_pt.x();
	max_pt.y = fmax_pt.y();
	max_pt.z = fmax_pt.z();
}

void Common::Sample(pcl::PointCloud<pcl::PointXYZRGB> &cloud, double gridsize)
{
	pcl::VoxelGrid<pcl::PointXYZRGB> grid;
	grid.setLeafSize(gridsize, gridsize, 100);
	grid.setInputCloud(cloud.makeShared());
	grid.filter(cloud);
}

void Common::Grid2Cloud(boost::shared_ptr<double> grid, int row, int col, double gd_size, pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
	cloud.reserve(col*row);
	pcl::PointXYZRGB pt;
	for (int i = 0; i < row; ++i)
	{
		for (int j = 0; j < col; ++j)
		{
			pt.x = j*gd_size;
			pt.y = i*gd_size;
			pt.z = grid.get()[col*i + j];
			if (!FloatComp(pt.z,-10000))
			{
				cloud.push_back(pt);
			}
		}
	}
}

void Common::OutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double radius /*= 10*/, double count /*= 10*/, pcl::PointIndicesPtr cloud_indices /*= nullptr*/)
{
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	if (cloud_indices)
		sor.setIndices(cloud_indices);
	sor.setRadiusSearch(radius);
	sor.setMinNeighborsInRadius(count);
	sor.setNegative(false);
	if (cloud_indices)
		sor.filter(cloud_indices->indices);                    //存储
	else
 		sor.filter(*cloud);

// 	// 离群点
// 	//int liqunK = 10;
// 	//double avgDistance = 10;
// 	// 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
// 	//个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
// 	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> Statsor;   //创建滤波器对象
// 	Statsor.setInputCloud(cloud);                           //设置待滤波的点云
// 	if (cloud_indices)
// 		Statsor.setIndices(cloud_indices);
// 	Statsor.setMeanK(3);                               //设置在进行统计时考虑查询点临近点数
// 	Statsor.setStddevMulThresh(0.3);                      //设置判断是否为离群点的阀值
// 
// 	if (cloud_indices)
// 		sor.filter(cloud_indices->indices);                    //存储
// 	else
// 		sor.filter(*cloud);
}

void Common::JuLeiFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, unsigned int threshold)
{
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;   //歐式聚類物件
	ec.setClusterTolerance(0.2);                     // 設定近鄰搜尋的搜尋半徑為2cm
	ec.setMinClusterSize(0);                 //設定一個聚類需要的最少的點數目為100
	ec.setMaxClusterSize(500000000);               //設定一個聚類需要的最大點數目為25000
	ec.setSearchMethod(tree);                    //設定點雲的搜尋機制
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);           //從點雲中提取聚類，並將點雲索引儲存在cluster_indices中


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr res_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{ 
		std::cout << "cluter points count = " << it->indices.size() << std::endl;
		if (it->indices.size() > threshold)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
			new pcl::PointIndices(*it);
			ExtractCloud(cloud, *it, cloud_cluster);
			*res_cloud += *cloud_cluster;
		}
		else
		{
			std::cout << "error cluter points count = " << it->indices.size() << std::endl;
		}
	}
	std::cout << "cloud count = " << cloud->size() << std::endl;
	std::cout << "res_cloud count = " << res_cloud->size() << std::endl;
	cloud = res_cloud;
}