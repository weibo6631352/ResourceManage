#include "GroundExtract.h"
#include <pcl/PointIndices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

#include "Setting.h"

#include <set>

GroundExtract::GroundExtract()
{
}


GroundExtract::~GroundExtract()
{
}


void GroundExtract::FindGroundIndices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndicesPtr cloud_indices, pcl::PointIndicesPtr ground_indices)
{
	// ������ȡ
	Setting setting = Setting::ins();
	int windowsize = setting.groundExact_.windowsize;
	float slope = setting.groundExact_.slope;
	float minlDistance = setting.groundExact_.minlDistance;
	float maxlDistance = setting.groundExact_.maxlDistance;



	//	������̬�˲���
	pcl::ProgressiveMorphologicalFilter<pcl::PointXYZRGB> pmf;
	pmf.setInputCloud(cloud);
	pmf.setMaxWindowSize(windowsize);
	pmf.setSlope(slope);
	pmf.setInitialDistance(minlDistance);
	pmf.setMaxDistance(maxlDistance);

	std::cout << "��ȡ����..." << std::endl;
	//��ȡ����
	pmf.extract(ground_indices->indices);
	cloud_indices->indices.reserve(cloud->size() - ground_indices->indices.size());
	std::set<int>ground_indices_set(ground_indices->indices.begin(), ground_indices->indices.end());
	for (int i = 0; i < cloud->size(); ++i)
	{
		if (ground_indices_set.find(i) == ground_indices_set.end())
			cloud_indices->indices.push_back(i);
	}
	std::cout << "��ȡ�������..." << std::endl;
}
