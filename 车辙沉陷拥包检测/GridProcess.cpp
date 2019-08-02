#include "GridProcess.h"

#include <pcl/PointIndices.h>

#include <iomanip>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp> 

#include "Setting.h"
#include "Common.h"

GridProcess::GridProcess()
	:row_(0), col_(0), size_(0)
{
}

GridProcess::~GridProcess()
{
}

pcl::PointXYZRGB GridProcess::GetGridMid(pcl::PointXYZRGB min, int row, int col, double gridSize, boost::shared_ptr<int*> &pointsArr)
{
	pcl::PointXYZRGB pt;
	pt.x = min.x + (col + 0.5) * gridSize;
	pt.y = min.y + (row + 0.5) * gridSize;
	pt.z = pointsArr.get()[row][col];
	return pt;
}

int GridProcess::GetRow(const pcl::PointXYZRGB &min, const pcl::PointXYZRGB &pt, const double gridSize)
{
	return (pt.y - min.y) / gridSize;
}

int GridProcess::GetCol(const pcl::PointXYZRGB &min, const pcl::PointXYZRGB &pt, const double gridSize)
{
	return (pt.x - min.x) / gridSize;
}

// 将点云存入二维数组
void GridProcess::PutPointCloud2Arr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr ground_indices, boost::shared_ptr<double> &pointsArr
	, int row, int col, pcl::PointXYZRGB min, double gridSize)
{
	boost::shared_ptr<int> countArr(new int[row*col], [](int*p){delete[] p; });
	memset(countArr.get(), 0, sizeof(int) * row*col);

	std::cout << "calc grid's  count&z" << std::endl;
	for (int i = 0; i < ground_indices->indices.size(); ++i)
	{
		pcl::PointXYZRGB &pt = cloud->at(ground_indices->indices[i]);
		++(countArr.get()[GetRow(min, pt, gridSize) * col + GetCol(min, pt, gridSize)]);
		pointsArr.get()[GetRow(min, pt, gridSize) * col + GetCol(min, pt, gridSize)] += (double)pt.z;
	}

	for (int i = 0; i < row; ++i)
	{
		for (int j = 0; j < col; ++j)
		{
			int count = countArr.get()[i*col + j];
			double &z = pointsArr.get()[i*col + j];
			if (0 == count)
				z = -10000;
			else
			{
				z /= (double)count;
				//std::cout << std::fixed << std::setprecision(5) << i << "," << j << "," << count << "：\t" << z << std::endl;
			}
		}
	}

}

void GridProcess::GenerateGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndicesPtr ground_indices)
{
	// RecoverGround
	size_ = Setting::ins().gridProcess_.gridSize;

	pcl::PointIndices::Ptr ground_erase(new pcl::PointIndices);
	std::vector<pcl::PointIndices::Ptr> towerIndies;

	// 计算二维数组的行列值
	pcl::PointXYZRGB min, max;
	Common::GetMinMax3D(*cloud, *ground_indices, min, max);

	row_ = (max.y - min.y) / size_ + 1;
	col_ = (max.x - min.x) / size_ + 1;

	std::cout << "点云数量" << cloud->size() << "\t地面点索引数量 " << ground_indices->indices.size() << std::endl;
	std::cout << "初始化平面网格" << (int)max.x << " " << (int)max.y << std::endl;


	pointArr_ = boost::shared_ptr<double>(new double[row_*col_], [](double*p){delete[] p; });
	memset(pointArr_.get(), 0, sizeof(double) * row_*col_);

	std::cout << ("申请网格数组") << row_ << "," << col_ << std::endl;
	PutPointCloud2Arr(cloud, ground_indices, pointArr_, row_, col_, min, size_);

}