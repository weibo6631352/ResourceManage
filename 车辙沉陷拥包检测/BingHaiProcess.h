#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

#include <boost/shared_ptr.hpp>

class BingHaiProcess
{
public:
	//定义三个结构体用于存放处理后的沉陷拥包车辙数据
	struct ChenXian{
		int index;
		int i;
		int j;
		int width;
		int length;
		int area;
		int deep;
	};
	struct YongBao{
		int index;
		int i;
		int j;
		int width;
		int length;
		int area;
		int deep;
	};
	struct CheZhe{
		int index;
		int i;
		int j;
		int width;
		int length;
		int area;
		int deep;
	};

	BingHaiProcess(boost::shared_ptr<double> cloud, int row, int col, double gridsize);
	~BingHaiProcess();

	
	void Check();

	boost::shared_ptr<double> cloud_;

	std::vector<ChenXian> cx_vector_;
	std::vector<YongBao> yb_vector_;
	std::vector<CheZhe> cz_vector_;
	boost::shared_ptr<boost::shared_ptr<int>> flag_cx;
	boost::shared_ptr<boost::shared_ptr<int>> flag_yb;


	int row;
	int column;

	double cz_standard;
	double cx_standard;
	double yb_standard;
protected:
	double GetAroundAvgHeight(int i, int j);
	void mark_cx(int i, int j, int index);
	void mark_yb(int i, int j, int index);



	int area ;
	int i_min, i_max, j_min, j_max;
	int deep ;

	double gridSize_;
};

