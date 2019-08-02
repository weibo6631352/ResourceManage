#include "BingHaiProcess.h"
#include <pcl/segmentation/cpc_segmentation.h>

#include "Setting.h"

BingHaiProcess::BingHaiProcess(boost::shared_ptr<double> cloud, int row, int col, double gridsize)
	:cloud_(cloud), column(col), row(row), gridSize_(gridsize)
{
	
	area = 0;
	i_min = i_max = j_min = j_max = 0;
	deep = 0;
	cz_standard = 2;
	cx_standard = -0.2;
	yb_standard = 0.2;
}


BingHaiProcess::~BingHaiProcess()
{
}

double BingHaiProcess::GetAroundAvgHeight(int i, int j)
{
	const double unvalid_gdnum = Setting::ins().gridProcess_.unvalid_num;
	int valid_num = 0;
	double height_sum = 0;

	if (j + 1 < column && cloud_.get()[i*column + j + 1] != unvalid_gdnum) {
		++valid_num;
		height_sum += cloud_.get()[i*column + j + 1];
	}
	if (i + 1 < row && j + 1 < column && cloud_.get()[(i + 1)*column + j + 1] != unvalid_gdnum) {
		++valid_num;
		height_sum += cloud_.get()[(i + 1)*column + j + 1];
	}
	if (i + 1 < row && cloud_.get()[(i + 1)*column + j] != unvalid_gdnum) {
		++valid_num;
		height_sum += cloud_.get()[(i + 1)*column + j];
	}
	if (i + 1 < row && j - 1 >= 0 && cloud_.get()[(i + 1)*column + j - 1] != unvalid_gdnum) {
		++valid_num;
		height_sum += cloud_.get()[(i + 1)*column + j - 1];
	}
	if (j - 1 >= 0 && cloud_.get()[i*column + j - 1] != unvalid_gdnum) {
		++valid_num;
		height_sum += cloud_.get()[i*column + j - 1];
	}
	if (i - 1 >= 0 && j - 1 > 0 && cloud_.get()[(i - 1)*column + j - 1] != unvalid_gdnum){
		++valid_num;
		height_sum += cloud_.get()[(i - 1)*column + j - 1];
	}
	if (i - 1 >= 0 && cloud_.get()[(i - 1)*column + j] != unvalid_gdnum){
		++valid_num;
		height_sum += cloud_.get()[(i - 1)*column + j];
	}
	if (i - 1 >= 0 && j + 1 < column && cloud_.get()[(i - 1)*column + j + 1] != unvalid_gdnum){
		++valid_num;
		height_sum += cloud_.get()[(i - 1)*column + j + 1];
	}


	return height_sum / valid_num;
}

void BingHaiProcess::Check()
{
	//计算数据
	cx_vector_.clear();
	yb_vector_.clear();
	cz_vector_.clear();
	int index_cx = 1;
	int index_yb = 1;
	int index_cz = 1;
	double unvalid_num = Setting::ins().gridProcess_.unvalid_num;

	std::cout << "start memery..." << std::endl;
	flag_cx = boost::shared_ptr<boost::shared_ptr<int>>(new boost::shared_ptr<int>[row], [&](boost::shared_ptr<int>*p){delete[]p; });
	for (int i = 0; i < row; i++)
	{
		std::cout << i << "/" << row << std::endl;
		flag_cx.get()[i] = boost::shared_ptr<int>(new int[column], [&](int* p){delete[]p; });
	}
	std::cout << "end memery..." << std::endl;
	flag_yb = boost::shared_ptr<boost::shared_ptr<int>>(new boost::shared_ptr<int>[row], [&](boost::shared_ptr<int>*p){delete[]p; });
	for (int i = 0; i < row; i++)
	{
		std::cout << i << "/" << row << std::endl;
		flag_yb.get()[i] = boost::shared_ptr<int>(new int[column], [&](int* p){delete[]p; });
		memset(flag_yb.get()[i].get(), 0, sizeof(int) * column);
	}

	std::cout << "start traverse..." << std::endl;
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < column; j++) {
			//std::cout << "index" << i << "," << j << std::endl;
			ChenXian c;
			YongBao y;
			CheZhe z;
			if (cloud_.get()[i*column + j] == unvalid_num)
				continue;
			
			if (cloud_.get()[i*column + j] - GetAroundAvgHeight(i, j) < cx_standard  && flag_cx.get()[i].get()[j] == 0) {
				//初始化
				c.i = i;
				c.j = j;
				z.i = i;
				z.j = j;
				area = 0;
				i_min = i, i_max = i, j_min = j, j_max = j;
				deep = 0;

				mark_cx(i, j, index_cx);

				c.index = index_cx;
				c.area = area;
				c.deep = deep;
				c.length = i_max - i_min + 1;
				if (i_max - i_min + 1 > cz_standard / gridSize_){
					z.index = index_cz;
					index_cz++;
					z.area = area;
					z.deep = deep;
					z.length = i_max - i_min + 1;
					z.width = j_max - j_min + 1;
					cz_vector_.push_back(z);
				}
				c.width = j_max - j_min + 1;
				cx_vector_.push_back(c);
				index_cx++;
			}

			if (cloud_.get()[i*column + j] - GetAroundAvgHeight(i, j) > yb_standard && flag_yb.get()[i].get()[j] == 0){
				y.i = i;
				y.j = j;
				area = 0;
				i_min = i, i_max = i, j_min = j, j_max = j;
				deep = 0;
				mark_yb(i, j, index_yb);
				y.index = index_yb;
				y.area = area;
				y.deep = deep;
				y.length = i_max - i_min + 1;
				y.width = j_max - j_min + 1;
				yb_vector_.push_back(y);
				index_yb++;
			}
		}
	}
}


void BingHaiProcess::mark_cx(int i, int j, int index){
 	//给标记号
	if (cloud_.get()[i*column + j] == Setting::ins().gridProcess_.unvalid_num)
		return;

	flag_cx.get()[i].get()[j] = index;
 	//记录i、j的最值，相减算长宽
 	if (i<i_min){
 		i_min = i;
 	}
 	if (i>i_max){
 		i_max = i;
 	}
 	if (j<j_min){
 		j_min = j;
 	}
 	if (j>j_max){
 		j_max = j;
 	}
 	//记深度
 	if (deep>cloud_.get()[i*column + j]){
		deep = cloud_.get()[i*column + j];
 	}
 	//记面积
 	area++;
 	//int len = row;
 	//递归算法
	if (j + 1 < column && cloud_.get()[i*column + j + 1] - GetAroundAvgHeight(i, j+1) <= cx_standard && flag_cx.get()[i].get()[j + 1] == 0) {
 		mark_cx(i, j + 1, index);
 	}
	if (i + 1 < row && j + 1 < column && cloud_.get()[(i + 1)*column + j + 1] - GetAroundAvgHeight(i+1, j + 1) <= cx_standard && flag_cx.get()[i + 1].get()[j + 1] == 0) {
 		mark_cx(i + 1, j + 1, index);
 	}
	if (i + 1 < row && cloud_.get()[(i + 1)*column + j] - GetAroundAvgHeight(i+1, j) <= cx_standard && flag_cx.get()[i + 1].get()[j] == 0) {
 		mark_cx(i + 1, j, index);
 	}
	if (i + 1 < row && j - 1 >= 0 && cloud_.get()[(i + 1)*column + j - 1] - GetAroundAvgHeight(i+1, j - 1) <= cx_standard && flag_cx.get()[i + 1].get()[j - 1] == 0) {
 		mark_cx(i + 1, j - 1, index);
 	}
	if (j - 1 >= 0 && cloud_.get()[i*column + j - 1] - GetAroundAvgHeight(i, j -1) <= cx_standard && flag_cx.get()[i].get()[j - 1] == 0) {
 		mark_cx(i, j - 1, index);
 	}
	if (i - 1 >= 0 && j - 1>0 && cloud_.get()[(i - 1)*column + j - 1] - GetAroundAvgHeight(i -1, j - 1) <= cx_standard && flag_cx.get()[i - 1].get()[j - 1] == 0){
 		mark_cx(i - 1, j - 1, index);
 	}
	if (i - 1 >= 0 && cloud_.get()[(i - 1)*column + j] - GetAroundAvgHeight(i-1, j) <= cx_standard && flag_cx.get()[i - 1].get()[j] == 0){
 		mark_cx(i - 1, j, index);
 	}
	if (i - 1 >= 0 && j + 1<column && cloud_.get()[(i - 1)*column + j + 1] - GetAroundAvgHeight(i-1, j + 1) <= cx_standard && flag_cx.get()[i - 1].get()[j + 1] == 0){
 		mark_cx(i - 1, j + 1, index);
 	}
 }
 
void BingHaiProcess::mark_yb(int i, int j, int index){
	if (cloud_.get()[i*column + j] == Setting::ins().gridProcess_.unvalid_num)
		return;

	flag_yb.get()[i].get()[j] = index;
 	if (i<i_min){
 		i_min = i;
 	}
 	if (i>i_max){
 		i_max = i;
 	}
 	if (j<j_min){
 		j_min = j;
 	}
 	if (j>j_max){
 		j_max = j;
 	}
	if (deep<cloud_.get()[i*column + j]){
		deep = cloud_.get()[i*column + j];
 	}
 	area++;
 	//int len = row;


	if (j + 1 < column && cloud_.get()[i*column + j + 1] - GetAroundAvgHeight(i, j + 1) >= yb_standard && flag_yb.get()[i].get()[j + 1] == 0) {
		mark_yb(i, j + 1, index);
	}
	if (i + 1 < row && j + 1 < column && cloud_.get()[(i + 1)*column + j + 1] - GetAroundAvgHeight(i + 1, j + 1) >= yb_standard && flag_yb.get()[i + 1].get()[j + 1] == 0) {
		mark_yb(i + 1, j + 1, index);
	}
	if (i + 1 < row && cloud_.get()[(i + 1)*column + j] - GetAroundAvgHeight(i + 1, j) >= yb_standard && flag_yb.get()[i + 1].get()[j] == 0) {
		mark_yb(i + 1, j, index);
	}
	if (i + 1 < row && j - 1 >= 0 && cloud_.get()[(i + 1)*column + j - 1] - GetAroundAvgHeight(i + 1, j - 1) >= yb_standard && flag_yb.get()[i + 1].get()[j - 1] == 0) {
		mark_yb(i + 1, j - 1, index);
	}
	if (j - 1 >= 0 && cloud_.get()[i*column + j - 1] - GetAroundAvgHeight(i, j - 1) >= yb_standard && flag_yb.get()[i].get()[j - 1] == 0) {
		mark_yb(i, j - 1, index);
	}
	if (i - 1 >= 0 && j - 1 > 0 && cloud_.get()[(i - 1)*column + j - 1] - GetAroundAvgHeight(i - 1, j - 1) >= yb_standard && flag_yb.get()[i - 1].get()[j - 1] == 0){
		mark_yb(i - 1, j - 1, index);
	}
	if (i - 1 >= 0 && cloud_.get()[(i - 1)*column + j] - GetAroundAvgHeight(i - 1, j) >= yb_standard && flag_yb.get()[i - 1].get()[j] == 0){
		mark_yb(i - 1, j, index);
	}
	if (i - 1 >= 0 && j + 1 < column && cloud_.get()[(i - 1)*column + j + 1] - GetAroundAvgHeight(i - 1, j + 1) >= yb_standard && flag_yb.get()[i - 1].get()[j + 1] == 0){
		mark_yb(i - 1, j + 1, index);
	}
 }
