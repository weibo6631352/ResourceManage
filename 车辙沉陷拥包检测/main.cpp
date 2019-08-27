#include <QApplication>
#include <QDialog>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>


#include "pctio.h"
#include "Setting.h"

#include "Common.h"
#include "GroundExtract.h"
#include "GridProcess.h"
#include "BingHaiProcess.h"
#include "CgalClassif.h"


int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	QDialog w;
	pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
	Setting::ins().input_file_ = "C:/Users/wang/Desktop/190429_025950_9_local.las";
	Setting::ins().outdir_ = "C:/Users/wang/Desktop/";
	Setting::ins().zone = 51;
	Setting::ins().southhemi = false;
	Setting::ins().sample_.gridSize = 0.1;    // 影响提取地面速度，不能小于网格尺寸
	Setting::ins().gridProcess_.gridSize = 0.2;
	Setting::ins().liQunDian_.radius = 2;
	Setting::ins().liQunDian_.count = (0.7*Setting::ins().liQunDian_.radius *Setting::ins().liQunDian_.radius*3.14) / (Setting::ins().sample_.gridSize * Setting::ins().sample_.gridSize);
	Setting::ins().groundExact_.minlDistance = 0.5;
	Setting::ins().groundExact_.maxlDistance = 0.5;
	Setting::ins().groundExact_.slope = 1.0;
	Setting::ins().groundExact_.windowsize = 2;


	std::string groundLas = Setting::ins().outdir_ + "ground_.las";
	std::string sampleLas = Setting::ins().outdir_ + "sample_.las";


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr grid_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr binghai_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointIndicesPtr cloud_indices(new pcl::PointIndices);
	pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);

	pct::io::Load_las_off(src_cloud, Setting::ins().input_file_);

	std::cout << "start Sample..." << src_cloud->size() << std::endl;
	Common::Sample(*src_cloud, Setting::ins().sample_.gridSize);
	pct::io::save_las_off(src_cloud, sampleLas);
	std::cout << "end Sample." << src_cloud->size() << std::endl;

	// 提取地面
	GroundExtract::FindGroundIndices(src_cloud, cloud_indices, ground_indices);
	if (ground_indices->indices.size() < 2)
		return EXIT_FAILURE;
	std::cout << "end FindGroundIndices." << std::endl;

	Common::ExtractCloud(src_cloud, ground_indices, ground_cloud);
	pct::io::save_las_off(ground_cloud, groundLas);
 	std::cout << "end FindGroundIndices save_las_off." << std::endl;


 	CgalClassif::classif(groundLas);
 	std::cout << "end classif." <<  std::endl;
 
 	pct::io::Load_las_off(src_cloud, groundLas);
 	Common::ExtractColorCloud(src_cloud);
	Common::JuLeiFilter(src_cloud, 1000);
 	Common::GenerateCloud(src_cloud, ground_indices);
	
 	//建立网格
 	boost::shared_ptr<GridProcess> gp(new GridProcess);
 	gp->GenerateGrid(src_cloud, ground_indices);
 	std::cout << "row:" << gp->row_ << ",col" << gp->col_ << std::endl;
 
 	Common::Grid2Cloud(gp->pointArr_, gp->row_, gp->col_, gp->size_, *grid_cloud);
 	pct::io::save_las_off(grid_cloud, Setting::ins().outdir_ + "grid_.las");
 
 
 	std::cout << "start BingHaiProcess..." << std::endl;
 	BingHaiProcess bh(gp->pointArr_, gp->row_, gp->col_, gp->size_);
 
 	bh.Check();
 	std::cout << "end BingHaiProcess." << std::endl;
 	Common::Grid2Cloud(gp->pointArr_, gp->row_, gp->col_, gp->size_, *binghai_cloud);
 

 
	for (int i = 0; i < bh.cx_vector_.size(); ++i)
	{
		BingHaiProcess::ChenXian &cx = bh.cx_vector_[i];
		std::cout << cx << std::endl;
		pcl::PointXYZRGB pt;
		pt.x = cx.j * gp->size_;
		pt.y = cx.i * gp->size_;
		pt.z = gp->pointArr_.get()[cx.i*bh.column + cx.j]+1;
		pt.r = 255;
		binghai_cloud->push_back(pt);
	}

	for (int i = 0; i < bh.yb_vector_.size(); ++i)
	{
		BingHaiProcess::YongBao &yb = bh.yb_vector_[i];
		std::cout << yb << std::endl;
		pcl::PointXYZRGB pt;
		pt.x = yb.j * gp->size_;
		pt.y = yb.i * gp->size_;
		pt.z = gp->pointArr_.get()[yb.i*bh.column + yb.j] + 1;
		pt.g = 255;
		binghai_cloud->push_back(pt);
	}

	for (int i = 0; i < bh.cz_vector_.size(); ++i)
	{
		BingHaiProcess::CheZhe &cz = bh.cz_vector_[i];
		std::cout << cz << std::endl;
		pcl::PointXYZRGB pt;
		pt.x = cz.j * gp->size_;
		pt.y = cz.i * gp->size_;
		pt.z = gp->pointArr_.get()[cz.i*bh.column + cz.j] + 1;
		pt.b = 255;
		binghai_cloud->push_back(pt);
	}
 
 	pct::io::save_las_off(binghai_cloud, Setting::ins().outdir_ + "bingHai_.las");

	w.exec();
	return a.exec();
}

