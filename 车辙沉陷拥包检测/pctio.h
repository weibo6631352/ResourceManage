#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Scene_points_with_normal_item.h>


 namespace pct
 {
     namespace io
     {
 		void save_las(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string path);
         void Load_las(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string path);
 
 		void save_las_off(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string path);
 		void Load_las_off(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string path);
 
 		Scene_points_with_normal_item* lasload(const std::string& file_path);
 		bool lassave(const Scene_points_with_normal_item* item, const std::string& file_path);
 
 	}
    
 }
 
