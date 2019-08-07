#include "CgalClassif.h"
#include <string>
#include <iostream>

#include <boost/typeof/typeof.hpp>
#include "Setting.h"
#include "pctio.h"
#include <Point_set_item_classification.h>

CgalClassif::CgalClassif()
{
}


CgalClassif::~CgalClassif()
{
}
 
bool CgalClassif::classif(std::string inputfile)
 {
 	int nb_scales = 3;
 	int method = 0;
 
 	std::string labelname_traverse;
 	std::string config_xml = Setting::ins().app_path_ + "/config.xml";
 
 	std::cout << "加载simple.las..." << std::endl;
 	boost::shared_ptr<Scene_points_with_normal_item> scene_item(pct::io::lasload(inputfile));
 	if (scene_item)
 	{
 		// 计算特征
 		std::cout << "计算特征..." << std::endl;
 		boost::shared_ptr<Point_set_item_classification> classifyy(new Point_set_item_classification(scene_item.get()));
 		classifyy->compute_features(3);
 
 		// 添加label
 		std::cout << "添加label..." << std::endl;
 		boost::property_tree::ptree pt;
 		boost::property_tree::xml_parser::read_xml(config_xml, pt);
 		BOOST_AUTO(labels, pt.get_child("classification.labels"));
		for (BOOST_AUTO(label, labels.begin()); label != labels.end(); ++label)
		{
			labelname_traverse = label->second.get<std::string>("name");
			unsigned int col = 0;
			if (labelname_traverse == std::string("ground"))
				col = (unsigned int)-1;
			classifyy->add_new_label(labelname_traverse.c_str(), col);
		}


		// 分类
		std::cout << "load_config..." << std::endl;
		classifyy->load_config(config_xml.c_str(), 0);
		std::cout << "分类..." << std::endl;
		classifyy->run(method, 0, 16, 0.5);
		std::cout << "保存..." << std::endl;
		std::string outpath = inputfile;
		std::cout << outpath << std::endl;
 		pct::io::lassave(scene_item.get(), outpath);
 	}
 	return true;
 }