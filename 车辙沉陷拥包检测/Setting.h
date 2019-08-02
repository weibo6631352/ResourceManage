#pragma once
#include <vector>

struct Setting
{
	static Setting& ins()
	{
		static Setting s;
		return s;
	}

private:
	Setting();

public:
	std::string input_file_;
	std::string app_path_;
	std::string outdir_;
	std::vector<double> global_offset_;
	std::vector<double> global_geo_offset_;
	unsigned int zone;
	unsigned int southhemi;

	struct GroundExactSetting
	{
		GroundExactSetting()
			:windowsize(2)
			, slope(0.6)
			, minlDistance(0.1)
			, maxlDistance(1)
		{

		}
		int windowsize;
		double slope;
		double minlDistance;
		double maxlDistance;
	} groundExact_;

	struct GridProcessSetting
	{
		GridProcessSetting()
			:gridSize(1), unvalid_num(-10000)
		{

		}
		double gridSize;
		double unvalid_num;
	} gridProcess_;

	struct SampleSetting
	{
		SampleSetting()
			:gridSize(0.2)
		{

		}
		double gridSize;
	} sample_;

	struct LiQunDianSetting
	{
		LiQunDianSetting()
			:radius(3), count(20)
		{

		}
		int count;
		double radius;
	} liQunDian_;

	struct ClassifSetting
	{
		ClassifSetting()
			:scales(3)
		{

		}
		int scales;
	} classif_;


};