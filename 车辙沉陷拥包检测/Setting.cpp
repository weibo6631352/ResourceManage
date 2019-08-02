#include "Setting.h"
#include <QApplication>


Setting::Setting()
	: global_offset_(3)
	, global_geo_offset_(3)
	, zone(0)
	, southhemi(false)

{
	app_path_ = QApplication::applicationDirPath().toLocal8Bit().data();
}
