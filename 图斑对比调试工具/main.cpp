#include <QtCore/QCoreApplication>



/*
PerceptualDiff - a program that compares two images using a perceptual metric
based on the paper :
A perceptual metric for production testing. Journal of graphics tools,
9(4):33-40, 2004, Hector Yee
Copyright (C) 2006-2011 Yangli Hector Yee
Copyright (C) 2011-2016 Steven Myint, Jeff Terrace

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include "FreeImage.h"
#pragma comment(lib, "FreeImage.lib")
#include "compare_args.h"
#include "lpyramid.h"
#include "metric.h"
#include "rgba_image.h"
#include <QtWidgets/QApplication>
#include <cstdlib>
#include <ciso646>
#include <iostream>
#include <string>
#include <QPixmap>
#include <QDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QGridLayout>
#include <QCheckBox>
#include <QLineEdit>
#include <QObject>
#include <QPushButton>
#include <QMessageBox>
#include <QHBoxLayout>
#include <dbscan.h>
#include <QScrollArea>

QLineEdit *g_fov_value;
QLineEdit *g_threshold_value;
QLineEdit *g_gamma_value;
QLineEdit *g_luminance_value;
QCheckBox *g_luminance_only;
QLineEdit *g_color_factor;
QLineEdit *g_dbscaneps;
QLineEdit *g_dbscanmin;
QLineEdit *g_cusmin;
QLineEdit *g_areamin_value;

const int g_showpic_width = 720;

QPixmap ImgMargin(QPixmap &src, QPixmap &mark);
void miduJulei(QImage &src, int dim, int dbscaneps, int dbscanmin/*, std::vector<std::vector<std::pair<int, int>>>& clusters*/);
void lianTong(QImage &src, int dim, int dbscaneps, int dbscanmin, int mincus);
void lianTong(QImage &src, QImage &midu, int dim, int dbscaneps, int dbscanmin, int mincus, int minArea);

void InitParamWidget(QGridLayout *layout, pdiff::CompareArgs &args)
{
	QLabel *fov_label = new QLabel(QStringLiteral("fov[0.1, 89.9] (default: 45.0)："));
	QLineEdit *fov_value = new QLineEdit(QString::number(args.parameters_.field_of_view));
	layout->addWidget(fov_label, 0, 0);
	layout->addWidget(fov_value, 0, 1);

	QLabel *threshold_label = new QLabel(QStringLiteral("threshold："));
	QLineEdit *threshold_value = new QLineEdit(QString::number(args.parameters_.threshold_pixels));
	layout->addWidget(threshold_label, 0, 3);
	layout->addWidget(threshold_value, 0, 4);


	QLabel *color_factor_label = new QLabel(QStringLiteral("color_factor [0.0, 1.0] (default: 1.0)："));
	QLineEdit *color_factor_value = new QLineEdit(QString::number(args.parameters_.color_factor));
	layout->addWidget(color_factor_label, 0, 6);
	layout->addWidget(color_factor_value, 0, 7);

	QLabel *gamma_label = new QLabel(QStringLiteral("gamma(default: 2.2)："));
	QLineEdit *gamma_value = new QLineEdit(QString::number(args.parameters_.gamma));
	layout->addWidget(gamma_label, 1, 0);
	layout->addWidget(gamma_value, 1, 1);

	QLabel *luminance_label = new QLabel(QStringLiteral("luminance(default: 100.0)："));
	QLineEdit *luminance_value = new QLineEdit(QString::number(args.parameters_.luminance));
	layout->addWidget(luminance_label, 1, 3);
	layout->addWidget(luminance_value, 1, 4);


	QCheckBox *luminance_only = new QCheckBox(QStringLiteral("luminance_only"));
	luminance_only->setChecked(args.parameters_.luminance_only);
	layout->addWidget(luminance_only, 1, 6);


	QLabel *dbscaneps_label = new QLabel(QStringLiteral("dbscaneps："));
	QLineEdit *dbscaneps_value = new QLineEdit(QString::number(5));
	layout->addWidget(dbscaneps_label, 2, 0);
	layout->addWidget(dbscaneps_value, 2, 1);

	QLabel *dbscanmin_label = new QLabel(QStringLiteral("dbscanmin："));
	QLineEdit *dbscanmin_value = new QLineEdit(QString::number(25));
	layout->addWidget(dbscanmin_label, 2, 3);
	layout->addWidget(dbscanmin_value, 2, 4);

	QLabel *cusmin_label = new QLabel(QStringLiteral("cusmin： (useless:-1)"));
	QLineEdit *cusmin_value = new QLineEdit(QString::number(50));
	layout->addWidget(cusmin_label, 2, 6);
	layout->addWidget(cusmin_value, 2, 7);

	QLabel *areamin_label = new QLabel(QStringLiteral("areamin： (useless:-1)"));
	QLineEdit *areamin_value = new QLineEdit(QString::number(100));
	layout->addWidget(areamin_label, 3, 0);
	layout->addWidget(areamin_value, 3, 1);

	layout->setColumnStretch(2, 1);
	layout->setColumnStretch(5, 1);

	g_fov_value = fov_value;
	g_threshold_value = threshold_value;
	g_gamma_value = gamma_value;
	g_luminance_value = luminance_value;
	g_luminance_only = luminance_only;
	g_color_factor = color_factor_value;
	g_dbscaneps = dbscaneps_value;
	g_dbscanmin = dbscanmin_value;
	g_cusmin = cusmin_value;
	g_areamin_value = areamin_value;
}

void GetWidgetParam(pdiff::CompareArgs &args)
{
	args.parameters_.field_of_view = g_fov_value->text().toFloat();
	args.parameters_.threshold_pixels = g_threshold_value->text().toUInt();
	args.parameters_.gamma = g_gamma_value->text().toFloat();
	args.parameters_.luminance = g_luminance_value->text().toFloat();
	args.parameters_.luminance_only = g_luminance_only->isChecked();
	args.parameters_.color_factor = g_color_factor->text().toFloat();
}



int main(int argc, char *argv[])
{
	QApplication a(argc, argv);


	pdiff::CompareArgs args(argc, argv);

	QDialog w;
	Qt::WindowFlags flags = w.windowFlags();

	flags |= Qt::WindowMinimizeButtonHint;
	flags |= Qt::WindowMaximizeButtonHint;
	w.setWindowFlags(flags);

	w.resize(1080, 720);
	QVBoxLayout *hLayout = new QVBoxLayout(&w);

	QGridLayout *gLayout = new QGridLayout;
	QWidget *parmaArea = new QWidget;
	parmaArea->setLayout(gLayout);
	hLayout->addWidget(parmaArea);

	InitParamWidget(gLayout, args);

	QPushButton *run = new QPushButton(QStringLiteral("执行"));
	QPushButton * show = new QPushButton(QStringLiteral("显示/隐藏"));
	QHBoxLayout * caozuo = new QHBoxLayout();
	caozuo->addStretch(1);
	caozuo->addWidget(run);
	caozuo->addStretch(1);
	caozuo->addWidget(show);
	caozuo->addStretch(1);
	hLayout->addLayout(caozuo);

	
	QScrollArea *scrollArea = new QScrollArea();
	scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
	QWidget * pic_widget = new QWidget();
	scrollArea->setWidget(pic_widget);
	scrollArea->setWidgetResizable(true);
	scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
	QVBoxLayout *scrollBox = new QVBoxLayout(pic_widget);
	

	QLabel img_a, img_b, res_c1, res_c2, res_midu_c1, res_midu_c2, res_jl_c1, res_jl_c2;
	QHBoxLayout *src_pic_layout = new QHBoxLayout;
	src_pic_layout->addWidget(&img_a);
	QLabel *src_label = new QLabel(QStringLiteral("A：\t原始图像\t：B"));
	src_label->setAlignment(Qt::AlignCenter);
	src_pic_layout->addWidget(src_label);
	src_pic_layout->addWidget(&img_b);
	scrollBox->addLayout(src_pic_layout);

	QHBoxLayout *dst_pic_layout = new QHBoxLayout;
	dst_pic_layout->addWidget(&res_c1);
	QLabel *dst_label = new QLabel(QStringLiteral("A：\t感知差异\t：B"));
	dst_label->setAlignment(Qt::AlignCenter);
	dst_pic_layout->addWidget(dst_label);
	dst_pic_layout->addWidget(&res_c2);
	scrollBox->addLayout(dst_pic_layout);

	QHBoxLayout *midu_pic_layout = new QHBoxLayout;
	midu_pic_layout->addWidget(&res_midu_c1);
	QLabel *midu_label = new QLabel(QStringLiteral("A：\t密度过滤\t：B"));
	midu_label->setAlignment(Qt::AlignCenter);
	midu_pic_layout->addWidget(midu_label);
	midu_pic_layout->addWidget(&res_midu_c2);
	scrollBox->addLayout(midu_pic_layout);

	QHBoxLayout *jl_pic_layout = new QHBoxLayout;
	jl_pic_layout->addWidget(&res_jl_c1);
	QLabel *jl_label = new QLabel(QStringLiteral("A：\t连通过滤\t：B"));
	jl_label->setAlignment(Qt::AlignCenter);
	jl_pic_layout->addWidget(jl_label);
	jl_pic_layout->addWidget(&res_jl_c2);
	scrollBox->addLayout(jl_pic_layout);


	pic_widget->setLayout(scrollBox);
	hLayout->addWidget(scrollArea);




	QObject::connect(show, &QPushButton::clicked, [&](){
		parmaArea->setHidden(!parmaArea->isHidden());
	});
	QObject::connect(run, &QPushButton::clicked, [&](){
		try
		{
			GetWidgetParam(args);
			if (args.verbose_)
			{
				args.print_args();
			}

			std::string reason;
			float error_sum = 0;
			const auto passed = pdiff::yee_compare(
				*args.image_a_,
				*args.image_b_,
				args.parameters_,
				nullptr,
				&error_sum,
				&reason,
				args.image_difference_.get(),
				args.verbose_ ? &std::cout : nullptr);

			if (passed)
			{
				if (args.verbose_)
				{
					std::cout << "PASS: " + reason;
				}
			}
			else
			{
				std::cout << "FAIL: " + reason;
			}

			if (args.sum_errors_)
			{
				const auto normalized =
					error_sum /
					(args.image_a_->get_width() *
					args.image_a_->get_height() * 255.);

				std::cout << error_sum << " error sum\n";
				std::cout << normalized << " normalzied error sum\n";
			}

			if (args.image_difference_.get())
			{
				args.image_difference_->write_to_file(args.image_difference_->get_name());

				std::cerr << "Wrote difference image to "
					<< args.image_difference_->get_name()
					<< "\n";
			}

			QPixmap res_pix(QString::fromLocal8Bit(args.out_img_.c_str()));
			QImage res_jl_img = res_pix.toImage();
			QImage res_midu_img = res_jl_img;
			std::cout << "miduJulei(res_jl_img, 2, " << g_dbscaneps->text().toInt() << ", " << g_dbscanmin->text().toInt() << ");" << std::endl;
			lianTong(res_jl_img, res_midu_img, 2, g_dbscaneps->text().toInt(), g_dbscanmin->text().toInt(), g_cusmin->text().toInt(), g_areamin_value->text().toInt());
			res_jl_img = res_jl_img.scaledToWidth(g_showpic_width);
			QPixmap res_jl_pix = QPixmap::fromImage(res_jl_img);
			QPixmap res_midu_pix = QPixmap::fromImage(res_midu_img);


			QPixmap img1;
			img1.load(QString::fromLocal8Bit(args.img1_.c_str()));
			img1 = img1.scaledToWidth(g_showpic_width);
			img_a.setPixmap(img1);

			QPixmap img2;
			img2.load(QString::fromLocal8Bit(args.img2_.c_str()));
			img2 = img2.scaledToWidth(g_showpic_width);
			img_b.setPixmap(img2);

			QPixmap img3(res_pix);
			img3 = img3.scaledToWidth(g_showpic_width);
			img3 = ImgMargin(img1, img3);
			res_c1.setPixmap(img3);


			QPixmap img4(res_pix);
			img4 = img4.scaledToWidth(g_showpic_width);
			img4 = ImgMargin(img2, img4);
			res_c2.setPixmap(img4);

			QPixmap img5(res_midu_pix);
			img5 = img5.scaledToWidth(g_showpic_width);
			img5 = ImgMargin(img1, img5);
			res_midu_c1.setPixmap(img5);


			QPixmap img6(res_midu_pix);
			img6 = img6.scaledToWidth(g_showpic_width);
			img6 = ImgMargin(img2, img6);
			res_midu_c2.setPixmap(img6);

			QPixmap img7(res_jl_pix);
			img7 = img7.scaledToWidth(g_showpic_width);
			img7 = ImgMargin(img1, img7);
			res_jl_c1.setPixmap(img7);


			QPixmap img8(res_jl_pix);
			img8 = img8.scaledToWidth(g_showpic_width);
			img8 = ImgMargin(img2, img8);
			res_jl_c2.setPixmap(img8);

			//w.resize(1500, w.size().height());
			/*	return passed ? EXIT_SUCCESS : EXIT_FAILURE;*/
		}
		catch (const pdiff::ParseException &exception)
		{
			std::cerr << exception.what() << "\n";
			return EXIT_FAILURE;
		}
		catch (const pdiff::RGBImageException &exception)
		{
			std::cerr << exception.what() << "\n";
			return EXIT_FAILURE;
		}
	});



	w.exec();


	return a.exec();
}

QPixmap ImgMargin(QPixmap &src, QPixmap &mark)
{
	QImage src_img = src.toImage();
	QImage mrg_img = mark.toImage();
	for (int i = 0; i < mrg_img.width(); ++i)
	{
		for (int j = 0; j < mrg_img.height(); ++j)
		{
			QColor rgb(mrg_img.pixel(i, j));

			if (rgb.blue() != 0 && rgb.blue() != 0 && rgb.blue() != 0)
			{
				src_img.setPixelColor(i, j, rgb);
			}
		}
	}

	return QPixmap::fromImage(src_img);
}


void miduJulei(QImage &src, int dim, int dbscaneps, int dbscanmin/*, std::vector<std::vector<std::pair<int, int>>>& clusters*/)
{
	struct vec2f {
		float data[2];
		vec2f(float x, float y){ data[0] = x; data[1] = y; };
		float operator[](int idx) const { return data[idx]; }
	};
	auto dbscan = DBSCAN<vec2f, float>();
	auto data = std::vector<vec2f>();
	for (int i = 0; i < src.width(); ++i)
		for (int j = 0; j < src.height(); ++j)
		{
			QColor rgb = src.pixelColor(i, j);
			if (rgb.blue() != 0 && rgb.blue() != 0 && rgb.blue() != 0)
				data.push_back(vec2f(i, j));
		}

	std::cout << "dbscan.Run(&data, " << dim << ", " << dbscaneps << ", " << dbscanmin << ");" << std::endl;
	//参数：数据， 维度（二维）， 考虑半径， 聚类最小
	dbscan.Run(&data, dim, dbscaneps, dbscanmin);
	std::cout << "dbscan.end" << std::endl;
	int cus_count = 0;
	src.fill(Qt::black);
	for (int i = 0; i < dbscan.Clusters.size(); ++i)
	{
		for (int j = 0; j < dbscan.Clusters[i].size(); ++j)
		{
			int ind = dbscan.Clusters[i][j];
			int xx = (int)data[ind][0];
			int yy = (int)data[ind][1];
			++cus_count;
			src.setPixelColor(xx, yy, Qt::blue);
		}
	}
	std::cout << "data.size = " << data.size() << "cus_count = " << cus_count << std::endl;
}

unsigned int GetArea(QImage &src)
{
	unsigned int maxx = 0;
	unsigned int maxy = 0;
	unsigned int minx = std::numeric_limits<unsigned int>::max();
	unsigned int miny = minx;
	for (int i = 0; i < src.width(); ++i)
		for (int j = 0; j < src.height(); ++j)
		{
			if (src.pixelColor(i, j).blue() == 255)
			{
				if (i < minx)
					minx = i;
				if (i > maxx)
					maxx = i;
				if (j < miny)
					miny = j;
				if (j > maxy)
					maxy = j;
			}
		}

	unsigned int area = (maxy - miny) * (maxx - minx);
	return area;
}

void lianTong(QImage &src, QImage &midu, int dim, int dbscaneps, int dbscanmin, int mincus, int minArea)
{
	QImage src_bak = src;
	src_bak.fill(Qt::black);
	struct vec2f {
		float data[2];
		vec2f(float x, float y){ data[0] = x; data[1] = y; };
		float operator[](int idx) const { return data[idx]; }
	};
	auto dbscan = DBSCAN<vec2f, float>();
	auto data = std::vector<vec2f>();
	for (int i = 0; i < src.width(); ++i)
		for (int j = 0; j < src.height(); ++j)
		{
			QColor rgb = src.pixelColor(i, j);
			if (rgb.blue() != 0 && rgb.blue() != 0 && rgb.blue() != 0)
				data.push_back(vec2f(i, j));
		}

	std::cout << "dbscan.Run(&data, " << dim << ", " << dbscaneps << ", " << dbscanmin << ");" << std::endl;
	//参数：数据， 维度（二维）， 考虑半径， 聚类最小
	dbscan.Run(&data, dim, dbscaneps, dbscanmin);
	std::cout << "dbscan.end" << std::endl;
	int cus_count = 0;
	src.fill(Qt::black);
	midu.fill(Qt::black);
	for (int i = 0; i < dbscan.Clusters.size(); ++i)
	{
		QImage area_img = src_bak;
		for (int j = 0; j < dbscan.Clusters[i].size(); ++j)
		{
			int ind = dbscan.Clusters[i][j];
			int xx = (int)data[ind][0];
			int yy = (int)data[ind][1];
			midu.setPixelColor(xx, yy, Qt::blue);
			area_img.setPixelColor(xx, yy, Qt::blue);
		}

		int curCount = dbscan.Clusters[i].size();

		
		if (mincus != -1 && curCount < mincus)
		{
			continue;
		}

		unsigned int curArea = GetArea(area_img);
		std::cout << "area" << curArea << std::endl;
		if (minArea != -1 && curArea < minArea)
		{
			continue;
		}

		cus_count += curCount;
		for (int j = 0; j < dbscan.Clusters[i].size(); ++j)
		{
			int ind = dbscan.Clusters[i][j];
			int xx = (int)data[ind][0];
			int yy = (int)data[ind][1];
			src.setPixelColor(xx, yy, Qt::blue);
		}
	}
	std::cout << "data.size = " << data.size() << "cus_count = " << cus_count << std::endl;
}