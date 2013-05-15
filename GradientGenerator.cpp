#include "GradientGenerator.h"


GradientGenerator::GradientGenerator(void)
{
	GradientGenerator(1, 0, CV_16S);
}

GradientGenerator::GradientGenerator(int scale, int delta, int ddepth)
{
	this->scale = scale;
	this->delta = delta;
	this->ddepth = ddepth;
}

GradientGenerator::~GradientGenerator(void)
{
}

void GradientGenerator::generateGradient(Mat src, Mat &dest, int mode)
{
	cout << "\nCompute gradient image" << endl;

	if (mode == GRADIENT_SCHARR)
		generateScharrGradient(src, dest);
	else if (mode == GRADIENT_SOBEL)
		generateSobelGradient(src, dest);
	else
		generateSimpleGradient(src, dest);
}

void GradientGenerator::generateSobelGradient(Mat &src, Mat &dest)
{
	Mat srcGray;

	GaussianBlur(src, src, Size(3, 3), 0, 0, BORDER_DEFAULT);
	cvtColor(src, srcGray, CV_RGB2GRAY);

	Mat gradX, gradY;
	Mat absGradX, absGradY;

	// gradient in x direction
	Sobel(srcGray, gradX, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(gradX, absGradX);

	// gradient in y direction
	Sobel(srcGray, gradY, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(gradY, absGradY);

	// combine x-gradient and y-gradient
	addWeighted( absGradX, 0.5, absGradY, 0.5, 0, dest);
}

void GradientGenerator::generateScharrGradient(Mat &src, Mat &dest)
{
	Mat srcGray;

	GaussianBlur(src, src, Size(3, 3), 0, 0, BORDER_DEFAULT);
	cvtColor(src, srcGray, CV_RGB2GRAY);

	Mat gradX, gradY;
	Mat absGradX, absGradY;

	// gradient in x direction
	Scharr(srcGray, gradX, ddepth, 1, 0, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(gradX, absGradX);

	// gradient in y direction
	Scharr(srcGray, gradY, ddepth, 0, 1, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(gradY, absGradY);

	// combine x-gradient and y-gradient
	addWeighted( absGradX, 0.5, absGradY, 0.5, 0, dest);
}

void GradientGenerator::generateSimpleGradient(Mat &src, Mat &dest)
{
	Mat srcGray;

	GaussianBlur(src, srcGray, Size(9, 9), 0, 0, BORDER_DEFAULT);
	cvtColor(srcGray, srcGray, CV_RGB2GRAY);

	dest = Mat::zeros(src.rows, src.cols, CV_8U);

	for (int y = 0; y < srcGray.rows; y++)
	{
		for (int x = 0; x < srcGray.cols; x++)
		{
			uchar value;
			short xdir;
			short ydir;

			if (x - 1 < 0)
				xdir = abs(srcGray.at<uchar> (y, x) - srcGray.at<uchar> (y, x + 1));
			else if (x + 1 == srcGray.cols)
				xdir = abs(srcGray.at<uchar> (y, x) - srcGray.at<uchar> (y, x - 1));
			else
				xdir = abs(srcGray.at<uchar> (y, x + 1) - srcGray.at<uchar> (y, x - 1));

			if (y - 1 < 0)
				ydir = abs(srcGray.at<uchar> (y, x) - srcGray.at<uchar> (y + 1, x));
			else if (y + 1 == srcGray.rows)
				ydir = abs(srcGray.at<uchar> (y, x) - srcGray.at<uchar> (y - 1, x));
			else
				ydir = abs(srcGray.at<uchar> (y + 1, x) - srcGray.at<uchar> (y - 1, x));

			value = (uchar) xdir + ydir;
			dest.at<uchar> (y, x) = value;
		}
	}
}
