#include "stdafx.h"

#pragma once
class GradientGenerator
{
public:
	GradientGenerator(void);
	GradientGenerator(int scale, int delta, int ddepth);
	~GradientGenerator(void);
	void generateGradient(Mat src, Mat &dest, int mode);
private:
	int scale;
	int delta;
	int ddepth;
	void generateSobelGradient(Mat &src, Mat &dest);
	void generateScharrGradient(Mat &src, Mat &dest);
	void generateSimpleGradient(Mat &src, Mat &dest);
};

