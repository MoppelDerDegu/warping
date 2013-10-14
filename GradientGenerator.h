#include "stdafx.h"

#pragma once
class GradientGenerator
{
public:
	GradientGenerator(void);
	GradientGenerator(int scale, int delta, int ddepth);
	~GradientGenerator(void);
	void generateGradient(Mat src, Mat &dest, int mode = GRADIENT_SIMPLE); // creates a gradient map with the specified mode
private:
	int scale; // used for sobel and scharr gradient method
	int delta; // used for sobel and scharr gradient method
	int ddepth; // used for sobel and scharr gradient method
	void generateSobelGradient(Mat &src, Mat &dest); // creates gradient based on the sobel operator
	void generateScharrGradient(Mat &src, Mat &dest); // creates gradient based on the sobel operator
	void generateSimpleGradient(Mat &src, Mat &dest); // creates gradient based on the sobel operator
};

