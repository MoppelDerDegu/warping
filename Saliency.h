// author : Ming-Ming Cheng
// paper: Global Contrast based Salient Region Detection

#pragma once

#include "stdafx.h"

struct Saliency
{
	// Region Contrast 
	static Mat GetRC(const Mat &img3f);
	static Mat GetRC(const Mat &img3f, double sigmaDist, double segK, int segMinSize, double segSigma);

	// Color quantization
	static int Quantize(const Mat& img3f, Mat &idx1i, Mat &_color3f, Mat &_colorNum, double ratio = 0.95);

private:
	struct Region{
		Region() { pixNum = 0;}
		int pixNum;  // Number of pixels
		vector<pair<float, int>> freIdx;  // Frequency of each color and its index
		Point2d centroid;
	};
	static void BuildRegions(const Mat& regIdx1i, vector<Region> &regs, const Mat &colorIdx1i, int colorNum);
	static void RegionContrast(const vector<Region> &regs, const Mat &color3fv, Mat& regSal1d, double sigmaDist);
};
