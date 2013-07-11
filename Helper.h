#pragma once

#include "stdafx.h"

struct Helper
{
	static IplImage* getNthFrame(CvCapture* capture, int n);
	static IplImage MatToIplImage(Mat& m);
	static Mat IplImageToMat(IplImage* im);
	static string getImageType(int number);
	static void drawMeshOverMat(const Mesh &mesh, Mat &mat);
	static double stringToDouble(const string &s);
	static int stringToInt(const string &s);
	static Mat meshAsMat(const Mesh &mesh, const Size &s);
	static void getImageROI(Quad &quad, Mat &roi, Mat &img);
	static Quad getRelativeCoordinates(Quad &quad);
	static vector<string> &split(const string &s, char delimiter, vector<string> &result);
	static vector<string> split(const string &s, char delimiter);
	// normalizes a to 0..1 and then multiplies (i,j) in a with (i, j) in b and writes the product in (i,j) in dest
	static void matXmat(const Mat &a, const Mat &b, Mat &dest);

	template <class T> static bool contains(vector<T> &vec, T &elem);
};

template <class T> bool Helper::contains(vector<T> &vec, T &elem)
{
	for (unsigned int i = 0; i < vec.size(); i++)
	{
		if (elem == vec.at(i))
			return true;
	}

	return false;
}
