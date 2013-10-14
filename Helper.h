#pragma once

#include "stdafx.h"

struct Helper
{
	static IplImage* getNthFrame(CvCapture* capture, int n); // grabs the n-th frame of a video
	static IplImage MatToIplImage(Mat& m);
	static Mat IplImageToMat(IplImage* im);
	static string getImageType(int number); // returns the type of an image as a string, e.g. CV_38FC3
	static void drawMeshOverMat(const Mesh &mesh, Mat &mat);
	static void drawMeshesOverMatStereo(Mesh &leftMesh, Mesh &rightMesh, Mat &mat); // draws the left mesh over the left half of the mat and the right one respectively
	static double stringToDouble(const string &s);
	static int stringToInt(const string &s);
	static Mat meshAsMat(const Mesh &mesh, const Size &s);
	static void getImageROI(Quad &quad, Mat &roi, Mat &img); // returns the a ROI placed over the quad in an image
	static Quad getRelativeCoordinates(Quad &quad); // returns a quad with relative coordinates of the specified quad
	static vector<string> &split(const string &s, char delimiter, vector<string> &result);
	static vector<string> split(const string &s, char delimiter);
	static void matXmat(const Mat &a, const Mat &b, Mat &dest); // normalizes a to 0..1 and then multiplies (i,j) in a with (i, j) in b and writes the product in (i,j) in dest
	static void adjustRightPathlineCoordinates(PathlineSets &rightPathlines, Size &newImageSize); // half the image width is substracted from the x coordinates of the pathlines of the right view
	static void printUsageInstructions();

	template <class T> static bool contains(vector<T> &vec, T &elem); // checks if an element is in a collection
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
