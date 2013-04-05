// author : Torben Dittrich
// edited by : Christopher Hipp

#include "StdAfx.h"
#include "DisparityMapBuilder.h"

#include "StereoImage.h"

//opencv includes
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/legacy/legacy.hpp>

using namespace cv;
using namespace std;

DisparityMapBuilder::DisparityMapBuilder(void)
{

}

DisparityMapBuilder::DisparityMapBuilder(CvSize imgSize)
{
	this->gray_left   = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	this->gray_right  = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
	this->saliencyMap = cvCreateMat( imgSize.height, imgSize.width, CV_8UC2 );
	this->percentageOfDepth = 0;
}

DisparityMapBuilder::~DisparityMapBuilder(void)
{
	cvReleaseImage(&this->gray_left);
	cvReleaseImage(&this->gray_right);
	cvReleaseMat(&this->saliencyMap);
}

void DisparityMapBuilder::computeDepthPercentage(int numPixels)
{
	this->percentageOfDepth = 0.;

	//compute how many pixels have a depth value greater than 100
	//these pixels are considered as salient in the depth map
	for(int i=0; i < this->saliencyMap->rows; i++)
	{
		for(int j=0; j < this->saliencyMap->cols; j++)
		{
			if(	cvGetReal2D(this->saliencyMap, i, j) >100 )		
			{
				this->percentageOfDepth += 1.;
			}
		}

	}
	
	this->percentageOfDepth = this->percentageOfDepth/numPixels;

	cout << "PERCENTAGE OF DEPTH: " << this->percentageOfDepth << endl;

}

//Block Matching
CvMat* DisparityMapBuilder::buildDisparityMapBM(StereoImage* source)
{
	cout << " CALCULATING DISPARITY MAP BASED ON BM " << endl;
	
    cvReleaseMat(&this->saliencyMap);
	
	//convert images to gray images as the algorithm expects gray scale images
	cvCvtColor(source->getLeft_eye(), gray_left, CV_BGR2GRAY);
	cvCvtColor(source->getRight_eye(), gray_right, CV_BGR2GRAY);

	Mat mat_left  = gray_left;
	Mat mat_right = gray_right;

	Size size = cvGetSize(gray_left);

	//create the image in which to save the disparities
    Mat imgDisparity16S = Mat( mat_left.rows, mat_left.cols, CV_16S );
    Mat imgDisparity8U = Mat( mat_left.rows, mat_left.cols, CV_8UC2 );

    // calculate the number of disparities
    int numOfDisp;
	if( (size.width/32) % 16 == 0)
		numOfDisp = (size.width/32);
	else
		numOfDisp = ((size.width/32)) + (16-(size.width/32) % 16); 

	// size of the block window; must be odd
    int SADWindowSize = 19;       


	// call the constructor for StereoBM
    StereoBM sbm( StereoBM::BASIC_PRESET,
				  numOfDisp,
				  SADWindowSize );
	
	// calculate the disparity image
    sbm( mat_left, mat_right, imgDisparity16S, CV_16S );
			
	// check its extreme values
    double minVal; double maxVal;

    minMaxLoc( imgDisparity16S, &minVal, &maxVal );

    // convert it into a CV_8UC2 image
    imgDisparity16S.convertTo( imgDisparity8U, CV_8UC2, 255/(maxVal - minVal));

	//remove small depth regions and increase more confident regions
	cvErode(&(CvMat)imgDisparity8U, &(CvMat)imgDisparity8U, 0, 5);
	cvDilate(&(CvMat)imgDisparity8U, &(CvMat)imgDisparity8U, 0, 8);

	this->saliencyMap = cvCloneMat(&(CvMat)imgDisparity8U);

	//compute depth percentage of the disparity map
	computeDepthPercentage(size.height*size.width);

	//clean up
	imgDisparity16S.release();
	mat_left.release();
	mat_right.release();
	imgDisparity8U.release();

	sbm.state.release();
	
	return this->saliencyMap;
}

//Semi Global Block Matching
CvMat* DisparityMapBuilder::buildDisparityMapSGBM(StereoImage* source)
{
	cout << " CALCULATING DISPARITY MAP BASED ON SGBM " << endl;
	
	cvReleaseMat(&this->saliencyMap);

	//convert images to gray images as the algorithm expects gray scale images
	cvCvtColor(source->getLeft_eye(), gray_left, CV_BGR2GRAY);
	cvCvtColor(source->getRight_eye(), gray_right, CV_BGR2GRAY);
	
	Size size = cvGetSize(gray_left);

	Mat mat_left  = gray_left;
	Mat mat_right = gray_right;

	int SADWindowSize = 3; //size of the block window

	int numOfDisp;
	
	//calculate number of disparities
	if( (size.width/32) % 16 == 0)
		numOfDisp = (size.width/32);
	else
		numOfDisp = ((size.width/32)) + (16-(size.width/32) % 16);
    
	StereoSGBM sgbm;
	
	//set state for stereo sgbm algorithm
	sgbm.preFilterCap = 63;
    sgbm.SADWindowSize = SADWindowSize;
	
	int cn = mat_left.channels();

    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = 0;
    sgbm.numberOfDisparities = numOfDisp;
    sgbm.uniquenessRatio = 10;
    sgbm.speckleWindowSize = 100;
    sgbm.speckleRange = 32;
    sgbm.disp12MaxDiff = 1;
    sgbm.fullDP = 1;
    
	//images for the disparity map
	Mat imgDisparity16S;
	Mat imgDisparity8U;

	//compute disparity map
	sgbm(mat_left, mat_right, imgDisparity16S);

	// check its extreme values
	double minVal; double maxVal;

    minMaxLoc( imgDisparity16S, &minVal, &maxVal );
	
	//convert the disparity map to a CV_8U image
	imgDisparity16S.convertTo(imgDisparity8U, CV_8U, 255/(maxVal + minVal));

	//remove small depth regions and increase more confident regions
	cvErode(&(CvMat)imgDisparity8U, &(CvMat)imgDisparity8U, 0, 5);
	cvDilate(&(CvMat)imgDisparity8U, &(CvMat)imgDisparity8U, 0, 8);
	
	this->saliencyMap = cvCloneMat(&(CvMat)imgDisparity8U);

	//compute depth percentage of the disparity map
	computeDepthPercentage(size.height*size.width);
		
	//cleanup
	imgDisparity16S.~Mat();
	mat_left.~Mat();
	mat_right.~Mat();
	sgbm.~StereoSGBM();
	imgDisparity8U.~Mat();
	
	return this->saliencyMap;
}

//Graph Cut
CvMat* DisparityMapBuilder::buildDisparityMapGC(StereoImage* source)
{
	cout << " CALCULATING DISPARITY MAP BASED ON GC " << endl;
		
	cvReleaseMat(&this->saliencyMap);

	//convert images to gray images as the algorithm expects gray scale images
	cvCvtColor(source->getLeft_eye(), gray_left, CV_BGR2GRAY);
	cvCvtColor(source->getRight_eye(), gray_right, CV_BGR2GRAY);

	Size size = cvGetSize(gray_left);

	//create the image in which to save the disparities
	CvMat* disparity_left_16S  = cvCreateMat( size.height, size.width, CV_16S );
	CvMat* disparity_right_16S = cvCreateMat( size.height, size.width, CV_16S ); // not used for further processing

	// calculate the number of disparities
	int numOfDisp;
	if( (size.width/32) % 16 == 0)
		numOfDisp = (size.width/32);
	else
		numOfDisp = ((size.width/32)) + (16-(size.width/32) % 16); 
	
	//compute disparity map
	CvStereoGCState* state = cvCreateStereoGCState( numOfDisp, 2 );
	cvFindStereoCorrespondenceGC( gray_left, gray_right, disparity_left_16S, disparity_right_16S, state, 0 );
	
	//convert it to an 8U image
	CvMat* disparity_left_8U = cvCreateMat( size.height, size.width, CV_8U );
	cvConvertScale( disparity_left_16S, disparity_left_8U, -16 );

	//remove small depth regions and increase more confident regions
	cvErode(disparity_left_8U, disparity_left_8U, 0, 5);
	cvDilate(disparity_left_8U, disparity_left_8U, 0, 8);
	
	this->saliencyMap = cvCloneMat(disparity_left_8U);

	//compute depth percentage of the disparity map
	computeDepthPercentage(size.height*size.width);
	
	//clean up
	cvReleaseMat(&disparity_left_16S);
	cvReleaseMat(&disparity_right_16S);
	cvReleaseMat(&disparity_left_8U);
	cvReleaseStereoGCState(&state);
	
	return this->saliencyMap;
}

float DisparityMapBuilder::getPercentageOfDepth()
{
	return this->percentageOfDepth;
}
