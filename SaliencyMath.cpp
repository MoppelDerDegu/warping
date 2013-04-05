// author : Torben Dittrich
// edited by : Christopher Hipp


#include "StdAfx.h"
#include "SaliencyMath.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;

SaliencyMath::SaliencyMath(int rows, int cols)
{
	this->image_saliency_map   = cvCreateMat(cols, rows, CV_8UC2);
	this->motion_saliency_map  = cvCreateMat(cols, rows, CV_8UC2);
	this->stereo_saliency_map  = cvCreateMat(cols, rows, CV_8UC2);
	this->combined_saliencyMap = cvCreateMat(cols, rows, CV_8UC1);
}


SaliencyMath::~SaliencyMath(void)
{
	cvReleaseMat(&this->image_saliency_map);
	cvReleaseMat(&this->motion_saliency_map);
	cvReleaseMat(&this->stereo_saliency_map);
	cvReleaseMat(&this->combined_saliencyMap);
}

void SaliencyMath::computeWeights(int colDist, float percentageOfDepthPixel)
{
	this->w_m = 0.3;

	if((colDist >= 110 && percentageOfDepthPixel >= 0.05) ||
		(colDist < 110 && percentageOfDepthPixel < 0.05))
	{
		this->w_i = 0.4;
		this->w_d = 0.3;
	}
	else if(colDist >= 110 && percentageOfDepthPixel < 0.05)
	{
		this->w_i = 0.5;
		this->w_d = 0.2;
	}
	else if(colDist < 110){
		if(percentageOfDepthPixel > 0.05 && percentageOfDepthPixel < 0.3)
		{
			this->w_d = 0.62*sqrt(percentageOfDepthPixel) + 0.16;
			this->w_i = 0.7 - this->w_d;
		}
		else if(percentageOfDepthPixel >= 0.3)
		{
			w_d = 0.5;
			w_i = 0.2;
		}
	}

	cout << "w_m= " << w_m << " w_d= " << w_d << " w_i= " << w_i << endl;

}


CvMat* SaliencyMath::computeSaliency()
{
	
	//walk through all pixels and weight their values to create the combined saliency map
	for(int i=0; i < this->image_saliency_map->rows; i++)
	{
		for(int j=0; j < this->image_saliency_map->cols; j++)
		{
			double pixelValue = 0.0;
			
			pixelValue = (cvGetReal2D(this->image_saliency_map, i, j)  * this->w_i) + 
						 (cvGetReal2D(this->stereo_saliency_map, i, j) * this->w_d) + 
						 (cvGetReal2D(this->motion_saliency_map, i, j) * this->w_m);
			
			//setting a threshold of 60
			if(pixelValue > 60)
				cvSetReal2D(this->combined_saliencyMap, i, j, pixelValue);
			else
				cvSetReal2D(this->combined_saliencyMap, i, j, 0.0);
		}

	}
	
	
	return this->combined_saliencyMap;
}

void SaliencyMath::setImageSaliencyMap(CvMat* imgSaliencyMap)
{
	this->image_saliency_map = imgSaliencyMap;
}

void SaliencyMath::setMotionSaliencyMap(CvMat* motionSaliencyMap)
{
	this->motion_saliency_map = motionSaliencyMap;
}

void SaliencyMath::setStereoSaliencyMap(CvMat* stereoSaliencyMap)
{
	this->stereo_saliency_map = stereoSaliencyMap;
}

CvMat* SaliencyMath::getMotionSaliency()
{
	return this->motion_saliency_map;
}

CvMat* SaliencyMath::getStereoSaliency()
{
	return this->stereo_saliency_map;
}

CvMat* SaliencyMath::getImageSaliency()
{
	return this->image_saliency_map;
}