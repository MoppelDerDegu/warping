// author : Torben Dittrich
// edited by : Christopher Hipp

#pragma once

#include <opencv\cv.h>
#include <opencv\highgui.h>


class SaliencyMath
{

private:
	CvMat* image_saliency_map;    //stores the histogram-based image saliency map
	CvMat* motion_saliency_map;   //stores the motion map
	CvMat* stereo_saliency_map;	  //stores the stereo saliency map
	CvMat* combined_saliencyMap;  //stores the combined saliency map

	double w_i, w_m, w_d;		  //weights for the different saliency maps

public:
	SaliencyMath(int, int);		  //constructor with height and width of the saliency maps
	~SaliencyMath(void);		  //destructor

	CvMat* computeSaliency();     //builds the combined saliency map

	// getter and setter for the saliency maps
	void setImageSaliencyMap( CvMat* );
	void setMotionSaliencyMap( CvMat* );
	void setStereoSaliencyMap( CvMat* );
	CvMat* getMotionSaliency();
	CvMat* getStereoSaliency();
	CvMat* getImageSaliency();
	
	void computeWeights(int, float);	//computes the weights based on the disparities and the number of colors
};

