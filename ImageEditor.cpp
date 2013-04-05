// author : Torben Dittrich
// edited by : Christopher Hipp

#include "StdAfx.h"

#include "ImageEditor.h"
#include "StereoImage.h"

#include <iostream> 

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;


ImageEditor::ImageEditor(void)
{
	
}

ImageEditor::ImageEditor(CvSize imgSize, int depth, int channels)
{
	this->both_eye_view  = cvCreateImage(cvSize(imgSize.width*2, imgSize.height),depth,channels);
	this->left_eye_view  = cvCreateImage(imgSize, depth, channels);
	this->right_eye_view = cvCreateImage(imgSize, depth, channels);
	this->blurred_left	 = cvCreateImage(imgSize, depth, channels);
	this->blurred_right	 = cvCreateImage(imgSize, depth, channels);
}


ImageEditor::~ImageEditor(void)
{
	cvReleaseImage(&this->both_eye_view);
	cvReleaseImage(&this->left_eye_view);
	cvReleaseImage(&this->right_eye_view);
	cvReleaseImage(&this->blurred_left);
	cvReleaseImage(&this->blurred_right);
	cvReleaseImage(&this->matchedTemplate);
}

StereoImage* ImageEditor::split_vertical(StereoImage* img)
{
	//Create Left Eye Image
	cvSetImageROI(img->getBoth_eye(), cvRect( 0, 0, (img->getBoth_eye()->width)/2, (img->getBoth_eye()->height) ) );
	cvCopy(img->getBoth_eye(), this->left_eye_view);
	cvResetImageROI( img->getBoth_eye() );

	//Create Right Eye Image
	cvSetImageROI(img->getBoth_eye(), cvRect( (img->getBoth_eye()->width)/2, 0, (img->getBoth_eye()->width)/2, img->getBoth_eye()->height));
	cvCopy(img->getBoth_eye(), this->right_eye_view);
	cvResetImageROI( img->getBoth_eye() );

	img->setLeft_eye(this->left_eye_view);
	img->setRight_eye(this->right_eye_view);

	return img;

}

IplImage* ImageEditor::mergeImages(IplImage* img1, IplImage* img2)
{
	//set the left view to the left half of the complete frame
	cvSetImageROI(both_eye_view, cvRect( 0, 0, img1->width, img1->height ) );
	cvCopy(img1, this->both_eye_view);
	cvResetImageROI( both_eye_view );

	//set the right view to the right half of the complete frame
	cvSetImageROI( both_eye_view, cvRect( both_eye_view->width/2, 0,  img2->width, img2->height ) );
	cvCopy(img2, both_eye_view);
	cvResetImageROI( both_eye_view );

	return both_eye_view;
}

IplImage* ImageEditor::smoothImage(CvMat* combSaliencyMap, StereoImage* frame, int numColors)
{
	cout << "BLURRING FRAME TO ENCODE" << endl;
	
	//initialize some parameters for the contours and matching
	CvRect bndRect = cvRect(0,0,0,0);
	CvPoint pt1, pt2, minloc, maxloc;
	double minval, maxval;

	//remove small artefacts in the combined saliency map, and increase the salient regions
	cvErode(combSaliencyMap, combSaliencyMap, NULL, 6);
	cvDilate(combSaliencyMap, combSaliencyMap, NULL, 16);
	
	//copy the original views as they are needed later
    cvCopy(frame->getLeft_eye(),this->blurred_left);
	cvCopy(frame->getRight_eye(),this->blurred_right);
	
	//create the storage for the contours
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* contour = 0;

	//Find the contours of the salient image regions in the frame.
	cvFindContours( combSaliencyMap, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
	
	//determine size of the blurring window
	int blurringSize;

	if(numColors < 30)
		blurringSize = 35;
	else if(numColors > 300)
		blurringSize = 3;
	else if((-numColors/10 + 33) % 2 == 1 )
		blurringSize = (-numColors/10 + 33);
	else
		blurringSize = (-numColors/10 + 34);

	//blur both views
	cvSmooth(frame->getLeft_eye(), this->blurred_left, CV_GAUSSIAN, blurringSize, blurringSize);
	cvSmooth(frame->getRight_eye(),this->blurred_right, CV_GAUSSIAN, blurringSize, blurringSize);

	//Process each salient contour in the current frame
	for( ; contour != 0; contour = contour->h_next )
	{
		//Get a bounding rectangle around the salient object.
		bndRect = cvBoundingRect(contour, 0);

		//check if the salient region can be expanded a little as long as it is not at the border of the image
		if(bndRect.x > 20 && (bndRect.x + bndRect.width) < (frame->getLeft_eye()->width - 20) 
			&& bndRect.y > 20 && (bndRect.y + bndRect.height) < (frame->getLeft_eye()->height - 20) )
		{
			pt1.x = bndRect.x - 20;
			pt1.y = bndRect.y - 20;
			pt2.x = bndRect.x + bndRect.width + 20;
			pt2.y = bndRect.y + bndRect.height + 20;
		}
		else
		{
			pt1.x = bndRect.x;
			pt1.y = bndRect.y;
			pt2.x = bndRect.x + bndRect.width;
			pt2.y = bndRect.y + bndRect.height;
		}

		//set the salient region in the original left view and its respective blurred image
		cvSetImageROI(frame->getLeft_eye(), cvRect( pt1.x, pt1.y, pt2.x, pt2.y ) );
		cvSetImageROI(this->blurred_left, cvRect( pt1.x, pt1.y, pt2.x, pt2.y ) );
		
		//need to find the salient in region in the right view
		this->matchedTemplate =  cvCreateImage(cvSize((frame->getRight_eye()->width - frame->getLeft_eye()->roi->width + 1), 
														(frame->getRight_eye()->height - frame->getLeft_eye()->roi->height + 1)), IPL_DEPTH_32F, 1);
		
		
		cvMatchTemplate(frame->getRight_eye(), frame->getLeft_eye(), this->matchedTemplate, CV_TM_SQDIFF);

		cvMinMaxLoc(this->matchedTemplate, &minval, &maxval, &minloc, &maxloc, 0);
		
		//set the roi in the original right view and its respective blurred image
		cvSetImageROI(frame->getRight_eye(), cvRect(minloc.x, 
													pt1.y, 
													minloc.x + (pt2.x-pt1.x),
													pt2.y));
		cvSetImageROI(this->blurred_right, cvRect(minloc.x, 
													pt1.y, 
													minloc.x + (pt2.x-pt1.x),
													pt2.y));

		//copy the salient regions in the blurred images
		cvCopy(frame->getLeft_eye(), this->blurred_left);
		cvCopy(frame->getRight_eye(),this->blurred_right);
					
		//reset the rois for the next contour
		cvResetImageROI( frame->getRight_eye() );
		cvResetImageROI( this->blurred_right );
				
		cvResetImageROI( frame->getLeft_eye() );
		cvResetImageROI( this->blurred_left );

		//release the matched template because of memory leaks
		cvReleaseImage( &this->matchedTemplate);
	}

	//merge the edited left and right views
	this->both_eye_view = mergeImages(this->blurred_left, this->blurred_right);
	
	//clean storage
	cvClearMemStorage(storage);
	cvReleaseMemStorage(&storage);
	
	return this->both_eye_view;
}
