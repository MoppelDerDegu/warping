#include "stdafx.h"

#include "MonoImageWarper.h"
#include "Helper.h"
#include "Saliency.h"
#include "ImageSaliencyDetector.h"
#include "GradientGenerator.h"
#include "FileManager.h"
#include "StereoImage.h"
#include "ImageEditor.h"
#include "MeshManager.h"
#include "StereoImageWarper.h"

#include "ImageEditor.h"
#include "DisparityMapBuilder.h"
#include "MotionDetector.h"
#include "SaliencyMath.h"
#include "ImageSaliencyDetector.h"


#if 0

int main(int argc, char* argv[])
{
	//Create a new movie capture object.
	CvCapture* input;
		
	//read the input parameters
	char* fileName = "D:/media/hhi1-1-3.avi";
	char* outputFilename = "D:/media/output";
	char* stereoAlgorithm = "BM";
		
	input = cvCaptureFromFile(fileName);
	
	if (!input)
	{
		cout << "CANNOT OPEN FILE: " << fileName << endl;
		return -1;
	}

	//Get size and frames per second of video frames 
	Size S = Size((int) cvGetCaptureProperty( input, CV_CAP_PROP_FRAME_WIDTH ),   
				  (int) cvGetCaptureProperty( input, CV_CAP_PROP_FRAME_HEIGHT ));

	double fps = cvGetCaptureProperty(input, CV_CAP_PROP_FPS);

	//Assign necessary instances
	IplImage* decodedFrame = cvQueryFrame(input);
	IplImage* outputFrame  = cvCreateImage(S,decodedFrame->depth,decodedFrame->nChannels);
	
	if( !decodedFrame )
	{
		cout << "THERE IS NO FRAME TO BE DECODED" << endl;
		return -1;
	}

//------------------------------------------------------------------------
//--------------------Start Initialization--------------------------------
//------------------------------------------------------------------------
	
	StereoImage* frame = new StereoImage(S, decodedFrame->depth, decodedFrame->nChannels);
	ImageEditor* ie = new ImageEditor(cvSize(S.width/2, S.height), decodedFrame->depth, decodedFrame->nChannels );
	
	ImageSaliencyDetector* isd = new ImageSaliencyDetector();
	DisparityMapBuilder* dmb = new DisparityMapBuilder(cvSize(S.width/2, S.height));
	MotionDetector* md = new MotionDetector();
	SaliencyMath* sm = new SaliencyMath(S.width/2, S.height);
	
	CvMat* combinedSaliency = cvCreateMat( S.width/2, S.height, CV_8UC1);
	VideoWriter outputVideo;

	//Add containter format to the output filename
	char* container = ".avi";
	char output[50];
	sprintf(output, outputFilename);
	strcat(output,container);

	outputVideo.open(output , CV_FOURCC('X','2','6','4'), fps , Size(S.width, S.height), true);

//------------------------------------------------------------------------
//---------------------------Edit Frames----------------------------------
//------------------------------------------------------------------------	
	int x=0;
	for(;;)
	{
		//if there are no more frames, jump out of the for.
		if( !decodedFrame )
		{
			break;
		}

		cout << x << endl;

		frame->setBoth_eye(decodedFrame);

		//split frames verically
		frame = ie->split_vertical(frame);

//------------------------------------------------------------------------
//--------------------Comput Image Saliency-------------------------------
//------------------------------------------------------------------------
		
		CvMat hcon = isd->hContrast(frame->getLeft_eye());
		sm->setImageSaliencyMap(&hcon);
				
//------------------------------------------------------------------------
//--------------------Dectect Motion--------------------------------------
//------------------------------------------------------------------------
		
		sm->setMotionSaliencyMap(md->detectMotion(frame->getLeft_eye()));
		
//------------------------------------------------------------------------
//-------------------Stereo Video Analysis--------------------------------
//------------------------------------------------------------------------
		
		if(strcmp(stereoAlgorithm, "BM") == 0 || strcmp(stereoAlgorithm, "bm") == 0)
			sm->setStereoSaliencyMap(dmb->buildDisparityMapBM(frame));
		else if(strcmp(stereoAlgorithm, "GC") == 0 || strcmp(stereoAlgorithm, "gc") == 0)
			sm->setStereoSaliencyMap(dmb->buildDisparityMapGC(frame));
		else if(strcmp(stereoAlgorithm, "SGBM") == 0 || strcmp(stereoAlgorithm, "sgbm") == 0)
			sm->setStereoSaliencyMap(dmb->buildDisparityMapSGBM(frame));
		else
		{
			cout << "UNDEFINED STEREO CORRESPONDENCE ALGORITHM" << endl;
			return -1;
		}

//------------------------------------------------------------------------
//-------------------Compute Weights--------------------------------------
//------------------------------------------------------------------------

		sm->computeWeights(isd->getMaxColorDistance(), dmb->getPercentageOfDepth());
		
//------------------------------------------------------------------------
//-------------------Compute Saliency Map---------------------------------
//------------------------------------------------------------------------

		if(x>0)
		{
			combinedSaliency = sm->computeSaliency();

			Mat m = combinedSaliency;

			FileManager::saveMat("combinedSaliency.png", "D:\\", m);

			outputFrame = ie->smoothImage(combinedSaliency, frame, isd->getNumColors());

			outputVideo.write(outputFrame);
		}
		else
			outputVideo.write(decodedFrame);
			
		x++;

		//load next frame
		decodedFrame = cvQueryFrame(input);
	}
	
	//clean up
	cvReleaseCapture(&input);
	outputVideo.~VideoWriter();
	
	cvReleaseMat(&combinedSaliency);
	cvReleaseImage(&outputFrame);
	dmb->~DisparityMapBuilder();
	md->~MotionDetector();
	
	//the following cannot be released as the algorithm would crash.
	//no reason for this was found
	//sm->~SaliencyMath();
	//ie->~ImageEditor();
	//cvReleaseImage(&decodedFrame);
	//frame->~StereoImage();


	
	cout << "FINISHED WRITING THE VIDEO FILE" << endl;

	return 0;
}
#endif

#if 1
int main(int argc, char* argv[])
{
	CvCapture* input;
	char* fileName = "D:/media/flymetothemoon.mp4";
	input = cvCaptureFromFile(fileName);

	IplImage* img = Helper::getNthFrame(input, 10);

	Size originalSize = Size((int) cvGetCaptureProperty(input, CV_CAP_PROP_FRAME_WIDTH), (int) cvGetCaptureProperty(input, CV_CAP_PROP_FRAME_HEIGHT));
	Size newSize(1280, 480);

	// initialization
	GradientGenerator gd;
	StereoImageWarper siw;
	MeshManager* mm = MeshManager::getInstance();
	StereoImage* frame = new StereoImage(originalSize, img->depth, img->nChannels);
	ImageEditor* ie = new ImageEditor(cvSize(originalSize.width/2, originalSize.height), img->depth, img->nChannels);
	CvMat* combinedSaliency = cvCreateMat(originalSize.width/2, originalSize.height, CV_8UC1);
	DisparityMapBuilder* dmb = new DisparityMapBuilder(cvSize(originalSize.width/2, originalSize.height));
	MotionDetector* md = new MotionDetector();
	SaliencyMath* sm = new SaliencyMath(originalSize.width/2, originalSize.height);
	ImageSaliencyDetector* isd = new ImageSaliencyDetector();

	frame->setBoth_eye(img);
	frame = ie->split_vertical(frame);

	Size leftSize = Size(frame->getLeft_eye()->width, frame->getLeft_eye()->height);
	Size rightSize = Size(frame->getRight_eye()->width, frame->getRight_eye()->height);
	/*
	Mesh left;
	mm->initializeMesh(left, leftSize);

	Mesh right = mm->generateRightEyeMesh(left, frame, rightSize);

	// compute saliency
	Mat saliencyMap = isd.hContrast(img);

	// compute gradient
	Mat gradient;
	gg.generateGradient(src, gradient);
	gradient = gradient * 3;

	// combine saliency and gradient
	Mat combined;
	Helper::matXmat(saliencyMap, gradient, combined);
	*/

	// compute image saliency
	CvMat hcon = isd->hContrast(frame->getLeft_eye());
	sm->setImageSaliencyMap(&hcon);

	// compute motion saliency
	CvMat* motion = md->detectMotion(frame->getLeft_eye());
	sm->setMotionSaliencyMap(motion);

	// compute disparity map
	CvMat* disp = dmb->buildDisparityMapBM(frame);
	sm->setStereoSaliencyMap(disp);

	// compute gradient map
	Mat gradient;
	gd.generateGradient(frame->getLeft_eye(), gradient);

	// combine saliency maps
	Mat final;
	Mat dispmat = disp;
	Mat hconmat = &hcon;
	dispmat.convertTo(dispmat, CV_8U);
	hconmat.convertTo(hconmat, CV_8U);
	final = dispmat + hconmat + gradient;

	FileManager::saveMat("combined saliency.png", "D:\\warping\\saliency\\", final);

	// warp the stereo frame
	siw.warpImage(frame, newSize, final);

	cvReleaseCapture(&input);
	cvReleaseMat(&combinedSaliency);
	delete dmb;
	delete md;
}
#endif