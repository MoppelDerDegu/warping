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
#include "StereoSolver.h"
#include "QuadSaliencyManager.h"

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
	int n = 10; // every n-th frame
	int currentFrame = 1;

	CvCapture* input;
	VideoCapture captmp;
	char* fileName = "D:/media/Flower.avi";
	string _fileName = fileName;

	captmp.open(_fileName);
	double totalFrameNumber = captmp.get(CV_CAP_PROP_FRAME_COUNT);
	captmp.release();

	input = cvCaptureFromFile(fileName);
	
	if (!input)
	{
		cerr << "CAN NOT OPEN FILE!" << endl;
		return -1;
	}

	// decode the first frame
	IplImage* img = cvQueryFrame(input);

	if (!img)
	{
		cerr << "THERE IS NO FRAME TO DECODE!" << endl;
		return -1;
	}

	Size originalSize = Size((int) cvGetCaptureProperty(input, CV_CAP_PROP_FRAME_WIDTH), (int) cvGetCaptureProperty(input, CV_CAP_PROP_FRAME_HEIGHT));
	Size newSize(360, 300);

	// initialization
	GradientGenerator gd;
	StereoImageWarper siw;
	StereoSolver ss(20);

	MeshManager* mm = MeshManager::getInstance();
	QuadSaliencyManager* qsm = QuadSaliencyManager::getInstance();

	Size leftSize, rightSize;

	CvMat* combinedSaliency = cvCreateMat( originalSize.width/2, originalSize.height, CV_8UC1);
	
	StereoImage* frame = new StereoImage(originalSize, img->depth, img->nChannels);

	ImageEditor* ie = new ImageEditor(cvSize(originalSize.width/2, originalSize.height), img->depth, img->nChannels);

	DisparityMapBuilder* dmb = new DisparityMapBuilder(cvSize(originalSize.width/2, originalSize.height));
	MotionDetector* md = new MotionDetector();
	SaliencyMath* sm = new SaliencyMath(originalSize.width/2, originalSize.height);
	ImageSaliencyDetector* isd = new ImageSaliencyDetector();

	Mat finalSaliency;

	vector<Mesh> leftDeformedMeshes, rightDeformedMeshes, leftLinearMeshes, rightLinearMeshes;

	vector<pair<float, Quad>> wfMapLeft, wfMapRight;

	pair<Mesh, Mesh> deformedMeshes;

	Mesh initialLeft, initialRight;

	int x = 0;
	bool lastFrameDecoded = false;
	/*
	// deform meshes for every n-th frame and the first and last frame
	while (true)
	{
		if (!img)
			break;

		cout << "\nCurrent Frame: " << currentFrame << endl;

		frame->setBoth_eye(img);
		frame = ie->split_vertical(frame);
		
		leftSize = Size(frame->getLeft_eye()->width, frame->getLeft_eye()->height);
		rightSize = Size(frame->getRight_eye()->width, frame->getRight_eye()->height);

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
		gradient = gradient * 2;

		// saliency weights
		sm->computeWeights(isd->getMaxColorDistance(), dmb->getPercentageOfDepth());

		// initialize mesh for left and right view
		mm->initializeMesh(initialLeft, Size(originalSize.width / 2, originalSize.height));
		initialRight = mm->generateRightEyeMesh(initialLeft, frame, Size(originalSize.width / 2, originalSize.height));

		if (x > 0)
		{
			// every other frame

			combinedSaliency = sm->computeSaliency();
			finalSaliency = combinedSaliency + gradient;

			// assign saliency values to quads of left and right view
			wfMapLeft = qsm->assignSaliencyValuesToQuads(initialLeft, finalSaliency);
			wfMapRight = qsm->assignSaliencyValuesToQuads(initialRight, finalSaliency);
			
			// warp left and right mesh
			deformedMeshes = ss.solveStereoImageProblem(initialLeft, initialRight, originalSize, newSize, wfMapLeft, wfMapRight);

			leftDeformedMeshes.push_back(deformedMeshes.first);
			rightDeformedMeshes.push_back(deformedMeshes.second);

			leftLinearMeshes.push_back(ss.getInitialLeft());
			rightLinearMeshes.push_back(ss.getInitialRight());
		}
		else
		{
			// first frame
			
			Mat dispmat = disp;
			Mat hconmat = &hcon;
			dispmat.convertTo(dispmat, CV_8U);
			hconmat.convertTo(hconmat, CV_8U);
			finalSaliency = dispmat + hconmat + gradient;

			// assign saliency values to quads of left and right view
			wfMapLeft = qsm->assignSaliencyValuesToQuads(initialLeft, finalSaliency);
			wfMapRight = qsm->assignSaliencyValuesToQuads(initialRight, finalSaliency);
			
			// warp left and right mesh
			deformedMeshes = ss.solveStereoImageProblem(initialLeft, initialRight, originalSize, newSize, wfMapLeft, wfMapRight);

			leftLinearMeshes.push_back(ss.getInitialLeft());
			rightLinearMeshes.push_back(ss.getInitialRight());

			leftDeformedMeshes.push_back(deformedMeshes.first);
			rightDeformedMeshes.push_back(deformedMeshes.second);
			
			x++;
			currentFrame = n + 1;
		}

		if (currentFrame < totalFrameNumber)
		{
			cvReleaseCapture(&input);
			input = cvCaptureFromFile(fileName);
			img = Helper::getNthFrame(input, currentFrame);
		}
		else
		{
			if (lastFrameDecoded)
				break;

			cvReleaseCapture(&input);
			input = cvCaptureFromFile(fileName);
			img = Helper::getNthFrame(input, totalFrameNumber - 2);

			lastFrameDecoded = true;
		}
		
		currentFrame = currentFrame + n;
	}

	// write left and right meshes to file
	FileManager::saveMeshesAsText("left meshes.txt", "D:\\warping\\mesh\\", leftDeformedMeshes);
	FileManager::saveMeshesAsText("right meshes.txt", "D:\\warping\\mesh\\", rightDeformedMeshes);
	
	FileManager::saveMeshesAsText("left linear meshes.txt", "D:\\warping\\mesh\\", leftLinearMeshes);
	FileManager::saveMeshesAsText("right linear meshes.txt", "D:\\warping\\mesh\\", rightLinearMeshes);
	*/
	cvReleaseCapture(&input);
	cvReleaseMat(&combinedSaliency);
	
//-------------------------------------------------------------------
//---------INTERPOLATE MESHES AND WARP EVERY SINGLE IMAGE------------
//-------------------------------------------------------------------

	input = cvCaptureFromFile(fileName);

	leftDeformedMeshes.clear();
	rightDeformedMeshes.clear();

	leftLinearMeshes.clear();
	rightLinearMeshes.clear();

	leftDeformedMeshes = FileManager::loadMeshes("D:\\warping\\mesh\\left meshes.txt");
	rightDeformedMeshes = FileManager::loadMeshes("D:\\warping\\mesh\\right meshes.txt");

	leftLinearMeshes = FileManager::loadMeshes("D:\\warping\\mesh\\left linear meshes.txt");
	rightLinearMeshes = FileManager::loadMeshes("D:\\warping\\mesh\\right linear meshes.txt");

	// decode the first frame
	img = cvQueryFrame(input);

	frame = new StereoImage(originalSize, img->depth, img->nChannels);
	frame->setBoth_eye(img);

	ie->split_vertical(frame);

	cvShowImage("bla", siw.warpImage(frame, newSize, leftDeformedMeshes.at(0), rightDeformedMeshes.at(0), leftLinearMeshes.at(0), rightLinearMeshes.at(0)));

	//delete dmb;
	//delete md;
}
#endif