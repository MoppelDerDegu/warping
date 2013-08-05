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
#include "PathlineTracker.h"
#include "PathlineManager.h"
#include "PathlineOptimizer.h"
#include "StereoPathlineSolver.h"

#include "ImageEditor.h"
#include "DisparityMapBuilder.h"
#include "MotionDetector.h"
#include "SaliencyMath.h"
#include "ImageSaliencyDetector.h"

#if 1
int main(int argc, char* argv[])
{
	int n = 10; // every n-th frame
	int currentFrame = 1;

	CvCapture* input;
	char* fileName = "D:/media/Flower.avi";
	char* outputFilenameInterpolated = "D:/warping/result/interpolated output";
	char* outputFilenameFinal = "D:/warping/result/final";
	char* container = ".avi";
	char output[50];
	string _fileName = fileName;

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

	int totalFrameNumber = (int) cvGetCaptureProperty(input, CV_CAP_PROP_FRAME_COUNT);

	Size originalSize = Size((int) cvGetCaptureProperty(input, CV_CAP_PROP_FRAME_WIDTH), (int) cvGetCaptureProperty(input, CV_CAP_PROP_FRAME_HEIGHT));
	Size newSize(360, 300);

//------------------------------------------------------------------------
//--------------------Start Initialization--------------------------------
//------------------------------------------------------------------------

	GradientGenerator gd;
	StereoImageWarper siw;
	StereoSolver ss(20);

	MeshManager* mm = MeshManager::getInstance();
	QuadSaliencyManager* qsm = QuadSaliencyManager::getInstance();
	PathlineManager* plm = PathlineManager::getInstance();

	Size leftSize, rightSize;

	PathlineAdjacencies adjacencies;

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

//-------------------------------------------------------------------
//---------DEFORM AND OPTIMIZE MESHES FOR EVERY N-TH FRAME-----------
//-------------------------------------------------------------------

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

	// clean up
	cvReleaseCapture(&input);
	//cvReleaseMat(&combinedSaliency);
	//delete dmb;
	//delete md;
	delete frame;


//-------------------------------------------------------------------
//---------TRACK PATHLINES IN THE ORIGINAL VIDEO---------------------
//-------------------------------------------------------------------
	
	input = cvCaptureFromFile(fileName);
	
	//PathlineTracker originalTracker(input);

	//originalTracker.trackPathlines();
	//PathlineSets originalPathlines = originalTracker.getPathlineSets();

	//FileManager::savePathlines("original pathlines.txt", "D:\\warping\\pathlines\\", originalPathlines.pathlines.at(0));

	vector<Pathline> _originalPathlines = FileManager::loadPathlines("D:\\warping\\pathlines\\original pathlines.txt");
	PathlineSets originalPathlines;
	originalPathlines.pathlines.push_back(_originalPathlines);

	// determine the pathline adjacencies
	Size seedSize = Size(originalSize.width / 2, originalSize.height);
	Mesh originalSeedMesh;
	mm->initializeMesh(originalSeedMesh, seedSize);
	plm->getAdjacencies(originalPathlines, originalSeedMesh, seedSize, adjacencies);

	cvReleaseCapture(&input);

//-------------------------------------------------------------------
//---------INTERPOLATE MESHES AND WARP EVERY SINGLE IMAGE------------
//-------------------------------------------------------------------
	/*
	input = cvCaptureFromFile(fileName);
	VideoWriter outputVideoInterpolated;

	double fps = cvGetCaptureProperty(input, CV_CAP_PROP_FPS);

	//Add containter format to the output filename
	sprintf(output, outputFilenameInterpolated);
	strcat(output,container);

	outputVideoInterpolated.open(output, CV_FOURCC('X','2','6','4'), fps , newSize, true);

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

	// index of the current frame
	currentFrame = 1;

	frame = new StereoImage(originalSize, img->depth, img->nChannels);
	
	// used to fetch the different meshes from the vector
	int meshIndex = 0;

	// alpha factor for interpolation with meshes
	float alphaFactor = 1.0 / n;

	while (true)
	{
		if (!img)
			break;

		frame->setBoth_eye(img);
		frame = ie->split_vertical(frame);

		if ((currentFrame - 1) % n == 0)
		{
			// take optimized mesh

			cout << "\nWarping Frame " << currentFrame << "/" << totalFrameNumber - 2 << endl;

			Mat tmp = siw.warpImage(frame, newSize, leftDeformedMeshes.at(meshIndex), rightDeformedMeshes.at(meshIndex), leftLinearMeshes.at(meshIndex), rightLinearMeshes.at(meshIndex));
			outputVideoInterpolated.write(tmp);

			currentFrame++;
			meshIndex++;
		}
		else
		{
			// interpolate meshes

			cout << "\nInterpolate meshes" << endl;
			cout << "Frame " << currentFrame << "/" << totalFrameNumber - 2 << endl;

			float alpha = alphaFactor * ((currentFrame - 1) % n);
			Mesh leftDeformed = mm->interpolateMesh(leftDeformedMeshes.at(meshIndex - 1), leftDeformedMeshes.at(meshIndex), alpha);
			Mesh rightDeformed = mm->interpolateMesh(rightDeformedMeshes.at(meshIndex - 1), rightDeformedMeshes.at(meshIndex), alpha);
			Mesh leftLinear = mm->interpolateMesh(leftLinearMeshes.at(meshIndex - 1), leftLinearMeshes.at(meshIndex), alpha);
			Mesh rightLinear = mm->interpolateMesh(rightLinearMeshes.at(meshIndex - 1), rightLinearMeshes.at(meshIndex), alpha);

			Mat tmp = siw.warpImage(frame, newSize, leftDeformed, rightDeformed, leftLinear, rightLinear);

			outputVideoInterpolated.write(tmp);

			currentFrame++;
		}

		// load next frame
		img = cvQueryFrame(input);
	}

	// clean up
	cvReleaseCapture(&input);
	outputVideoInterpolated.~VideoWriter();
	//delete frame;
	*/
//-------------------------------------------------------------------
//---------TRACK PATHLINES IN THE DEFORMED VIDEO---------------------
//-------------------------------------------------------------------

	//input = cvCaptureFromFile(output);
	/*
	PathlineTracker deformedTracker(input, leftDeformedMeshes, rightDeformedMeshes);

	deformedTracker.trackPathlines();
	PathlineSets deformedPathlines = deformedTracker.getPathlineSets();

	FileManager::savePathlines("deformed pathlines.txt", "D:\\warping\\pathlines\\", deformedPathlines.pathlines.at(0));
	*/
	vector<Pathline> _deformedPathlines = FileManager::loadPathlines("D:\\warping\\pathlines\\deformed pathlines.txt");
	PathlineSets deformedPathlines;
	deformedPathlines.pathlines.push_back(_deformedPathlines);
	

	//cvReleaseCapture(&input);

//-------------------------------------------------------------------
//---------------------OPTIMIZE PATHLINES----------------------------
//-------------------------------------------------------------------

	PathlineSets leftOrigPathlines, rightOrigPathlines, leftDeformedPathlines, rightDeformedPathlines;

	// split the sets of pathlines into left and right view
	plm->splitPathlineSets(originalPathlines, leftOrigPathlines, rightOrigPathlines);
	plm->splitPathlineSets(deformedPathlines, leftDeformedPathlines, rightDeformedPathlines);

	std::sort(adjacencies.neighbors.begin(), adjacencies.neighbors.end());

	PathlineOptimizer leftPathlineOptimizer(leftOrigPathlines, leftDeformedPathlines, adjacencies, originalSize, newSize);
	PathlineOptimizer rightPathlineOptimizer(rightOrigPathlines, rightDeformedPathlines, adjacencies, originalSize, newSize);
	
	// optimize pathline
	PathlineSets leftOptimized, rightOptimized;
	//leftPathlineOptimizer.optimizePathlines(leftOptimized);
	//rightPathlineOptimizer.optimizePathlines(rightOptimized);
	//leftPathlineOptimizer.join();
	//rightPathlineOptimizer.join();

	// merge left and right optimized pathlines
	PathlineSets optimizedPathlines;
	//plm->mergePathlineSets(leftOptimized, rightOptimized, optimizedPathlines);

	//FileManager::savePathlines("optimized pathlines.txt", "D:\\warping\\pathlines\\", optimizedPathlines.pathlines.at(0));
	optimizedPathlines.pathlines.push_back(FileManager::loadPathlines("D:\\warping\\pathlines\\optimized pathlines.txt"));

	plm->splitPathlineSets(optimizedPathlines, leftOptimized, rightOptimized);

//-------------------------------------------------------------------------------------------------
//---------DEFORM AND OPTIMIZE MESHES FOR EVERY N-TH FRAME USING THE OPTIMIZED PATHLINES-----------
//-------------------------------------------------------------------------------------------------
	
	input = cvCaptureFromFile(fileName);

	// decode first frame
	img = cvQueryFrame(input);

	x = 0;
	currentFrame = 1;
	lastFrameDecoded = false;

	// adjust coordinates of right hand pathlines to be fit for the optimization
	Helper::adjustRightPathlineCoordinates(rightOptimized, newSize);
	Helper::adjustRightPathlineCoordinates(rightOrigPathlines, originalSize);

	leftDeformedMeshes.clear();
	rightDeformedMeshes.clear();

	leftLinearMeshes.clear();
	rightLinearMeshes.clear();

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
			StereoPathlineSolver spSolver(20, currentFrame, leftOptimized, rightOptimized, leftOrigPathlines, rightOrigPathlines);
			deformedMeshes = spSolver.solveStereoImageProblem(initialLeft, initialRight, originalSize, newSize, wfMapLeft, wfMapRight);

			leftDeformedMeshes.push_back(deformedMeshes.first);
			rightDeformedMeshes.push_back(deformedMeshes.second);

			leftLinearMeshes.push_back(spSolver.getInitialLeft());
			rightLinearMeshes.push_back(spSolver.getInitialRight());
		
			currentFrame = currentFrame + n;
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
			StereoPathlineSolver spSolver(20, currentFrame, leftOptimized, rightOptimized, leftOrigPathlines, rightOrigPathlines);
			deformedMeshes = spSolver.solveStereoImageProblem(initialLeft, initialRight, originalSize, newSize, wfMapLeft, wfMapRight);

			leftLinearMeshes.push_back(spSolver.getInitialLeft());
			rightLinearMeshes.push_back(spSolver.getInitialRight());

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

			currentFrame = totalFrameNumber - 2;

			lastFrameDecoded = true;
		}
	}

	// write left and right meshes to file
	FileManager::saveMeshesAsText("left meshes including pathlines.txt", "D:\\warping\\mesh\\", leftDeformedMeshes);
	FileManager::saveMeshesAsText("right meshes including pathlines.txt", "D:\\warping\\mesh\\", rightDeformedMeshes);
	
	FileManager::saveMeshesAsText("left linear meshes including pathlines.txt", "D:\\warping\\mesh\\", leftLinearMeshes);
	FileManager::saveMeshesAsText("right linear meshes including pathlines.txt", "D:\\warping\\mesh\\", rightLinearMeshes);

	// clean up
	cvReleaseCapture(&input);
	cvReleaseMat(&combinedSaliency);
	
//-------------------------------------------------------------------
//---------INTERPOLATE MESHES AND WARP EVERY SINGLE IMAGE------------
//-------------------------------------------------------------------
	/*
	input = cvCaptureFromFile(fileName);
	VideoWriter outputVideoFinal;

	double fps = cvGetCaptureProperty(input, CV_CAP_PROP_FPS);

	//Add containter format to the output filename
	sprintf(output, outputFilenameFinal);
	strcat(output,container);

	outputVideoFinal.open(output, CV_FOURCC('X','2','6','4'), fps , newSize, true);

	leftDeformedMeshes.clear();
	rightDeformedMeshes.clear();

	leftLinearMeshes.clear();
	rightLinearMeshes.clear();

	leftDeformedMeshes = FileManager::loadMeshes("D:\\warping\\mesh\\left meshes including pathlines.txt");
	rightDeformedMeshes = FileManager::loadMeshes("D:\\warping\\mesh\\right meshes including pathlines.txt");

	leftLinearMeshes = FileManager::loadMeshes("D:\\warping\\mesh\\left linear meshes including pathlines.txt");
	rightLinearMeshes = FileManager::loadMeshes("D:\\warping\\mesh\\right linear meshes including pathlines.txt");

	// decode the first frame
	img = cvQueryFrame(input);

	// index of the current frame
	currentFrame = 1;

	frame = new StereoImage(originalSize, img->depth, img->nChannels);
	
	// used to fetch the different meshes from the vector
	int meshIndex = 0;

	// alpha factor for interpolation with meshes
	float alphaFactor = 1.0 / n;

	while (true)
	{
		if (!img)
			break;

		frame->setBoth_eye(img);
		frame = ie->split_vertical(frame);

		if ((currentFrame - 1) % n == 0)
		{
			// take optimized mesh

			cout << "\nWarping Frame " << currentFrame << "/" << totalFrameNumber - 2 << endl;

			Mat tmp = siw.warpImage(frame, newSize, leftDeformedMeshes.at(meshIndex), rightDeformedMeshes.at(meshIndex), leftLinearMeshes.at(meshIndex), rightLinearMeshes.at(meshIndex));
			outputVideoFinal.write(tmp);

			currentFrame++;
			meshIndex++;
		}
		else
		{
			// interpolate meshes

			cout << "\nInterpolate meshes" << endl;
			cout << "Frame " << currentFrame << "/" << totalFrameNumber - 2 << endl;

			float alpha = alphaFactor * ((currentFrame - 1) % n);
			Mesh leftDeformed = mm->interpolateMesh(leftDeformedMeshes.at(meshIndex - 1), leftDeformedMeshes.at(meshIndex), alpha);
			Mesh rightDeformed = mm->interpolateMesh(rightDeformedMeshes.at(meshIndex - 1), rightDeformedMeshes.at(meshIndex), alpha);
			Mesh leftLinear = mm->interpolateMesh(leftLinearMeshes.at(meshIndex - 1), leftLinearMeshes.at(meshIndex), alpha);
			Mesh rightLinear = mm->interpolateMesh(rightLinearMeshes.at(meshIndex - 1), rightLinearMeshes.at(meshIndex), alpha);

			Mat tmp = siw.warpImage(frame, newSize, leftDeformed, rightDeformed, leftLinear, rightLinear);

			outputVideoFinal.write(tmp);

			currentFrame++;
		}

		// load next frame
		img = cvQueryFrame(input);
	}

	// clean up
	cvReleaseCapture(&input);
	outputVideoFinal.~VideoWriter();
	*/
}
#endif