#include "MonoImageWarper.h"
#include "ImageWarper.h"
#include "stdafx.h"
#include "QuadSaliencyManager.h"
#include "MonoSolver.h"
#include "Helper.h"
#include "WarpingMath.h"
#include "FileManager.h"
#include "MeshManager.h"
#include "GradientGenerator.h"
#include "ImageSaliencyDetector.h"

MonoImageWarper::MonoImageWarper(void)
{
}

MonoImageWarper::~MonoImageWarper(void)
{
}

IplImage* MonoImageWarper::warpImage(IplImage* img, Size &destSize, Mat &saliencyMap)
{
	cout << "\nStart image warping" << endl;

	//initialisation
	oldSize.height = img->height;
	oldSize.width = img->width;
	newSize = destSize;
	Mat src = img;
	Mat dest = Mat::zeros(destSize, CV_32FC3);
	QuadSaliencyManager* qsm = QuadSaliencyManager::getInstance();
	MonoSolver solver(oldSize);
	MeshManager* mm = MeshManager::getInstance();
	Mesh initialMesh;

	// place regular grid mesh over image
	mm->initializeMesh(initialMesh, oldSize);

	// assign saliency values to quads
	vector<pair<float, Quad>> wfMap = qsm->assignSaliencyValuesToQuads(initialMesh, saliencyMap);

	Mesh deformedMesh = solver.solveImageProblem(initialMesh, initialMesh, destSize, wfMap);
	Mesh linearScaledMesh = solver.getInitialGuess();
	
	//linearly scale the image as starting point
	Mat linearScaledImage;
	resize(src, linearScaledImage, newSize);
	linearScaledImage.convertTo(linearScaledImage, CV_32FC3);

	// do the warping according to mesh
	warp(linearScaledMesh, deformedMesh, linearScaledImage, dest);

	// convert dest frame back to original type
	dest.convertTo(dest, src.type());

	return &Helper::MatToIplImage(dest);
}