#include "StereoImageWarper.h"
#include "MeshManager.h"
#include "QuadSaliencyManager.h"
#include "StereoSolver.h"
#include "StereoImage.h"

StereoImageWarper::StereoImageWarper(void)
{
}


StereoImageWarper::~StereoImageWarper(void)
{
}

IplImage* StereoImageWarper::warpImage(StereoImage* img, Size &destSize, Mat &saliencyMap)
{
	cout << "\nStart stereo image warping" << endl;

	//initialisation
	oldSize.height = img->getBoth_eye()->height;
	oldSize.width = img->getBoth_eye()->width;
	newSize = destSize;
	Mat src = img->getBoth_eye();
	Mesh deformedLeft, deformedRight, initialLeft, initialRight, linearScaledLeft, linearScaledRight;
	MeshManager* mm = MeshManager::getInstance();
	QuadSaliencyManager* qsm = QuadSaliencyManager::getInstance();
	StereoSolver ss;

	Size destLeftSize(newSize.width / 2, newSize.height);
	Mat destLeft = Mat::zeros(destLeftSize, CV_32FC3);
	Mat destRight = Mat::zeros(destLeftSize, CV_32FC3);

	mm->initializeMesh(initialLeft, destLeftSize);
	initialRight = mm->generateRightEyeMesh(initialLeft, img, Size(oldSize.width / 2, oldSize.height));

	// TODO saliencyMap Breite halbieren?
	vector<pair<float, Quad>> wfMapLeft = qsm->assignSaliencyValuesToQuads(initialLeft, saliencyMap);
	vector<pair<float, Quad>> wfMapRight = qsm->assignSaliencyValuesToQuads(initialRight, saliencyMap);

	pair<Mesh, Mesh> deformedMeshes = ss.solveStereoImageProblem(initialLeft, initialRight, oldSize, destSize, wfMapLeft, wfMapRight);
	
	deformedLeft = deformedMeshes.first;
	deformedRight = deformedMeshes.second;

	linearScaledLeft = ss.getInitialLeft();
	linearScaledRight = ss.getInitialRight();
	
	// linear scaled images of left and right view
	Mat linearLeft, linearRight;
	Mat leftEye = img->getBoth_eye();
	Mat rightEye = img->getBoth_eye();
	
	resize(leftEye, linearLeft, destLeftSize);
	resize(rightEye, linearRight, destLeftSize);

	warp(linearScaledLeft, deformedLeft, linearLeft, destLeft);
	warp(linearScaledRight, deformedRight, linearRight, destRight);

	destLeft.convertTo(destLeft, src.type());
	destRight.convertTo(destRight, src.type());

	// TODO merge left and right warped image

	return NULL;
}