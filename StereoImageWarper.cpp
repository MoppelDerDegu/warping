#include "StereoImageWarper.h"
#include "MeshManager.h"
#include "QuadSaliencyManager.h"
#include "StereoSolver.h"
#include "StereoImage.h"
#include "Helper.h"
#include "FileManager.h"

StereoImageWarper::StereoImageWarper(void)
{
}

StereoImageWarper::~StereoImageWarper(void)
{
}

Mat StereoImageWarper::warpImage(StereoImage* img, Size &destSize, Mat &saliencyMap)
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
	StereoSolver ss(20);

	Size destLeftSize(newSize.width / 2, newSize.height);
	Size oldLeftSize(oldSize.width / 2, oldSize.height);
	Mat destLeft = Mat::zeros(destLeftSize, CV_32FC3);
	Mat destRight = Mat::zeros(destLeftSize, CV_32FC3);

	// initialize mesh for left and right view
	mm->initializeMesh(initialLeft, oldLeftSize);
	initialRight = mm->generateRightEyeMesh(initialLeft, img, oldLeftSize);

	// assign saliency values to quads of left and right view
	vector<pair<float, Quad>> wfMapLeft = qsm->assignSaliencyValuesToQuads(initialLeft, saliencyMap);
	vector<pair<float, Quad>> wfMapRight = qsm->assignSaliencyValuesToQuads(initialRight, saliencyMap);

	// deform left and right mesh
	pair<Mesh, Mesh> deformedMeshes = ss.solveStereoImageProblem(initialLeft, initialRight, oldSize, destSize, wfMapLeft, wfMapRight);
	
	deformedLeft = deformedMeshes.first;
	deformedRight = deformedMeshes.second;

	linearScaledLeft = ss.getInitialLeft();
	linearScaledRight = ss.getInitialRight();

	// linear scaled images of left and right view
	Mat linearLeft, linearRight;
	Mat leftEye = img->getLeft_eye();
	Mat rightEye = img->getRight_eye();
	
	resize(leftEye, linearLeft, destLeftSize);
	resize(rightEye, linearRight, destLeftSize);

	linearLeft.convertTo(linearLeft, CV_32FC3);
	linearRight.convertTo(linearRight, CV_32FC3);

	// warp left and right view
	warp(linearScaledLeft, deformedLeft, linearLeft, destLeft);
	warp(linearScaledRight, deformedRight, linearRight, destRight);

	destLeft.convertTo(destLeft, src.type());
	destRight.convertTo(destRight, src.type());

	// merge left and right warped image
	Mat dest = Mat::zeros(Size(destLeft.size().width * 2, destLeft.size().height), src.type());
	Mat roi = dest(Rect(0, 0, destLeft.size().width, destLeft.size().height));
	destLeft.copyTo(roi);
	roi = dest(Rect(destLeft.size().width, 0, destLeft.size().width, destLeft.size().height));
	destRight.copyTo(roi);

	// draw mesh over left and right image
	Helper::drawMeshOverMat(deformedLeft, destLeft);
	Helper::drawMeshOverMat(deformedRight, destRight);
	Mat destwithmesh = Mat::zeros(Size(destLeft.size().width * 2, destLeft.size().height), src.type());
	roi = destwithmesh(Rect(0, 0, destLeft.size().width, destLeft.size().height));
	destLeft.copyTo(roi);
	roi = destwithmesh(Rect(destLeft.size().width, 0, destLeft.size().width, destLeft.size().height));
	destRight.copyTo(roi);

	// merge left and right linear scaled image
	Mat linearBoth = Mat::zeros(Size(destLeft.size().width * 2, destLeft.size().height), linearLeft.type());
	roi = linearBoth(Rect(0, 0, destLeft.size().width, destLeft.size().height));
	linearLeft.copyTo(roi);
	roi = linearBoth(Rect(destLeft.size().width, 0, destLeft.size().width, destLeft.size().height));
	linearRight.copyTo(roi);

	return dest;
}

Mat StereoImageWarper::warpImage(StereoImage* img, Size &destSize, Mesh &deformedLeft, Mesh &deformedRight, Mesh &linearScaledLeft, Mesh &linearScaledRight)
{
	cout << "\nStart stereo image warping" << endl;

	newSize = destSize;
	Mat src = img->getBoth_eye();

	Size destLeftSize(newSize.width / 2, newSize.height);
	Mat destLeft = Mat::zeros(destLeftSize, CV_32FC3);
	Mat destRight = Mat::zeros(destLeftSize, CV_32FC3);

	Mat linearLeft, linearRight;
	Mat leftEye = img->getLeft_eye();
	Mat rightEye = img->getRight_eye();

	// scale down left and right for reference during the image warping
	resize(leftEye, linearLeft, destLeftSize);
	resize(rightEye, linearRight, destLeftSize);

	linearLeft.convertTo(linearLeft, CV_32FC3);
	linearRight.convertTo(linearRight, CV_32FC3);

	// warp left and right view
	warp(linearScaledLeft, deformedLeft, linearLeft, destLeft);
	warp(linearScaledRight, deformedRight, linearRight, destRight);

	destLeft.convertTo(destLeft, src.type());
	destRight.convertTo(destRight, src.type());

	// merge left and right warped image
	Mat dest = Mat::zeros(Size(destLeft.size().width * 2, destLeft.size().height), src.type());
	Mat roi = dest(Rect(0, 0, destLeft.size().width, destLeft.size().height));
	destLeft.copyTo(roi);
	roi = dest(Rect(destLeft.size().width, 0, destLeft.size().width, destLeft.size().height));
	destRight.copyTo(roi);

	return dest;
}