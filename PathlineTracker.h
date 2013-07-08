#pragma once

#include "stdafx.h"
#include "IFrameProcessor.h"
#include "StereoImage.h"

class PathlineTracker : public IFrameProcessor
{

public:
	// constructor for the case to track pathlines in the original video
	PathlineTracker(CvCapture* input);

	// constructor for the case to track pathlines in the deformed video
	PathlineTracker(CvCapture* input, vector<Mesh> &leftSeedMeshes, vector<Mesh> &rightSeedMeshes);
	~PathlineTracker(void);

	PathlineSets getPathlineSets() const;

	// tracks the path lines in the video seed on a regular grid
	void trackPathlines();
	
private:
	bool warpedVideo;

	CvCapture* input;
	PathlineSets sets; // set of all pathlines of the video
	vector<Pathline> pathlines; // pathlines between frame i, i+1, ... , j
	Mesh seedLeft, seedRight;
	int frameCounter;
	int maxFrames;
	bool addNewPoints;
	Size videoSize;
	Size leftSize, rightSize;

	vector<Mesh> leftSeedMeshes, rightSeedMeshes;
	Mesh leftSeedMesh, rightSeedMesh;

	Mat currentGray, prevGray;
	StereoImage current, prev;
	vector<uchar> status;
	vector<float> err;
	vector<Point2f> initial;
	vector<Point2f> detected;

	bool acceptPoint(unsigned int i, Point2f p);
	void handleTrackedPoints();
	void seedNewPoints();
	void seedNewPoints(Mesh &left, Mesh &right);
	void addSeedPointsToPathlines();
	void appendTrackedPointsToPathlines();

protected:
	void process(Mat &currentFrame, Mat &prevFrame);
};

