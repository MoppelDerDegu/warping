#pragma once

#include "stdafx.h"
#include "StereoImage.h"

class PathlineTracker
{

public:
	// constructor for the case to track pathlines in the original video
	PathlineTracker(VideoCapture &capture);

	// constructor for the case to track pathlines in the deformed video
	PathlineTracker(VideoCapture &capture, vector<Mesh> &leftSeedMeshes, vector<Mesh> &rightSeedMeshes);

	~PathlineTracker(void);

	PathlineSets getPathlineSets() const;

	// tracks the path lines in the video seed on a regular grid
	void trackPathlines();
	
private:
	bool warpedVideo;

	VideoCapture capture;
	PathlineSets sets; // set of all pathlines of the video
	vector<Pathline> pathlines; // pathlines between frame i, i+1, ... , j
	int frameCounter;
	int maxFrames;
	bool addNewPoints;
	Size videoSize;
	Size leftSize, rightSize;

	vector<Mesh> leftSeedMeshes, rightSeedMeshes; // seed meshes for the left and right view, used during tracking points in the deformed video
	Mesh leftSeedMesh, rightSeedMesh; // seed mesh for the left and right view of the video

	Mat currentGray, prevGray; // current and previous frame as gray scale images
	vector<uchar> status; // specifies if a point was found in the next frame
	vector<float> err; // error values of the different points
	vector<Point2f> initial; // initial points to look for
	vector<Point2f> detected; // points found in the next frame

	bool acceptPoint(unsigned int i, Point2f p); // checks if point i is accepted
	void handleTrackedPoints(); // handles tracked points
	void seedNewPoints();
	void seedNewPoints(Mesh &left, Mesh &right);
	void addSeedPointsToPathlines();
	void appendTrackedPointsToPathlines();
	void process(Mat &currentFrame, Mat &prevFrame); // looks for points from the previous frame in the current frame
};

