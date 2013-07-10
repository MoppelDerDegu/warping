#include "PathlineTracker.h"
#include "MeshManager.h"

PathlineTracker::PathlineTracker(CvCapture* input)
{
	this->input = input;
	frameCounter = 1;
	addNewPoints = true;
	warpedVideo = false;
	videoSize = Size((int) cvGetCaptureProperty(input, CV_CAP_PROP_FRAME_WIDTH), (int) cvGetCaptureProperty(input, CV_CAP_PROP_FRAME_HEIGHT));
	leftSize = Size(videoSize.width / 2, videoSize.height);
	rightSize = Size(videoSize.width / 2, videoSize.height);
	maxFrames = ((int) cvGetCaptureProperty(input, CV_CAP_PROP_FRAME_COUNT)) - 2;

	MeshManager* mm = MeshManager::getInstance();
	mm->initializeMesh(leftSeedMesh, leftSize);
	mm->initializeMesh(rightSeedMesh, rightSize);
}

PathlineTracker::PathlineTracker(CvCapture* input, vector<Mesh> &leftSeedMeshes, vector<Mesh> &rightSeedMeshes)
{
	this->input = input;
	frameCounter = 1;
	addNewPoints = true;
	this->leftSeedMeshes = leftSeedMeshes;
	this->rightSeedMeshes = rightSeedMeshes;
	warpedVideo = true;
	videoSize = Size((int) cvGetCaptureProperty(input, CV_CAP_PROP_FRAME_WIDTH), (int) cvGetCaptureProperty(input, CV_CAP_PROP_FRAME_HEIGHT));
	leftSize = Size(videoSize.width / 2, videoSize.height);
	rightSize = Size(videoSize.width / 2, videoSize.height);
	maxFrames = ((int) cvGetCaptureProperty(input, CV_CAP_PROP_FRAME_COUNT)) - 2;
}

PathlineTracker::~PathlineTracker(void)
{
	cvReleaseCapture(&input);
}

PathlineSets PathlineTracker::getPathlineSets() const
{
	return sets;
}

void PathlineTracker::trackPathlines()
{
	cout << "\nTracking Pathlines..." << endl;

	// decode the first frame
	IplImage* img = cvQueryFrame(input);

	// initialize necessary objects
	current = StereoImage(videoSize, img->depth, img->nChannels);
	prev = StereoImage(videoSize, img->depth, img->nChannels);

	int x = 0;
	while (true)
	{
		if (!img)
			break;
		
		current.setBoth_eye(img);

		if (x == 0)
		{
			// for the first frame of the video
			prev.setBoth_eye(img);
			x++;
		}

		//process the current frame, i.e. look for feature points
		Mat currentFrame = current.getBoth_eye();
		Mat prevFrame = prev.getBoth_eye();
		process(currentFrame, prevFrame);

		// set current frame to previous frame
		prev = current;

		// load next frame
		img = cvQueryFrame(input);

		frameCounter++;
	}
}

void PathlineTracker::process(Mat &currentFrame, Mat &prevFrame)
{
	cout << ">> Processing frame " << frameCounter << endl;

	cvtColor(currentFrame, currentGray, CV_BGR2GRAY);
	cvtColor(prevFrame, prevGray, CV_BGR2GRAY);

	// seed points for the first frame
	if (frameCounter == 1)
	{
		seedNewPoints();
		addSeedPointsToPathlines();
	}

	// track the points between two consecutive frames
	calcOpticalFlowPyrLK(prevGray, currentGray, initial, detected, status, err, Size(250, 250), 3);

	handleTrackedPoints();
}

void PathlineTracker::handleTrackedPoints()
{
	bool ok = true;

	for (unsigned int i = 0; i < detected.size(); i++)
	{
		if (!acceptPoint(i, detected.at(i)))
		{
			seedNewPoints();
			ok = false;
			break;
		}
	}

	if (ok)
	{
		// append tracked points to the pathlines
		if (frameCounter != 1)
			appendTrackedPointsToPathlines();

		if (frameCounter == maxFrames)
		{
			// every feature point was found in the last frame
			sets.pathlines.push_back(pathlines);
		}
	}
	else if (!ok)
	{
		// push tacked pathlines to the set of pathlines
		sets.pathlines.push_back(pathlines);

		// add seed points to new pathlines
		addSeedPointsToPathlines();

		if (frameCounter == maxFrames)
		{
			// feature points were not found in the last frame, so add new seed points to the sets of pathlines
			sets.pathlines.push_back(pathlines);
		}
	}
}

bool PathlineTracker::acceptPoint(unsigned int i, Point2f p)
{
	if (status.at(i) == 0)
	{
		addNewPoints = true;
		return false;
	}

	if (i / (status.size() / 2.0) < 1)
	{
		// point is in the left view

		if (p.x > leftSize.width)
		{
			// point has been found in the right view
			addNewPoints = true;
			return false;
		}
		else
		{
			addNewPoints = false;
			return true;
		}		
	}
	else
	{
		// point is in the right view
		
		if (p.x < leftSize.width)
		{
			// point has been found in the left view
			addNewPoints = true;
			return false;
		}
		else
		{
			addNewPoints = false;
			return true;
		}
	}
}

void PathlineTracker::seedNewPoints()
{
	initial.clear();

	if (warpedVideo)
	{
		Mesh leftseed = leftSeedMeshes.at(frameCounter - 1);
		Mesh rightseed = rightSeedMeshes.at(frameCounter - 1);
		
		seedNewPoints(leftseed, rightseed);
	}
	else
		seedNewPoints(leftSeedMesh, rightSeedMesh);
}

void PathlineTracker::seedNewPoints(Mesh &left, Mesh &right)
{
	MeshManager* mm = MeshManager::getInstance();

	vector<Vertex> innerLeft = mm->getInnerVertices(left, leftSize);
	vector<Vertex> innerRight = mm->getInnerVertices(right, rightSize);

	// add seed points for the left view
	for (unsigned int i = 0; i < innerLeft.size(); i++)
	{
		Point2f p(innerLeft.at(i).x, innerLeft.at(i).y);
		initial.push_back(p);
	}

	// add seed points for the right view
	for (unsigned int i = 0; i < innerRight.size(); i++)
	{
		Point2f p(innerRight.at(i).x + leftSize.width, innerRight.at(i).y);
		initial.push_back(p);
	}
}

void PathlineTracker::addSeedPointsToPathlines()
{
	pathlines.clear();

	// add pathline points for the left view
	for (unsigned int i = 0; i < initial.size() / 2; i++)
	{
		Point2f p = initial.at(i);

		Pathline pl;
		pair<int, Point2f> pathseed;

		pathseed.first = frameCounter;
		pathseed.second = p;

		pl.path.push_back(pathseed);
		pl.seedIndex = i;

		pathlines.push_back(pl);
	}

	int index = 0;
	// add pathline points for the right view
	for (unsigned int i = initial.size() / 2; i < initial.size(); i++)
	{
		Point2f p = initial.at(i);

		Pathline pl;
		pair<int, Point2f> pathseed;

		pathseed.first = frameCounter;
		pathseed.second = p;

		pl.path.push_back(pathseed);
		pl.seedIndex = index;

		pathlines.push_back(pl);

		index++;
	}
}

void PathlineTracker::appendTrackedPointsToPathlines()
{
	for (unsigned int i = 0; i < detected.size(); i++)
	{
		Point2f point = detected.at(i);

		pair<int, Point2f> p;

		p.first = frameCounter;
		p.second = point;

		pathlines.at(i).path.push_back(p);
	}
}