#include "PathlineTracker.h"
#include "MeshManager.h"


PathlineTracker::PathlineTracker(CvCapture* input)
{
	this->input = input;
	frameCounter = 1;
	addNewPoints = true;
	warpedVideo = false;
	videoSize = Size((int) cvGetCaptureProperty(input, CV_CAP_PROP_FRAME_WIDTH), (int) cvGetCaptureProperty(input, CV_CAP_PROP_FRAME_HEIGHT));
}

PathlineTracker::PathlineTracker(CvCapture* input, vector<Mesh> &leftSeedMeshes, vector<Mesh> &rightSeedMeshes)
{
	this->input = input;
	frameCounter = 0;
	addNewPoints = true;
	this->leftSeedMeshes = leftSeedMeshes;
	this->rightSeedMeshes = rightSeedMeshes;
	warpedVideo = true;
	videoSize = Size((int) cvGetCaptureProperty(input, CV_CAP_PROP_FRAME_WIDTH), (int) cvGetCaptureProperty(input, CV_CAP_PROP_FRAME_HEIGHT));
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
}

void PathlineTracker::process(Mat &input)
{
}

bool PathlineTracker::acceptPoint(unsigned i, Point2f p)
{
	if (status.at(i) == 0)
	{
		addNewPoints = true;
		return false;
	}

	if (i / status.size() < 1)
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

void PathlineTracker::handleTrackedPoints()
{
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
	{
		seedNewPoints(leftSeedMesh, rightSeedMesh);
	}
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