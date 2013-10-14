#pragma once

#include "stdafx.h"

class PathlineManager
{
private:
	PathlineManager()
	{
	}

	static PathlineManager* single;
	static bool instanceFlag;

	bool liesInQuad(Quad &quad, Point2f &point); // checks if a point lies within a squad

public:
	~PathlineManager(void);
	static PathlineManager* getInstance();

	void getAdjacencies(PathlineSets &sets, Mesh &seedMesh, Size &seedMeshSize, PathlineAdjacencies &result);
	pair<Pathline, Pathline> getNeighbors(pair<unsigned int, unsigned int> &neighbors, vector<Pathline> &pathlines); // returns neighboring pathlines

	void splitPathlineSets(PathlineSets &original, PathlineSets &left, PathlineSets &right); // split pathlines into two different sets containing the left and the right pathlines
	void mergePathlineSets(PathlineSets &left, PathlineSets &right, PathlineSets &result); // merge left and right pathlines together into one set

	// Initializes a scaling matrix mapping
	void createPathlineMatrixMapping(PathlineSets &pathlineSets, PathlineMatrixMapping &result, Size &oldSize, Size &newSize);

	/*
		Initializes a  translation vector mapping with zero entries, i.e.
		(0)
		(0)
	*/
	void createPathlineTransVecMapping(PathlineSets &pathlineSets, PathlineTransVecMapping &result);

	// Initializes a scaling matrix mapping for neighboring pathlines
	void createNeighborMatrixMapping(PathlineSets &pathlineSets, PathlineAdjacencies &adjacencies, NeighborMatrixMapping &result, Size &oldSize, Size &newSize);

	// transforms a PathlineMatrixMapping and PathlineTransVecMapping into a vector<double>. That vector is appended with dummyvariables
	// which are used during the optimization but we are not interested in.
	void mappingsToDoubleVec(map<Pathline, ScalingMatrix2x2> &matMapping, map<Pathline, TranslationVector2> &vecMapping, int numberOfDummyVariables, vector<double> &result);
	
	void mapPathlinesToQuads(int frame, PathlineSets &pathlines, Mesh &mesh, map<int, int> &result); // which pathline belongs to which quad
	void getPointsInFrame(vector<Pathline> &lines, int frame, vector<Point2f> &result); // returns pathline points in the specified frame
	void getLinesContainingFrame(PathlineSets &pathlines, int frame, vector<Pathline> &result); // returns all pathlines that travers the specified frame
};

