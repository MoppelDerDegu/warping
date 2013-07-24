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

	bool liesInQuad(Quad &quad, Point2f &point);

public:
	~PathlineManager(void);
	static PathlineManager* getInstance();

	void getAdjacencies(PathlineSets &sets, Mesh &seedMesh, Size &seedMeshSize, PathlineAdjacencies &result);
	pair<Pathline, Pathline> getNeighbors(pair<unsigned int, unsigned int> &neighbors, vector<Pathline> &pathlines);

	void splitPathlineSets(PathlineSets &original, PathlineSets &left, PathlineSets &right);
	void mergePathlineSets(PathlineSets &left, PathlineSets &right, PathlineSets &result);

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
	
	void mapPathlinesToQuads(int frame, PathlineSets &pathlines, Mesh &mesh, map<int, int> &result);
	void getPointsInFrame(vector<Pathline> &lines, int frame, vector<Point2f> &result);
	void getLinesContainingFrame(PathlineSets &pathlines, int frame, vector<Pathline> &result);
};

