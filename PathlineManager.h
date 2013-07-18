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

public:
	~PathlineManager(void);
	static PathlineManager* getInstance();

	void getAdjacencies(PathlineSets &sets, Mesh &seedMesh, Size &seedMeshSize, PathlineAdjacencies &result);
	pair<Pathline, Pathline> getNeighbors(pair<unsigned int, unsigned int> &neighbors, vector<Pathline> &pathlines);

	void splitPathlineSets(PathlineSets &original, PathlineSets &left, PathlineSets &right);

	/*
		Initializes a scaling matrix mapping with identity matrices, i.e.
		(1 0)
		(0 1)
	*/
	void createPathlineMatrixMapping(PathlineSets &pathlineSets, PathlineMatrixMapping &result);

	/*
		Initializes a  translation vector mapping with zero entries, i.e.
		(0)
		(0)
	*/
	void createPathlineTransVecMapping(PathlineSets &pathlineSets, PathlineTransVecMapping &result);

	/*
		Initializes a scaling matrix mapping for neighboring pathlines with identity matrices, i.e.
		(1 0)
		(0 1)
	*/
	void createNeighborMatrixMapping(PathlineSets &pathlineSets, PathlineAdjacencies &adjacencies, NeighborMatrixMapping &result);

	// transforms a PathlineMatrixMapping and PathlineTransVecMapping into a vector<double>. That vector is appended with dummyvariables
	// which are used during the optimization but we are not interested in.
	void mappingsToDoubleVec(vector<pair<Pathline, ScalingMatrix2x2>> &matMapping, vector<pair<Pathline, TranslationVector2>> &vecMapping, int numberOfDummyVariables, vector<double> &result);
};

