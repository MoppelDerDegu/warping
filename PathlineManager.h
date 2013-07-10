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
};

