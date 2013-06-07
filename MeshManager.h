#pragma once

#include "stdafx.h"
#include "StereoImage.h"

class MeshManager
{
private:
	MeshManager()
	{
	}

	static bool instanceFlag;
	static MeshManager *single;
	int determineQuadNumber(Size & size, int &quadNumberX, int &quadNumberY);
	void buildQuadsAndEdges(Mesh &mesh); // builds quads and edges from given vertices

public:
    ~MeshManager()
    {
        instanceFlag = false;
    }

	static MeshManager* getInstance();
	void initializeMesh(Mesh &result, Size &size);
	Mesh deepCopyMesh(const Mesh &m);
	vector<double> meshToDoubleVec(Mesh &m);
	void doubleVecToMesh(const vector<double> &x, Mesh &result);
	Mesh generateRightEyeMesh(Mesh &leftEyeMesh, StereoImage* img, Size &rightEyeSize);
};

