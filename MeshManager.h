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
	
	Mesh deepCopyMesh(const Mesh &m);

	vector<double> meshToDoubleVec(Mesh &m);
	void doubleVecToMesh(const vector<double> &x, Mesh &result);

	void initializeMesh(Mesh &result, Size &size);
	Mesh generateRightEyeMesh(Mesh &leftEyeMesh, StereoImage* img, Size &rightEyeSize);

	vector<double> xCoordsToDoubleVec(Mesh &m);
	void xCoordsToMesh(const vector<double> &x, Mesh &result);

	vector<double> yCoordsToDoubleVec(Mesh &m);
	void yCoordsToMesh(const vector<double> &y, Mesh &result);

	void mergeXandYMeshes(Mesh &xMesh, Mesh &yMesh, Mesh &result);
};

