#pragma once

#include "stdafx.h"
#include "StereoImage.h"

class MeshManager
{
private:
	static bool instanceFlag;
	static MeshManager *single;
	int determineQuadNumber(Size & size, int &quadNumberX, int &quadNumberY);
	MeshManager()
	{
	}

public:
    static MeshManager* getInstance();
    ~MeshManager()
    {
        instanceFlag = false;
    }

	void initializeMesh(Mesh &result, Size &size);
	Mesh deepCopyMesh(const Mesh &m);
	vector<double> meshToDoubleVec(Mesh &m);
	void doubleVecToMesh(const vector<double> &x, Mesh &result);
	Mesh generateRightEyeMesh(Mesh &leftEyeMesh, StereoImage* img, Size &rightEyeSize);
};

