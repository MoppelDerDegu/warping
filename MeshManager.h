#pragma once

#include "stdafx.h"
#include "StereoImage.h"

// Manager class that contains convenience methods for handling meshes
class MeshManager
{
private:
	MeshManager()
	{
	}

	static bool instanceFlag;
	static MeshManager *single;
	int determineQuadNumber(Size & size, int &quadNumberX, int &quadNumberY); // returns the number of quads that fill the specified size

public:
    ~MeshManager()
    {
        instanceFlag = false;
    }

	static MeshManager* getInstance();
	
	void buildQuadsAndEdges(Mesh &mesh); // builds quads and edges from given vertices

	Mesh deepCopyMesh(const Mesh &m); // performs a deep copy of a mesh

	vector<Vertex> getInnerVertices(const Mesh &m, Size size); // returns a vector containing all non edge vertices of a mesh

	vector<double> meshToDoubleVec(Mesh &m);
	void doubleVecToMesh(const vector<double> &x, Mesh &result);

	void initializeMesh(Mesh &result, Size &size); // initializes a mesh for a specified size with a fixes quad size
	Mesh generateRightEyeMesh(Mesh &leftEyeMesh, StereoImage* img, Size &rightEyeSize); // looks for the inner vertices of the left view in the right view and constructs the mesh accordingly
	Mesh interpolateMesh(Mesh &first, Mesh &second, float alpha); // interpolates to meshes linearly and returns the interpolated mesh
};

