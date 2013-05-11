#include "stdafx.h"
#include "FileManager.h"
#include "Helper.h"

BOOL FileManager::mkDir(const char* &path)
{
	static char buffer[1024];
	strcpy(buffer, path);
	for (int i = 0; buffer[i] != 0; i ++)
	{
		if (buffer[i] == '\\')
		{
			buffer[i] = '\0';
			CreateDirectoryA(buffer, 0);
			buffer[i] = '\\';
		}
	}
	return CreateDirectoryA(path, 0);
}

void FileManager::saveMeshAsImage(const string fileName, const string dir, const Mesh &m, const Size &s)
{
	const char* _dir = (char*) dir.c_str();
	mkDir(_dir);

	Mat mat = Helper::meshAsMat(m, s);

	imwrite(dir + fileName, mat);
}

void FileManager::saveMat(const string fileName, const string dir, const Mat &mat)
{
	const char* _dir = (char*) dir.c_str();
	mkDir(_dir);
	imwrite(dir + fileName, mat);
}

void FileManager::saveMeshAsText(const string fileName, const string dir, Mesh &mesh)
{
	vector<double> vertices = Helper::meshToDoubleVec(mesh);

	const char* _dir = (char*) dir.c_str();
	mkDir(_dir);

	ofstream myfile;
	myfile.open(dir + fileName);

	if (myfile.is_open())
	{
		for (unsigned int i = 0; i < vertices.size(); i++)
		{
			if (i < vertices.size() - 1)
				myfile << vertices.at(i) << "\n";
			else
				myfile << vertices.at(i);
		}

		myfile.close();
	}
	else
		cerr << "Unable to open file: " << fileName << endl;
}

Mesh FileManager::loadMesh(const string fileName)
{
	vector<double> res;
	Mesh resmesh;
	string line;
	ifstream myfile;
	myfile.open(fileName);

	if (myfile.is_open())
	{
		while (myfile.good())
		{
			getline(myfile, line);
			res.push_back(Helper::stringToDouble(line));
		}

		Helper::doubleVecToMesh(res, resmesh);
	}
	else
		cerr << "Unable to open file: " << fileName << endl;

	return resmesh;
}

void FileManager::saveMeshROIAsImage(const string fileName, const string dir, const Mesh &m, const Size &s)
{
	const char* _dir = (char*) dir.c_str();
	mkDir(_dir);
	Mat mat = Helper::meshAsMat(m, s);

	Scalar lineColor(0, 0, 255); // red
	int thickness = 1;
	int linetype = 8;

	for (unsigned int i = 0; i < m.quads.size(); i++)
	{
		Quad quad = m.quads.at(i);
		Point topleft, topright, bottomleft, bottomright;
		int roiWidth, roiHeight;

		if (quad.v1.y > quad.v2.y)
		{
			topleft.y = quad.v2.y;
			topright.y = quad.v2.y;
		}
		else
		{
			topleft.y = quad.v1.y;
			topright.y = quad.v1.y;
		}

		if (quad.v1.x > quad.v3.x)
		{
			topleft.x = quad.v3.x;
			bottomleft.x = quad.v3.x;
		}
		else
		{
			topleft.x = quad.v1.x;
			bottomleft.x = quad.v1.x;
		}

		if (quad.v2.x < quad.v4.x)
			topright.x = quad.v4.x;
		else
			topright.x = quad.v2.x;

		if (quad.v3.y < quad.v4.y)
			bottomleft.y = quad.v4.y;
		else
			bottomleft.y = quad.v3.y;

		Vertex _topleft, _topright, _bottomleft;
		_topleft.x = topleft.x;
		_topleft.y = topleft.y;
		_topright.x = topright.x;
		_topright.y = topright.y;
		_bottomleft.x = bottomleft.x;
		_bottomleft.y = bottomleft.y;

		roiWidth = (int) Helper::getDistance(_topleft, _topright);
		roiHeight = (int) Helper::getDistance(_topleft, _bottomleft);

		bottomright.x = bottomleft.x + roiWidth;
		bottomright.y = topright.y + roiHeight;

		line(mat, topleft, topright, lineColor, thickness, linetype);
		line(mat, topright, bottomright, lineColor, thickness, linetype);
		line(mat, bottomright, bottomleft, lineColor, thickness, linetype);
		line(mat, bottomleft, topleft, lineColor, thickness, linetype);
	}

	imwrite(dir + fileName, mat);
}