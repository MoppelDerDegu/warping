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

	Mat mat = Mat::zeros(s.height, s.width, CV_8UC3);
	
	// line parameters
	Scalar lineColor(0, 0, 0); // black
	int thickness = 1;
	int linetype = 8;

	// make everything white
	for (int y = 0; y < mat.rows; y++)
	{
		for (int x = 0; x < mat.cols; x++)
		{
			mat.at<Vec3b> (y, x) [0] = 255;
			mat.at<Vec3b> (y, x) [1] = 255;
			mat.at<Vec3b> (y, x) [2] = 255;
		}
	}

	// draw lines
	Point start, end;
	Quad q;
	for (unsigned int i = 0; i < m.quads.size(); i++)
	{
		q = m.quads.at(i);
		for (unsigned int j = 0; j < 4; j++)
		{
			if (j == 0)
			{
				start.x = q.v1.x;
				start.y = q.v1.y;
				end.x = q.v2.x;
				end.y = q.v2.y;

				line(mat, start, end, lineColor, thickness, linetype);
			}
			else if (j == 1)
			{
				start.x = q.v2.x;
				start.y = q.v2.y;
				end.x = q.v4.x;
				end.y = q.v4.y;

				line(mat, start, end, lineColor, thickness, linetype);
			}
			else if (j == 2)
			{
				start.x = q.v4.x;
				start.y = q.v4.y;
				end.x = q.v3.x;
				end.y = q.v3.y;

				line(mat, start, end, lineColor, thickness, linetype);
			}
			else
			{
				start.x = q.v3.x;
				start.y = q.v3.y;
				end.x = q.v1.x;
				end.y = q.v1.y;

				line(mat, start, end, lineColor, thickness, linetype);
			}
		}
	}

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