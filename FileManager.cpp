#include "stdafx.h"
#include "FileManager.h"
#include "Helper.h"
#include "WarpingMath.h"
#include "MeshManager.h"

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

void FileManager::saveMeshAsImage(const string &fileName, const string &dir, const Mesh &m, const Size &s)
{
	const char* _dir = (char*) dir.c_str();
	mkDir(_dir);

	Mat mat = Helper::meshAsMat(m, s);

	imwrite(dir + fileName, mat);
}

void FileManager::saveMat(const string &fileName, const string &dir, const Mat &mat)
{
	const char* _dir = (char*) dir.c_str();
	mkDir(_dir);
	imwrite(dir + fileName, mat);
}

void FileManager::saveMeshAsText(const string &fileName, const string &dir, Mesh &mesh)
{
	MeshManager* mm = MeshManager::getInstance();

	vector<double> vertices = mm->meshToDoubleVec(mesh);

	const char* _dir = (char*) dir.c_str();
	mkDir(_dir);

	ofstream myfile;
	myfile.open(dir + fileName);

	if (myfile.is_open())
	{
		// write dimension of the mesh
		myfile << mesh.quadNumberX << "\n";
		myfile << mesh.quadNumberY << "\n";

		// write coordinates
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

Mesh FileManager::loadMesh(const string &fileName)
{
	MeshManager* mm = MeshManager::getInstance();

	vector<double> res;
	Mesh resmesh;
	string line;
	ifstream myfile;
	myfile.open(fileName);

	if (myfile.is_open())
	{
		int count = 0;
		while (myfile.good())
		{
			getline(myfile, line);

			if (count < 2)
			{
				if (count == 0)
					resmesh.quadNumberX = (int) Helper::stringToDouble(line);
				else
					resmesh.quadNumberY = (int) Helper::stringToDouble(line);

				count++;
			}
			else
				res.push_back(Helper::stringToDouble(line));
		}

		mm->doubleVecToMesh(res, resmesh);
	}
	else
		cerr << "Unable to open file: " << fileName << endl;

	return resmesh;
}

void FileManager::saveMeshROIAsImage(const string &fileName, const string &dir, const Mesh &m, const Size &s)
{
	const char* _dir = (char*) dir.c_str();
	mkDir(_dir);
	Mat mat = Helper::meshAsMat(m, s);

	Scalar lineColor(0, 0, 255); // red
	int thickness = 1;
	int linetype = 8;

	for (unsigned int i = 0; i < m.quads.size(); i++)
	{
		// place ROI over quad

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

		roiWidth = (int) WarpingMath::getDistance(_topleft, _topright);
		roiHeight = (int) WarpingMath::getDistance(_topleft, _bottomleft);

		bottomright.x = bottomleft.x + roiWidth;
		bottomright.y = topright.y + roiHeight;

		line(mat, topleft, topright, lineColor, thickness, linetype);
		line(mat, topright, bottomright, lineColor, thickness, linetype);
		line(mat, bottomright, bottomleft, lineColor, thickness, linetype);
		line(mat, bottomleft, topleft, lineColor, thickness, linetype);
	}

	imwrite(dir + fileName, mat);
}

void FileManager::saveMeshesAsText(const string &fileName, const string &dir, vector<Mesh> &meshes)
{
	MeshManager* mm = MeshManager::getInstance();
	vector<vector<double>> vertices;

	for (unsigned int i = 0; i < meshes.size(); i++)
	{
		vector<double> tmp = mm->meshToDoubleVec(meshes.at(i));
		vertices.push_back(tmp);
	}

	const char* _dir = (char*) dir.c_str();
	mkDir(_dir);

	ofstream myfile;
	myfile.open(dir + fileName);

	if (myfile.is_open())
	{
		for (unsigned int i = 0; i < vertices.size(); i++)
		{
			// write dimension of the mesh
			myfile << "-" << "\n";
			myfile << meshes.at(i).quadNumberX << "\n";
			myfile << meshes.at(i).quadNumberY << "\n";

			// write coordinates
			for (unsigned int j = 0; j < vertices.at(i).size(); j++)
			{
				if (j < vertices.at(i).size() - 1)
					myfile << vertices.at(i).at(j) << "\n";
				else
				{
					if (i < vertices.size() - 1)
						myfile << vertices.at(i).at(j) << "\n";
					else
						myfile << vertices.at(i).at(j) << "\n" << "-";
				}
			}
		}

		myfile.close();
	}
	else
		cerr << "Unable to open file: " << fileName << endl;
}

vector<Mesh> FileManager::loadMeshes(const string &fileName)
{
	MeshManager* mm = MeshManager::getInstance();

	Mesh mesh;
	vector<double> vertexCoords;
	vector<Mesh> res;
	string line;
	ifstream myfile;
	myfile.open(fileName);

	if (myfile.is_open())
	{
		int count = 0;
		bool first = true;
		while (myfile.good())
		{
			getline(myfile, line);

			if (line.compare("-") == 0)
			{
				if (!first)
				{
					mm->doubleVecToMesh(vertexCoords, mesh);
					res.push_back(mesh);
					vertexCoords.clear();
				}
				
				count = 0;
				first = false;
			}
			else
			{
				if (count < 2)
				{
					if (count == 0)
						mesh.quadNumberX = (int) Helper::stringToDouble(line);
					else
						mesh.quadNumberY = (int) Helper::stringToDouble(line);

					count++;
				}
				else
					vertexCoords.push_back(Helper::stringToDouble(line));
			}
		}
	}
	else
		cerr << "Unable to open file: " << fileName << endl;

	return res;
}

void FileManager::savePathlines(const string &fileName, const string &dir, const vector<Pathline> &pathlines)
{
	const char* _dir = (char*) dir.c_str();
	mkDir(_dir);

	ofstream myfile;
	myfile.open(dir + fileName);

	if (myfile.is_open())
	{
		for (unsigned int i = 0; i < pathlines.size(); i++)
		{
			Pathline pl = pathlines.at(i);

			myfile << pl.seedIndex << " ";

			for (unsigned int j = 0; j < pl.path.size(); j++)
			{
				pair<int, Point2f> pair = pl.path.at(j);

				myfile << pair.first << " " << pair.second.x << " " << pair.second.y << " ";
			}

			myfile << "\n";
		}

		myfile.close();
	}
	else
		cerr << "Unable to open file: " << fileName << endl;

}

vector<Pathline> FileManager::loadPathlines(const string &fileName)
{
	vector<Pathline> result;

	string line;
	ifstream myfile;
	myfile.open(fileName);

	if (myfile.is_open())
	{
		while (myfile.good())
		{
			getline(myfile, line);

			vector<string> tokens = Helper::split(line, ' ');

			if (tokens.empty())
				break;

			Pathline pl;

			// seed index of the pathline
			pl.seedIndex = Helper::stringToInt(tokens.at(0));
			
			for (unsigned int i = 1; i < tokens.size(); i += 3)
			{
				pair<int, Point2f> pair;

				// number of frame
				pair.first = Helper::stringToInt(tokens.at(i));

				// point of the pathline
				pair.second = Point2f(Helper::stringToInt(tokens.at(i + 1)), Helper::stringToInt(tokens.at(i + 2)));

				pl.path.push_back(pair);
			}

			result.push_back(pl);
		}
	}
	else
		cerr << "Unable to open file: " << fileName << endl;

	return result;
}