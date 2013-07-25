#include "PathlineManager.h"
#include "MeshManager.h"
#include "Helper.h"
#include "WarpingMath.h"

bool PathlineManager::instanceFlag = false;
PathlineManager* PathlineManager::single = NULL;

PathlineManager* PathlineManager::getInstance()
{
	if (!instanceFlag)
	{
		single = new PathlineManager();
		instanceFlag = true;
	}

	return single;
}

PathlineManager::~PathlineManager(void)
{
	instanceFlag = false;
}

void PathlineManager::getAdjacencies(PathlineSets &sets, Mesh &seedMesh, Size &seedMeshSize, PathlineAdjacencies &result)
{
	vector<pair<unsigned int, unsigned int>> tuples;
	PathlineAdjacencies restmp;
	result.neighbors.clear();

	MeshManager* mm = MeshManager::getInstance();

	int vertexNumberX = seedMesh.quadNumberX - 1;
	int vertexNumberY = seedMesh.quadNumberY - 1;
	
	Mesh tmp;
	tmp.quadNumberX = vertexNumberX - 1;
	tmp.quadNumberY = vertexNumberY - 1;

	vector<Pathline> lines = sets.pathlines.at(0);

	// divide lines in half since we are only interested in the left pathlines
	// this assumes that seed indices of the left and right pathlines are identical
	lines.resize(lines.size() / 2);

	tmp.vertices = mm->getInnerVertices(seedMesh, seedMeshSize);
	mm->buildQuadsAndEdges(tmp);

	// i and j are neighbors
	int i, j;

	// find neighbors
	for (unsigned int k = 0; k < tmp.quads.size(); k++)
	{
		Quad q = tmp.quads.at(k);
		for (unsigned int o = 0; o < 4; o++)
		{
			// find i
			for (unsigned int l = 0; l < lines.size(); l++)
			{
				Vertex v;

				if (o == 0)
				{
					v.x = lines.at(l).path.at(0).second.x;
					v.y = lines.at(l).path.at(0).second.y;

					if (v == q.v1)
					{
						i = lines.at(l).seedIndex;
						break;
					}
				}
				else if (o == 1)
				{
					v.x = lines.at(l).path.at(0).second.x;
					v.y = lines.at(l).path.at(0).second.y;

					if (v == q.v2)
					{
						i = lines.at(l).seedIndex;
						break;
					}
				}
				else if (o == 2)
				{
					v.x = lines.at(l).path.at(0).second.x;
					v.y = lines.at(l).path.at(0).second.y;

					if (v == q.v3)
					{
						i = lines.at(l).seedIndex;
						break;
					}
				}
				else if (o == 3)
				{
					v.x = lines.at(l).path.at(0).second.x;
					v.y = lines.at(l).path.at(0).second.y;

					if (v == q.v4)
					{
						i = lines.at(l).seedIndex;
						break;
					}
				}
			}

			// find j
			for (unsigned int l = 0; l < lines.size(); l++)
			{
				Vertex v;

				if (o == 0)
				{
					// neighbor of v1

					v.x = lines.at(l).path.at(0).second.x;
					v.y = lines.at(l).path.at(0).second.y;

					if (v == q.v2)
					{
						j = lines.at(l).seedIndex;
						break;
					}
				}
				else if (o == 1)
				{
					// neighbor of v2

					v.x = lines.at(l).path.at(0).second.x;
					v.y = lines.at(l).path.at(0).second.y;

					if (v == q.v4)
					{
						j = lines.at(l).seedIndex;
						break;
					}
				}
				else if (o == 2)
				{
					// neighbor of v3

					v.x = lines.at(l).path.at(0).second.x;
					v.y = lines.at(l).path.at(0).second.y;

					if (v == q.v1)
					{
						j = lines.at(l).seedIndex;
						break;
					}
				}
				else if (o == 3)
				{
					// neighbor of v4

					v.x = lines.at(l).path.at(0).second.x;
					v.y = lines.at(l).path.at(0).second.y;

					if (v == q.v3)
					{
						j = lines.at(l).seedIndex;
						break;
					}
				}
			}

			pair<unsigned int, unsigned int> neighbors;
			neighbors.first = i;
			neighbors.second = j;

			restmp.neighbors.push_back(neighbors);
		}
	}

	result = restmp;

	//eliminate duplicates
	for (unsigned int a = 0; a < restmp.neighbors.size(); a++)
	{
		pair<unsigned int, unsigned int> p1 = restmp.neighbors.at(a);

		for (unsigned int b = 0; b < restmp.neighbors.size(); b++)
		{
			pair<unsigned int, unsigned int> p2 = restmp.neighbors.at(b);
			pair<unsigned int, unsigned int> reci(p1.second, p1.first);

			if (reci == p2 && !Helper::contains(tuples, reci))
			{
				tuples.push_back(p1);
				result.neighbors.erase(remove(result.neighbors.begin(), result.neighbors.end(), p2), result.neighbors.end());
				break;
			}
			else if (Helper::contains(tuples, reci))
				break;
		}
	}
}

void PathlineManager::createPathlineMatrixMapping(PathlineSets &pathlineSets, PathlineMatrixMapping &result, Size &oldSize, Size &newSize)
{
	result.mapping.clear();

	double x = newSize.width / (float) oldSize.width;
	double y = newSize.height / (float) oldSize.height;

	for (unsigned int i = 0; i < pathlineSets.pathlines.size(); i++)
	{
		map<Pathline, ScalingMatrix2x2> mapping;

		for (unsigned int j = 0; j < pathlineSets.pathlines.at(i).size(); j++)
		{
			ScalingMatrix2x2 mat;
			mat.vx = x;
			mat.vy = y;

			pair<Pathline, ScalingMatrix2x2> pair(pathlineSets.pathlines.at(i).at(j), mat);
			
			mapping.insert(pair);
		}

		result.mapping.push_back(mapping);
	}
}

void PathlineManager::createPathlineTransVecMapping(PathlineSets &pathlineSets, PathlineTransVecMapping &result)
{
	result.mapping.clear();

	for (unsigned int i = 0; i < pathlineSets.pathlines.size(); i++)
	{
		map<Pathline, TranslationVector2> mapping;

		for (unsigned int j = 0; j < pathlineSets.pathlines.at(i).size(); j++)
		{
			TranslationVector2 vec;
			vec.x = 0.0;
			vec.y = 0.0;

			pair<Pathline, TranslationVector2> pair(pathlineSets.pathlines.at(i).at(j), vec);

			mapping.insert(pair);
		}

		result.mapping.push_back(mapping);
	}
}

pair<Pathline, Pathline> PathlineManager::getNeighbors(pair<unsigned int, unsigned int> &neighbors, vector<Pathline> &pathlines)
{
	pair<Pathline, Pathline> pair;
	
	for (unsigned int k = 0; k < pathlines.size(); k++)
	{
		if (pathlines.at(k).seedIndex == neighbors.first)
		{
			pair.first = pathlines.at(k);
			break;
		}
	}

	for (unsigned int k = 0; k < pathlines.size(); k++)
	{
		if (pathlines.at(k).seedIndex == neighbors.second)
		{
			pair.second = pathlines.at(k);
			break;
		}
	}

	return pair;
}

void PathlineManager::mappingsToDoubleVec(map<Pathline, ScalingMatrix2x2> &matMapping, map<Pathline, TranslationVector2> &vecMapping, int numberOfDummyVariables, vector<double> &result)
{
	result.clear();

	for (map<Pathline, ScalingMatrix2x2>::iterator it = matMapping.begin(); it != matMapping.end(); ++it)
	{
		result.push_back(it->second.vx);
		result.push_back(it->second.vy);
	}

	for (map<Pathline, TranslationVector2>::iterator it = vecMapping.begin(); it != vecMapping.end(); ++it)
	{
		result.push_back(it->second.x);
		result.push_back(it->second.y);
	}

	vector<double> tmp;
	
	// dummy variables for scaling matrices between pairing pathlines
	for (unsigned int i = 0; i < numberOfDummyVariables; i++)
	{
		tmp.push_back(1.0);
	}

	// append dummy variables to the resulting vector
	result.insert(result.end(), tmp.begin(), tmp.end());
}

void PathlineManager::splitPathlineSets(PathlineSets &original, PathlineSets &left, PathlineSets &right)
{
	left.pathlines.clear();
	right.pathlines.clear();

	left.pathlines = original.pathlines;
	right.pathlines = original.pathlines;

	for (unsigned int i = 0; i < original.pathlines.size(); i++)
	{
		// left
		vector<Pathline>::iterator leftFirst = original.pathlines.at(i).begin();
		vector<Pathline>::iterator leftLast = original.pathlines.at(i).begin() + original.pathlines.at(i).size() / 2;

		vector<Pathline> leftSub(leftFirst, leftLast);
		left.pathlines.at(i) = leftSub;

		// right
		vector<Pathline>::iterator rightFirst = original.pathlines.at(i).begin() + original.pathlines.at(i).size() / 2;
		vector<Pathline>::iterator rightLast = original.pathlines.at(i).begin() + original.pathlines.at(i).size();

		vector<Pathline> rightSub(rightFirst, rightLast);
		right.pathlines.at(i) = rightSub;
	}
}

void PathlineManager::createNeighborMatrixMapping(PathlineSets &pathlineSets, PathlineAdjacencies &adjacencies, NeighborMatrixMapping &result, Size &oldSize, Size &newSize)
{
	result.mapping.clear();

	double x = newSize.width / (float) oldSize.width;
	double y = newSize.height / (float) oldSize.height;

	for (unsigned int i = 0; i < pathlineSets.pathlines.size(); i++)
	{
		map<pair<unsigned int, unsigned int>, ScalingMatrix2x2> mapping;

		for (unsigned int j = 0; j < adjacencies.neighbors.size(); j++)
		{
			pair<pair<unsigned int, unsigned int>, ScalingMatrix2x2> pair;

			pair.first = adjacencies.neighbors.at(j);
			
			ScalingMatrix2x2 si;
			si.vx = x;
			si.vy = y;

			pair.second = si;

			mapping.insert(pair);
		}

		result.mapping.push_back(mapping);
	}
}

void PathlineManager::mergePathlineSets(PathlineSets &left, PathlineSets &right, PathlineSets &result)
{
	if (left.pathlines.size() != right.pathlines.size())
		throw invalid_argument("Left and right Pathlines are not equivalent.");

	result.pathlines.clear();

	for (unsigned int i = 0; i < left.pathlines.size(); i++)
	{
		vector<Pathline> lines;
		lines.insert(lines.end(), left.pathlines.at(i).begin(), left.pathlines.at(i).end());
		lines.insert(lines.end(), right.pathlines.at(i).begin(), right.pathlines.at(i).end());

		result.pathlines.push_back(lines);
	}
}

void PathlineManager::mapPathlinesToQuads(int frame, PathlineSets &pathlines, Mesh &mesh, map<int, int> &result)
{
	result.clear();

	vector<Point2f> points;
	vector<Pathline> lines;
	getLinesContainingFrame(pathlines, frame, lines);
	getPointsInFrame(lines, frame, points);
	
	for (unsigned int i = 0; i < points.size(); i++)
	{
		for (unsigned int j = 0; j < mesh.quads.size(); j++)
		{
			if (liesInQuad(mesh.quads.at(j), points.at(i)))
			{
				result.insert(pair<int, int>(i, j));
				break;
			}
		}
	}
}

void PathlineManager::getPointsInFrame(vector<Pathline> &lines, int frame, vector<Point2f> &result)
{
	for (auto it = lines.begin(); it != lines.end(); ++it)
	{
		vector<pair<int, Point2f>> &path = it->path;

		for (auto iter = path.begin(); iter != path.end(); ++iter)
		{
			if (iter->first == frame)
			{
				result.push_back(iter->second);
				break;
			}
		}
	}
}

bool PathlineManager::liesInQuad(Quad &quad, Point2f &point)
{
	bool in = true;

	if (WarpingMath::area(quad, point) > WarpingMath::area(quad))
		in = false;

	return in;
}

void PathlineManager::getLinesContainingFrame(PathlineSets &pathlines, int frame, vector<Pathline> &result)
{
	for (unsigned int i = 0; i < pathlines.pathlines.size(); i++)
	{
		for (unsigned int j = 0; j < pathlines.pathlines.at(i).size(); j++)
		{
			for (unsigned int k = 0; k < pathlines.pathlines.at(i).at(j).path.size(); k++)
			{
				if (pathlines.pathlines.at(i).at(j).path.at(k).first == frame)
				{
					result = pathlines.pathlines.at(i);
					break;
				}
			}
		}
	}
}