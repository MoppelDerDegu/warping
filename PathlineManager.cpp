#include "PathlineManager.h"
#include "MeshManager.h"
#include "Helper.h"

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