#include "MeshManager.h"
#include "stdafx.h"
#include "WarpingMath.h"

bool MeshManager::instanceFlag = false;
MeshManager* MeshManager::single = NULL;

MeshManager* MeshManager::getInstance()
{
    if (!instanceFlag)
    {
        single = new MeshManager();
        instanceFlag = true;
    }

    return single;
}

/*
	The order of the vertices being pushed to the vector is as follows:

	v1---v2---v13--v19
	|	 |	  |		|
	v3---v4---v14--v20
	|	 |    |		|
	v5---v6---v15--v21
	|	 |	  |	    |
	v7---v8---v16--v22
	|	 |	  |		|
	v9---v10--v17--v23
	|	 |	  |	    |
	v11--v12--v18--v24
*/
void MeshManager::initializeMesh(Mesh &result, Size &size)
{
	cout << ">> Initialize mesh" << endl;

	int x, y, numberOfQuads, quadNumberX, quadNumberY;
	numberOfQuads = determineQuadNumber(size, quadNumberX, quadNumberY);

	result.quadNumberX = quadNumberX;
	result.quadNumberY = quadNumberY;

	// traverses image column-wise and creates quads
	for (int i = 0; i < numberOfQuads; i++)
	{
		Quad q;
		Edge e1, e2, e3, e4;

		x = (int) i / quadNumberY;
		y = i % quadNumberY;

		if (x < quadNumberX - 1 &&  y < quadNumberY - 1)
		{
			// inner quads
			q.v1.x = x * QUAD_SIZE;
			q.v1.y = y * QUAD_SIZE;

			q.v2.x = (x + 1) * QUAD_SIZE;
			q.v2.y = y * QUAD_SIZE;

			q.v3.x = x * QUAD_SIZE;
			q.v3.y = (y + 1) * QUAD_SIZE;

			q.v4.x = (x + 1) * QUAD_SIZE;
			q.v4.y = (y + 1) * QUAD_SIZE;
		}
		else
		{
			if (x == quadNumberX - 1 && y != quadNumberY - 1)
			{
				// rightmost quads
				q.v1.x = x * QUAD_SIZE;
				q.v1.y = y * QUAD_SIZE;

				q.v2.x = size.width;
				q.v2.y = y * QUAD_SIZE;

				q.v3.x = x * QUAD_SIZE;
				q.v3.y = (y + 1) * QUAD_SIZE;

				q.v4.x = size.width;
				q.v4.y = (y + 1) * QUAD_SIZE;
			}
			else if (x != quadNumberX - 1 && y == quadNumberY - 1)
			{
				// bottom quads
				q.v1.x = x * QUAD_SIZE;
				q.v1.y = y * QUAD_SIZE;

				q.v2.x = (x + 1) * QUAD_SIZE;
				q.v2.y = y * QUAD_SIZE;

				q.v3.x = x * QUAD_SIZE;
				q.v3.y = size.height;

				q.v4.x = (x + 1) * QUAD_SIZE;
				q.v4.y = size.height;
			}
			else if (x == quadNumberX - 1 && y == quadNumberY - 1)
			{
				// bottom right quad
				q.v1.x = x * QUAD_SIZE;
				q.v1.y = y * QUAD_SIZE;

				q.v2.x = size.width;
				q.v2.y = y * QUAD_SIZE;

				q.v3.x = x * QUAD_SIZE;
				q.v3.y = size.height;

				q.v4.x = size.width;
				q.v4.y = size.height;
			}
		}

		e1.src = q.v1;
		e1.dest = q.v2;
		e2.src = q.v2;
		e2.dest = q.v4;
		e3.src = q.v4;
		e3.dest = q.v3;
		e4.src = q.v3;
		e4.dest = q.v1;

		result.quads.push_back(q);

		if (i == 0)
		{
			result.vertices.push_back(q.v1);
			result.vertices.push_back(q.v2);
			result.vertices.push_back(q.v3);
			result.vertices.push_back(q.v4);
			result.edges.push_back(e1);
			result.edges.push_back(e2);
			result.edges.push_back(e3);
			result.edges.push_back(e4);
		}
		else
		{
			if (x == 0)
			{
				// don't add front edge of a quad since it's already part of the mesh
				result.edges.push_back(e2);
				result.edges.push_back(e3);
				result.edges.push_back(e4);

				// don't add v1 and v2 since they are redundant
				result.vertices.push_back(q.v3);
				result.vertices.push_back(q.v4);
			}
			else
			{
				if (y == 0)
				{
					// don't add the left-hand edge of a quad since it's redundant
					result.edges.push_back(e1);
					result.edges.push_back(e2);
					result.edges.push_back(e3);

					// don't add v1 and v3 since they are redundant
					result.vertices.push_back(q.v2);
					result.vertices.push_back(q.v4);
				}
				else
				{
					// don't add top and left-hand edge since they are redundant
					result.edges.push_back(e2);
					result.edges.push_back(e3);

					// only add bottom right vertex
					result.vertices.push_back(q.v4);
				}
			}
		}
	}
}

Mesh MeshManager::deepCopyMesh(const Mesh &m)
{
	Mesh result;
	
	result.quadNumberX = m.quadNumberX;
	result.quadNumberY = m.quadNumberY;

	for (unsigned int i = 0; i < m.quads.size(); i++)
	
	{
		Quad f;
		Vertex v1;
		Vertex v2;
		Vertex v3;
		Vertex v4;
		
		v1.x = m.quads.at(i).v1.x;
		v1.y = m.quads.at(i).v1.y;
		v2.x = m.quads.at(i).v2.x;
		v2.y = m.quads.at(i).v2.y;
		v3.x = m.quads.at(i).v3.x;
		v3.y = m.quads.at(i).v3.y;
		v4.x = m.quads.at(i).v4.x;
		v4.y = m.quads.at(i).v4.y;

		f.v1 = v1;
		f.v2 = v2;
		f.v3 = v3;
		f.v4 = v4;

		result.quads.push_back(f);
	}

	for (unsigned int i = 0; i < m.edges.size(); i++)
	{
		Edge edge;
		edge.src = m.edges.at(i).src;
		edge.dest = m.edges.at(i).dest;
		
		result.edges.push_back(edge);
	}

	for (unsigned int i = 0; i < m.vertices.size(); i++)
	{
		result.vertices.push_back(m.vertices.at(i));
	}

	return result;
}

vector<double> MeshManager::meshToDoubleVec(Mesh &m)
{
	vector<double> x;

	for (unsigned int i = 0; i < m.vertices.size(); i++)
	{
		x.push_back(m.vertices.at(i).x);
		x.push_back(m.vertices.at(i).y);
	}

	return x;
}

void MeshManager::doubleVecToMesh(const vector<double> &x, Mesh &result)
{
	// clear mesh
	result.vertices.clear();
	result.edges.clear();
	result.quads.clear();

	int quadNumberX = result.quadNumberX;
	int quadNumberY = result.quadNumberY;
	int totalNumber = quadNumberX * quadNumberY;

	// vertices
	for (unsigned int i = 0; i < x.size(); i += 2)
	{
		Vertex v;
		v.x = WarpingMath::round(x.at(i));
		v.y = WarpingMath::round(x.at(i + 1));

		result.vertices.push_back(v);
	}

	int vertexCount = 0;
	int diff = quadNumberY * 2 + 1;
	int xfac, yfac;

	// quads and edges
	for (int i = 0; i < totalNumber; i++)
	{
		Quad q;
		Edge e1, e2, e3, e4;

		xfac = (int) i / quadNumberY;
		yfac = i % quadNumberY;

		if (vertexCount <= quadNumberY * 2)
		{
			// first column of mesh
			q.v1 = result.vertices.at(vertexCount);
			q.v2 = result.vertices.at(vertexCount + 1);
			q.v3 = result.vertices.at(vertexCount + 2);
			q.v4 = result.vertices.at(vertexCount + 3);

			if (vertexCount < (quadNumberY - 1) * 2)
				vertexCount += 2;
			else
				vertexCount += 4;
		}
		else
		{
			if (xfac < 2)
			{
				// second column
				q.v1 = result.vertices.at(vertexCount - diff);
				q.v2 = result.vertices.at(vertexCount);
				q.v3 = result.vertices.at(vertexCount + 1 - (diff - 1));
				q.v4 = result.vertices.at(vertexCount + 1);
			}
			else
			{
				// all other columns
				q.v1 = result.vertices.at(vertexCount - diff);
				q.v2 = result.vertices.at(vertexCount);
				q.v3 = result.vertices.at(vertexCount + 1 - diff);
				q.v4 = result.vertices.at(vertexCount + 1);
			}

			if (i < quadNumberY * 2)
				diff--;	

			if (yfac == quadNumberY - 1)
				vertexCount += 2;
			else
				vertexCount++;
		}

		e1.src = q.v1;
		e1.dest = q.v2;
		e2.src = q.v2;
		e2.dest = q.v4;
		e3.src = q.v4;
		e3.dest = q.v3;
		e4.src = q.v3;
		e4.dest = q.v1;

		result.quads.push_back(q);

		if (i == 0)
		{
			result.edges.push_back(e1);
			result.edges.push_back(e2);
			result.edges.push_back(e3);
			result.edges.push_back(e4);
		}
		else
		{
			if (xfac == 0)
			{
				// don't add front edge of a quad since it's already part of the mesh
				result.edges.push_back(e2);
				result.edges.push_back(e3);
				result.edges.push_back(e4);
			}
			else
			{
				if (yfac == 0)
				{
					// don't add the left-hand edge of a quad since it's redundant
					result.edges.push_back(e1);
					result.edges.push_back(e2);
					result.edges.push_back(e3);
				}
				else
				{
					// don't add top and left-hand edge since they are redundant
					result.edges.push_back(e2);
					result.edges.push_back(e3);
				}
			}
		}
	}
}

int MeshManager::determineQuadNumber(Size & size, int &quadNumberX, int &quadNumberY)
{
	int xnumber, ynumber;

	if (size.width % QUAD_SIZE < QUAD_SIZE / 2)
		xnumber = size.width / QUAD_SIZE;
	else
		xnumber = 1 + (size.width / QUAD_SIZE);

	if (size.height % QUAD_SIZE < QUAD_SIZE / 2)
		ynumber = size.height / QUAD_SIZE;
	else
		ynumber = 1 + (size.height / QUAD_SIZE);
	
	quadNumberX = xnumber;
	quadNumberY = ynumber;

	return xnumber * ynumber;
}

Mesh MeshManager::generateRightEyeMesh(Mesh &leftEyeMesh, StereoImage* img, Size &rightEyeSize)
{
	Mesh result;
	
	vector<Point2f> initial;
	vector<Point2f> detected;

	// maps indices of the vector to the index of the vertex in the mesh
	map<int, int> indexMap;

	int key = 0; // index of the tracked points

	// track only inner points of the mesh
	for (unsigned int i = 0; i < leftEyeMesh.vertices.size(); i++)
	{
		// assuming lefteye size == righteye size
		if (!(leftEyeMesh.vertices.at(i).x == 0 || leftEyeMesh.vertices.at(i).x == rightEyeSize.width ||
			leftEyeMesh.vertices.at(i).y == 0 || leftEyeMesh.vertices.at(i).y == rightEyeSize.height))
		{
			Point2f p;
			p.x = leftEyeMesh.vertices.at(i).x;
			p.y = leftEyeMesh.vertices.at(i).y;
			
			initial.push_back(p);

			indexMap.insert(pair<int, int>(key, i));
			key++;
		}
	}

	// TODO look for feature points in the right eye view

	return result;
}