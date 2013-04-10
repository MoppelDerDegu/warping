#include "stdafx.h"

// number of quads in the image in x and y direction respectively
#define QUAD_NUMBER_X 30
#define QUAD_NUMBER_Y 30
#define QUAD_NUMBER_TOTAL QUAD_NUMBER_X * QUAD_NUMBER_Y

typedef struct Edge
{
	Vertex src;
	Vertex dest;
};

// vertex numeration of a quad is like this
/*
	v1 --------- v2
	|			 |
	|			 |
	|			 |
	v3 --------- v4
*/
typedef struct Quad
{
	Vertex v1;
	Vertex v2;
	Vertex v3;
	Vertex v4;
};

typedef struct Mesh
{
	vector<Vertex> vertices;
	vector<Edge> edges;
	vector<Quad> quads;
};