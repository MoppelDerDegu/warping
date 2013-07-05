// stdafx.h : Includedatei für Standardsystem-Includedateien
// oder häufig verwendete projektspezifische Includedateien,
// die nur in unregelmäßigen Abständen geändert werden.
//

#ifndef HEADER_H
#define HEADER_H

#pragma once
#pragma warning (disable : 4996)
#pragma warning (disable : 4244)
#pragma warning (disable : 4091)
#pragma warning (disable : 4305)
#pragma warning (disable : 4018)

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>
#include <Windows.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/legacy/legacy.hpp>


using namespace cv;
using namespace std;

#define GRADIENT_SOBEL 1
#define GRADIENT_SCHARR 2
#define GRADIENT_SIMPLE 3

// length and height of a quad
#define QUAD_SIZE 20

typedef struct Pathline
{
	// frame index and location in the frame
	vector<pair<int, Point2f>> path;

	// indicates on which vertex this path line was originally seeded
	unsigned int seedIndex;
};

typedef struct PathlineAdjacencies
{
	// stores neighboring pathlines
	vector<pair<unsigned int, unsigned int>> neighbors;
};

typedef struct PathlineSets
{
	// inner vector specifies pathlines between frame i, i+1, ... , j
	// outer vector is the set of all these pathlines in the whole video, i.e. between frame 1 ... n
	vector<vector<Pathline>> pathlines;
};

typedef struct Vertex
{
	int x, y;
	Vertex operator+ (Vertex);
	Vertex operator- (Vertex);
	Vertex operator* (float);
	Vertex operator* (double);
	Vertex operator* (int);
	bool operator== (Vertex &v);
};

typedef struct Edge
{
	Vertex src;
	Vertex dest;
	bool operator== (Edge &e);
};

// vertex and edge numeration of a quad is like this
/*        e1
	v1 --------- v2
	|			 |
 e4	|			 | e2
	|			 |
	v3 --------- v4
		  e3
*/
typedef struct Quad
{
	Vertex v1;
	Vertex v2;
	Vertex v3;
	Vertex v4;
	bool operator== (Quad &q);
};

typedef struct Mesh
{
	int quadNumberX;
	int quadNumberY;
	vector<Vertex> vertices;
	vector<Edge> edges;
	vector<Quad> quads;
};

template<typename T> inline T sqr(T x) { return x * x;}
template<class T> inline T vecDist3(const Vec<T, 3> &v1, const Vec<T, 3> &v2) {return sqrt(sqr(v1[0] - v2[0])+sqr(v1[1] - v2[1])+sqr(v1[2] - v2[2]));}
template<class T> inline T vecSqrDist3(const Vec<T, 3> &v1, const Vec<T, 3> &v2) {return sqr(v1[0] - v2[0])+sqr(v1[1] - v2[1])+sqr(v1[2] - v2[2]);}
template<class T1, class T2> inline void operator /= (Vec<T1, 3> &v1, const T2 v2) { v1[0] /= v2; v1[1] /= v2; v1[2] /= v2; }
template<class T> inline T pntSqrDist(const Point_<T> &p1, const Point_<T> &p2) {return sqr(p1.x - p2.x) + sqr(p1.y - p2.y);}


// TODO: Hier auf zusätzliche Header, die das Programm erfordert, verweisen.
#endif