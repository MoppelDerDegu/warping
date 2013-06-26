// stdafx.cpp : Quelldatei, die nur die Standard-Includes einbindet.
// 3D_Video_Saliency_Detector.pch ist der vorkompilierte Header.
// stdafx.obj enthält die vorkompilierten Typinformationen.

#include "stdafx.h"

Vertex Vertex::operator+ (Vertex param)
{
	Vertex tmp;
	tmp.x = x + param.x;
	tmp.y = y + param.y;
	return tmp;
}

Vertex Vertex::operator- (Vertex param)
{
	Vertex tmp;
	tmp.x = x - param.x;
	tmp.y = y - param.y;
	return tmp;
}

bool Vertex::operator== (Vertex &v)
{
	return (v.x == x && v.y == y);
}

bool Edge::operator== (Edge &e)
{
	return (e.src == src && e.dest == dest);
}

bool Quad::operator== (Quad &q)
{
	return (q.v1 == v1 && q.v2 == v2 && q.v3 == v3 && q.v4 == v4);
}

Vertex Vertex::operator* (float param)
{
	Vertex tmp;
	tmp.x = x * param;
	tmp.y = y * param;
	return tmp;
}

Vertex Vertex::operator* (double param)
{
	Vertex tmp;
	tmp.x = x * param;
	tmp.y = y * param;
	return tmp;
}

Vertex Vertex::operator* (int param)
{
	Vertex tmp;
	tmp.x = x * param;
	tmp.y = y * param;
	return tmp;
}

// TODO: Auf zusätzliche Header verweisen, die in STDAFX.H
// und nicht in dieser Datei erforderlich sind.
