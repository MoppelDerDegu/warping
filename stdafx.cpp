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

bool Pathline::operator<(const Pathline &p) const
{
	return seedIndex < p.seedIndex;
}

bool Pathline::operator==(const Pathline &p) const
{
	if (seedIndex != p.seedIndex)
		return false;

	for (unsigned int i = 0; i < p.path.size(); i++)
	{
		const pair<int, Point2f> &elem = p.path.at(i);
		
		if (elem.first != path.at(i).first || elem.second.x != path.at(i).second.x || elem.second.y != path.at(i).second.y)
			return false;
	}

	return true;
}

// TODO: Auf zusätzliche Header verweisen, die in STDAFX.H
// und nicht in dieser Datei erforderlich sind.
