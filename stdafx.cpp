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

// TODO: Auf zusätzliche Header verweisen, die in STDAFX.H
// und nicht in dieser Datei erforderlich sind.
