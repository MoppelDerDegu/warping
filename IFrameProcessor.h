#pragma once

#include "stdafx.h"

class IFrameProcessor
{

protected:
	virtual void process(Mat &input) = 0;
};

