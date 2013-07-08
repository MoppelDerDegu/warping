#pragma once

#include "stdafx.h"

class IFrameProcessor
{

protected:
	virtual void process(Mat &currentFrame, Mat &prevFrame) = 0;
};

