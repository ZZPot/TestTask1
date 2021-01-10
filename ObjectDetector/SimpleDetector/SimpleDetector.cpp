#include "SimpleDetector.h"


SimpleDetector::SimpleDetector()
{
	_frameCount = 0;
}
SimpleDetector::~SimpleDetector()
{
	
}

bool SimpleDetector::AddFrame(cv::Mat frame)
{
	_frameCount++;
	cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
	threshold(frame, frameBW, 0, 255, cv::THRESH_OTSU);
	if (countNonZero(frameBW) > ((frameBW.cols * frameBW.rows) / 2))
		frameBW = 1 - frameBW;
	return true;
}
std::vector<Obj2d> SimpleDetector::DetectObjects(cv::Mat frame)
{
	std::vector<Obj2d> res = FindObjects(frameBW,
		std::vector<type_condition>(), std::vector<int>(), cv::RETR_EXTERNAL);
	return res;
}
