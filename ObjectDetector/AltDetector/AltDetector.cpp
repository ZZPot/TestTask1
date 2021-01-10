#include "AltDetector.h"


AltDetector::AltDetector():
	mov_p(nullptr)
{
	_frameCount = 0;
}
AltDetector::~AltDetector()
{
	if(mov_p != nullptr)
	{
		delete mov_p;
	}
}

bool AltDetector::AddFrame(cv::Mat frame)
{
	_frameCount++;
	if(mov_p == nullptr)
	{
		mov_p = new moving_points(frame);
	}
	mov_p->AddFrame(frame);
	if (_frameCount == 1)
	{
		return false;
	}
	return true;
}
std::vector<Obj2d> AltDetector::DetectObjects(cv::Mat frame)
{
	Obj2d obj;
	if (!mov_p->GetObj(&obj))
		return std::vector<Obj2d>();
	GetObj2d(&obj);
	return { obj };
}
