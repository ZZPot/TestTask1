#include "BGSDetector.h"


BGSDetector::BGSDetector(int framesToAdapt):
	_framesToAdapt(framesToAdapt)
{
	_pMOG2 = cv::createBackgroundSubtractorMOG2(_framesToAdapt, 16, false);
	_frameCount = 0;
}
BGSDetector::~BGSDetector()
{
	
}

bool BGSDetector::AddFrame(cv::Mat frame)
{
	_frameCount++;
	_pMOG2->apply(frame, _fgMask);
	if (_frameCount <= _framesToAdapt)
	{
		return false;
	}
	//cv::Mat bg_img;
	// Causes exception, dunno why
	//_pMOG2->getBackgroundImage(bg_img);
	//cv::imshow(WND_NAME_BG_BGS, bg_img);
	return true;
}
std::vector<Obj2d> BGSDetector::DetectObjects(cv::Mat frame)
{
	std::vector<Obj2d> res;
	if (_frameCount <= _framesToAdapt)
	{
		return res;
	}
	
	res = FindObjects(_fgMask,
		std::vector<type_condition>(), std::vector<int>(), cv::RETR_EXTERNAL);
	return res;
}
