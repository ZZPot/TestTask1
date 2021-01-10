#pragma once
#include "../ObjectDetector.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>

#define WND_NAME_BG_BGS		"Background(BGS)"


class BGSDetector: public ObjectDetector
{
	public:
		BGSDetector(int framesToAdapt = 4);
		~BGSDetector();
		virtual bool AddFrame(cv::Mat frame);
		virtual std::vector<Obj2d> DetectObjects(cv::Mat frame);
	protected:
		cv::Mat _fgMask;
		int _framesToAdapt;
		cv::Ptr<cv::BackgroundSubtractorMOG2> _pMOG2;
		int _frameCount;
};