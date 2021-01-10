#pragma once
#include <opencv2/core.hpp>
#include <FeatureDetector/FeatureDetector.h>

class ObjectDetector
{
	public:
		virtual bool AddFrame(cv::Mat frame) = 0;
		virtual std::vector<Obj2d> DetectObjects(cv::Mat frame) = 0;
		virtual ~ObjectDetector(){}
};