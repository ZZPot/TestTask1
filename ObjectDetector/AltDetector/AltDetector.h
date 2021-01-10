#pragma once
#include "../ObjectDetector.h"
#include "../../MovingPoints.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>

#define WND_NAME_BG_ALT		"Background(Alt)"


class AltDetector: public ObjectDetector
{
	public:
		AltDetector();
		~AltDetector();
		virtual bool AddFrame(cv::Mat frame);
		virtual std::vector<Obj2d> DetectObjects(cv::Mat frame);
	protected:
		int _frameCount;
		moving_points* mov_p;
};