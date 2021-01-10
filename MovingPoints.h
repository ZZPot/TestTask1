#pragma once
#include "FeatureDetector/FeatureDetector.h"

#define MIN_POINTS_COUNT	4

class moving_points
{
public:
	moving_points(cv::Mat init_frame);
	void AddFrame(cv::Mat new_frame);
	bool GetObj(Obj2d* obj);
protected:
	void AddPoints();
	bool AcceptPoint(unsigned id);
	std::vector<cv::Point2f> _points[2];
	cv::Mat _frames[2];

	double _qlevel;
	int _max_count;
	double _min_dist;
	std::vector<unsigned char> _point_status;
	std::vector<float> _point_error;
};

