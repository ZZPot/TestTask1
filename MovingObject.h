#pragma once
#include "FeatureDetector/FeatureDetector.h"
#include <queue>

#define COLOR_OBJECT	0
#define COLOR_TRACE		1
#define COLOR_MOVE		2
#define COLOR_LINE		3
#define COLOR_MARK		4
#define COLOR_CUT		5
#define MARK_TRACE		cv::MARKER_TRIANGLE_DOWN
#define MARK_POINT		cv::MARKER_TILTED_CROSS

extern std::vector<cv::Scalar> basic_colors; 
struct line_segment
{
	cv::Point2f p1;
	cv::Point2f p2;
};

class moving_object
{
public:
	moving_object(Obj2d obj);
	void NewPos(Obj2d obj);
	unsigned SetTraceLen(unsigned new_len);
	unsigned GetTraceLen();
	Obj2d GetCurrent();
	std::deque<cv::Point2f> GetTrace();
	line_segment GetLastMove();
protected:
	Obj2d _obj;
	cv::Point2f _prev_center;
	unsigned _trace_len;
	std::deque<cv::Point2f> _trace;
};

cv::Point2f WindowToLogic(cv::Point2f p, cv::Size size);
cv::Point2f LogicToWindow(cv::Point2f p, cv::Size size);
bool GetEq(line_segment ls, cv::Point2f& point);
float GetAngleDiff(line_segment line1, line_segment line2);
bool Intersect(line_segment line1, line_segment line2, cv::Point2f* point = nullptr);
float IntersectObject(Obj2d& obj, line_segment ls, std::vector<std::pair<int, cv::Point2f>>& points);

void DrawTrace(cv::Mat img, std::deque<cv::Point2f> trace, cv::Scalar color);
void DrawFrame(cv::Mat img, moving_object& mov_obj, line_segment ls);
void DrawSegments(cv::Mat img, std::vector<std::pair<int, cv::Point2f>> points);
bool TopDownPred(std::pair<int, cv::Point2f>& p1, std::pair<int, cv::Point2f>& p2);