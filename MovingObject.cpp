#include "MovingObject.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <opencv2/imgproc.hpp>

#pragma warning(disable: 4267)

#define IN_RANGE(val, min, max) ((min) <= (val)) &&  ((val) <= (max))
#define GET_DIST(p1, p2)	sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2))

moving_object::moving_object(Obj2d obj)
{
	_obj = obj;
	_prev_center = _obj.r_rect.center;
	_trace.push_front(_prev_center);
}
void moving_object::NewPos(Obj2d obj)
{
	_prev_center = _obj.r_rect.center;
	_obj = obj;
	_trace.push_front(_obj.r_rect.center);
	if(_trace.size() > _trace_len + 1)
		_trace.resize(_trace_len + 1);
}
unsigned moving_object::SetTraceLen(unsigned new_len)
{
	unsigned old = _trace_len;
	_trace_len = new_len;
	if(_trace.size() > new_len + 1)
		_trace.resize(new_len + 1);
	return old;
}
unsigned moving_object::GetTraceLen()
{
	return _trace_len;
}
Obj2d moving_object::GetCurrent()
{
	return _obj;
}
std::deque<cv::Point2f> moving_object::GetTrace()
{
	return _trace;
}
line_segment moving_object::GetLastMove()
{
	line_segment _last_move = {_prev_center, _obj.r_rect.center};
	return _last_move;
}

cv::Point2f WindowToLogic(cv::Point2f p, cv::Size size)
{
	cv::Point2f res;
	res.x = p.x / (size.width - 1);
	res.y = 1 - p.y / (size.height - 1);
	return res;
}
cv::Point2f LogicToWindow(cv::Point2f p, cv::Size size)
{
	cv::Point2f res;
	res.x = p.x * (size.width - 1);
	res.y = (1 - p.y) * (size.height - 1);
	return res;
}
bool GetEq(line_segment ls, cv::Point2f& point)
{
	float x_diff = ls.p1.x - ls.p2.x, 
			y_diff = ls.p1.y - ls.p2.y;
	float eps = std::numeric_limits<float>::epsilon();
	if(abs(x_diff) < eps)
		return true; // vertical, any of p1.x and p2.x is the equality of line
	if(abs(y_diff) < eps) 
	{
		point.x = 0;
		point.y = ls.p1.y;
		return false;
	}
	// common case of previous
	point.x = y_diff / x_diff;
	point.y = ls.p1.y - ls.p1.x * point.x;
	return false;
}
float GetAngleDiff(line_segment line1, line_segment line2)
{
	cv::Point2f p[2];
	bool vertical[2] = {GetEq(line1, p[0]), GetEq(line2, p[1])};
	float angle1 = vertical[0] ? 90 : 90 - 180 / M_PI * atan(p[0].x);
	float angle2 = vertical[1] ? 90 : 90 - 180 / M_PI * atan(p[1].x);
	return angle1 - angle2;
}
bool Intersect(line_segment line1, line_segment line2, cv::Point2f* point)
{
	float eps = std::numeric_limits<float>::epsilon();
	float	top[2] = {fminf(line1.p1.y, line1.p2.y), fminf(line2.p1.y, line2.p2.y)},
			bottom[2] = {fmaxf(line1.p1.y, line1.p2.y), fmaxf(line2.p1.y, line2.p2.y)},
			left[2] = {fminf(line1.p1.x, line1.p2.x), fminf(line2.p1.x, line2.p2.x)},
			right[2] = {fmaxf(line1.p1.x, line1.p2.x), fmaxf(line2.p1.x, line2.p2.x)};
	cv::Point2f p[2];
	bool vertical[2] = {GetEq(line1, p[0]), GetEq(line2, p[1])};

	if (vertical[0] && vertical[1]) // both vertical
	{
		if(abs(left[0] - left[1]) < eps) // same shift
		{
			bool intersection[2] = {IN_RANGE(top[0], top[1], bottom[1]), IN_RANGE(bottom[0], top[1], bottom[1])};
			if(intersection[0] || intersection[1])
			{
				if(point != nullptr)
				{
					point->x = line1.p1.x; // all x equal
					point->y = intersection[0] ? (top[0] + bottom[1])/2 : (top[1] + bottom[0])/2;
				}
				return true;
			}
		}
		return false;
	}
	if (abs(p[0].x - p[1].x) < eps) // same angle
	{
		if(abs(p[0].y - p[1].y) < eps) // same shift
		{
			bool intersection[2] = {IN_RANGE(left[0], left[1], right[1]), IN_RANGE(right[0], left[1], right[1])};
			if(intersection[0] || intersection[1])
			{
				if(point != nullptr)
				{
					point->x = intersection[0] ? (left[0] + right[1])/2 : (left[1] + right[0])/2;
					point->y = p[0].x * point->x + p[0].y;
				}
				return true;
			}
		}
		return false;
	}
	cv::Point2f poc;
	if (vertical[0] || vertical[1]) // one vertical
	{
		unsigned vert = vertical[0] ? 0 : 1;
		poc.x = left[vert];
		poc.y = p[!vert].x * poc.x + p[!vert].y;
	}
	else
	{
		// if angles not equal
		poc.x = (p[1].y - p[0].y)/(p[0].x - p[1].x);
		poc.y = p[0].x * poc.x + p[0].y;
	}
	// Point Of Cross should be in range
	bool res =	IN_RANGE(poc.x, left[0], right[0]) && IN_RANGE(poc.x, left[1], right[1]) &&
				IN_RANGE(poc.y, top[0], bottom[0]) && IN_RANGE(poc.y, top[1], bottom[1]);
	if(res && (point != nullptr))
		*point = poc;
	return res;
}
float IntersectObject(Obj2d& obj, line_segment ls, std::vector<std::pair<int, cv::Point2f>>& points)
{
	unsigned contour_size = obj.contours[0][0].size(); // only for first outer contour
	contour_type& contour = obj.contours[0][0];
	for(unsigned i = 0; i < contour_size; i++) 
	{
		line_segment contour_line = {contour[i%contour_size], contour[(i+1)%contour_size]}; // it will also get last-first segment
		cv::Point2f poc;
		if(Intersect(contour_line, ls, &poc))
			points.push_back(std::pair<int, cv::Point2f>(i, poc));
	}
	if(cv::pointPolygonTest(contour, ls.p1, false) >= 0)
		points.push_back(std::pair<int, cv::Point2f>(-1, ls.p1));
	if(cv::pointPolygonTest(contour, ls.p2, false) >= 0)
		points.push_back(std::pair<int, cv::Point2f>(-1, ls.p2));
	if(points.size() && (points.size() % 2))
		points.push_back(*points.rbegin());
	sort(points.begin(), points.end(), TopDownPred);
	float res = 0;
	for(unsigned i = 0; i < points.size(); i+=2) 
		res += GET_DIST(points[i].second, points[i+1].second);
	return res;
}
void DrawTrace(cv::Mat img, std::deque<cv::Point2f> trace, cv::Scalar color)
{
	for(unsigned i = 1; i < trace.size(); i++) 
	{
		cv::arrowedLine(img, trace[i], trace[i-1], color);
	}		
}
void DrawFrame(cv::Mat img, moving_object& mov_obj, line_segment ls)
{
	Obj2d obj = mov_obj.GetCurrent();
	std::vector<cv::Scalar> colors = {basic_colors[COLOR_OBJECT]};
	DrawContours(obj.contours, colors, img, cv::Point(), 1);
	DrawTrace(img, mov_obj.GetTrace(), basic_colors[COLOR_TRACE]);
	line_segment last_move = mov_obj.GetLastMove();
	cv::arrowedLine(img, last_move.p1, last_move.p2, basic_colors[COLOR_MOVE], 1);
	cv::line(img, ls.p1, ls.p2, basic_colors[COLOR_LINE], 1);
}
void DrawSegments(cv::Mat img, std::vector<std::pair<int, cv::Point2f>> points)
{
	for(unsigned i = 0; i < points.size(); i+=2)
	{
		cv::line(img, points[i].second, points[i+1].second, basic_colors[COLOR_CUT], 2);
		cv::drawMarker(img, points[i].second, basic_colors[COLOR_MARK], MARK_POINT, 6, 2);
		cv::drawMarker(img, points[i+1].second, basic_colors[COLOR_MARK], MARK_POINT, 6, 2);
	}
}
bool TopDownPred(std::pair<int, cv::Point2f>& p1, std::pair<int, cv::Point2f>& p2)
{
	return p1.second.y < p2.second.y;
}
std::vector<cv::Scalar> basic_colors = {cv::Scalar(0, 0, 255),
										cv::Scalar(200, 200, 200),
										cv::Scalar(255, 0, 0),
										cv::Scalar(255, 255, 255),
										cv::Scalar(0, 220, 0),
										cv::Scalar(0, 220, 220)};