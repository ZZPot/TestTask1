#include "MovingPoints.h"
#include <opencv2/optflow.hpp>

moving_points::moving_points(cv::Mat init_frame):
_qlevel(0.01), _max_count(50), _min_dist(10)
{
	cv::cvtColor(init_frame, _frames[0], cv::COLOR_BGR2GRAY);
	AddPoints();
}
void moving_points::AddFrame(cv::Mat new_frame)
{
	cv::cvtColor(new_frame, _frames[1], cv::COLOR_BGR2GRAY);
	if(_frames[0].empty())
		_frames[0] = _frames[1].clone();
	if(_points[0].size() < MIN_POINTS_COUNT)
		AddPoints();
	cv::calcOpticalFlowPyrLK(_frames[0], _frames[1], _points[0], _points[1], _point_status, _point_error);
	int points_size =  _points[1].size();
	auto points_iter = _points[1].begin();
	for(int i = points_size-1; i >= 0; i--)
	{
		if(!AcceptPoint(i))
			_points[1].erase(points_iter + i);
	}
	_points[0] = _points[1];
	_frames[0] = _frames[1].clone();
}
bool moving_points::GetObj(Obj2d* obj)
{
	if(!_points[0].size())
		return false;
	std::vector<int> hull_id;
	cv::convexHull(_points[0], hull_id);
	obj->contours.resize(1); // first level
	obj->contours[0].resize(1); // one contour
	for(unsigned i = 0; i < hull_id.size(); i++)
		obj->contours[0][0].push_back(_points[0][hull_id[i]]);
	return true;
}
void moving_points::AddPoints()
{
	std::vector<cv::Point> new_points;
	cv::goodFeaturesToTrack(_frames[0],	new_points, _max_count,	_qlevel, _min_dist);
	_points[0].insert(_points[0].end(), new_points.begin(), new_points.end());
}
bool moving_points::AcceptPoint(unsigned id)
{
	float dist = sqrt(	pow((_points[0][id].x - _points[1][id].x), 2) + 
						pow((_points[0][id].y - _points[1][id].y), 2));
	return _point_status[id] && (dist > 1); // at least one pixel
}