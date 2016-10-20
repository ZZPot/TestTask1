#include "MovingObject.h"
#include <opencv2/imgproc.hpp>
#include <stdio.h>
using namespace cv;

#define WND_NAME_ORIG	"Original"
#define WND_NAME_MODEL	"Model"
#define FRAMES_DIR		"MovingObject1/%04d.bmp"
#define FONT_SIZE		15
#define INFO_LEN		64

#pragma warning (disable: 4996)
int main()
{
	VideoCapture files;
	if(!files.open(FRAMES_DIR))
	{
		return 1;
	}
	Point2f p1(0, 0), p2(0.5, 1);
	Mat frame;
	unsigned frame_count = 0;
	while(files.read(frame))
	{
		imshow(WND_NAME_ORIG, frame);
		cvtColor(frame, frame, CV_BGR2GRAY);
		
		Mat frame_bw;
		threshold(frame, frame_bw, 0, 255, THRESH_OTSU);
		if(countNonZero(frame_bw) > ((frame_bw.cols * frame_bw.rows) / 2))
			frame_bw = 1 - frame_bw;
		std::vector<Obj2d> objects = FindObjects(frame_bw, 
						std::vector<type_condition>(), std::vector<int>(), RETR_EXTERNAL);
		if(!objects.size())
			continue;
		static line_segment that_line = {LogicToWindow(p1, frame.size()), LogicToWindow(p2, frame.size())};
		static moving_object mov_obj = objects[0];
		mov_obj.SetTraceLen(5);
		mov_obj.NewPos(objects[0]);
		Mat model_frame = Mat::zeros(frame.size(), CV_8UC3);
		DrawFrame(model_frame, mov_obj, that_line);
		Point2f poc;
		line_segment last_move = mov_obj.GetLastMove();
		bool intersect_center = Intersect(last_move, that_line, &poc);
		std::vector<std::pair<int, cv::Point2f>> points;
		float intersection_len = IntersectObject(objects[0], that_line, points);

		unsigned info_count = 1;
		char info_str[INFO_LEN] = "";
		sprintf(info_str, "Frame: %d", frame_count); // unsecure, VS will generate warning
		putText(model_frame, info_str, Point(0, FONT_SIZE*info_count++), 
				FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1, LINE_AA);
		if(points.size())
		{
			DrawSegments(model_frame, points);
			sprintf(info_str, "Length: %.1f", intersection_len);
			putText(model_frame, info_str, Point(0, FONT_SIZE*info_count++), 
				FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1, LINE_AA);
		}
		if(intersect_center)
		{
			float angle = GetAngleDiff(last_move, that_line);
			drawMarker(model_frame, poc, basic_colors[COLOR_MARK], MARK_POINT, 6, 2);
			poc = WindowToLogic(poc,model_frame.size());
			sprintf(info_str, "Crossed at: %.2f:%.2f", poc.x, poc.y);
			putText(model_frame, info_str, Point(0, FONT_SIZE*info_count++), 
				FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1, LINE_AA);
			sprintf(info_str, "Angle: %.1fdeg", angle);
			putText(model_frame, info_str, Point(0, FONT_SIZE*info_count++), 
				FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1, LINE_AA);
		}		
		imshow(WND_NAME_MODEL, model_frame);
		waitKey(0);
		frame_count++;
	}
	return 0;
}