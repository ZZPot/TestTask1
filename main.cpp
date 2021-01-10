#include "MovingObject.h"
#include "MovingPoints.h"
#include "ObjectDetector/BGSDetector/BGSDetector.h"
#include "ObjectDetector/AltDetector/AltDetector.h"
#include "ObjectDetector/SimpleDetector/SimpleDetector.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <stdio.h>
#include <conio.h>

#define WND_NAME_ORIG	"Original"
#define WND_NAME_MODEL	"Model"
#define WND_NAME_BG		"Background"
#define FRAMES_DIR		"MovingObject1Fixed/%04d.png"
#define FONT_SIZE		15
#define INFO_LEN		64

//#define ALT_METHOD
//#define BG_SEG

#pragma warning (disable: 4996 4267)
int main()
{
	cv::VideoCapture files;
	if(!files.open(FRAMES_DIR))
	{
		return 1;
	}
	cv::Point2f p1(0, 0), p2(0.5, 1);
	cv::Mat frame;
	unsigned frame_count = 0;

	ObjectDetector *objDetector = nullptr;

#ifdef BG_SEG
	objDetector = new BGSDetector;
#elif defined ALT_METHOD
	objDetector = new AltDetector;
#else
	objDetector = new SimpleDetector;
#endif
	

	while(files.read(frame))
	{
		cv::imshow(WND_NAME_ORIG, frame);
		static line_segment that_line = {LogicToWindow(p1, frame.size()), LogicToWindow(p2, frame.size())};
		Obj2d obj;

		// Object detection
		if (!objDetector->AddFrame(frame))
		{
			continue;
		}
		std::vector<Obj2d> objects = objDetector->DetectObjects(frame);
		if (!objects.size())
			continue;
		obj = objects[0];

		// Tracing it's movements
		// Need to collapse it to some class
		static moving_object mov_obj = obj;
		mov_obj.SetTraceLen(5);
		mov_obj.NewPos(obj);
		cv::Mat model_frame = cv::Mat::zeros(frame.size(), CV_8UC3);
		DrawFrame(model_frame, mov_obj, that_line);
		cv::Point2f poc;
		line_segment last_move = mov_obj.GetLastMove();
		bool intersect_center = Intersect(last_move, that_line, &poc);
		std::vector<std::pair<int, cv::Point2f>> points;
		float intersection_len = IntersectObject(obj, that_line, points);

		unsigned info_count = 1;
		char info_str[INFO_LEN] = "";
		sprintf(info_str, "Frame: %d", frame_count); // unsecure, VS will generate warning
		putText(model_frame, info_str, cv::Point(0, FONT_SIZE*info_count++),
			cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, cv::LineTypes::LINE_8);
		if(points.size())
		{
			DrawSegments(model_frame, points);
			sprintf(info_str, "Length: %.1f", intersection_len);
			putText(model_frame, info_str, cv::Point(0, FONT_SIZE*info_count++),
				cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, cv::LineTypes::LINE_8);
		}
		//if(intersect_center)
		{
			float angle = GetAngleDiff(last_move, that_line);
			drawMarker(model_frame, poc, basic_colors[COLOR_MARK], MARK_POINT, 6, 2);
			poc = WindowToLogic(poc,model_frame.size());
			sprintf(info_str, "Crossed at: %.2f:%.2f", poc.x, poc.y);
			putText(model_frame, info_str, cv::Point(0, FONT_SIZE*info_count++),
				cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, cv::LineTypes::LINE_8);
			sprintf(info_str, "Angle: %.1fdeg", angle);
			putText(model_frame, info_str, cv::Point(0, FONT_SIZE*info_count++),
				cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, cv::LineTypes::LINE_8);
		}		
		imshow(WND_NAME_MODEL, model_frame);

		if(cv::waitKey(0) == 27)
			break;
		frame_count++;
	}
	if(objDetector != nullptr)
		delete objDetector;
	return 0;
}