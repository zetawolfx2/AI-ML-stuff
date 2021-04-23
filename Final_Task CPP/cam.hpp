#include "opencv2/opencv.hpp"
#include <iostream>
#include "global.hpp"

using namespace std;
using namespace cv;

string path_c = "/home/zetawolfx2/Desktop/Task_Video.mp4";
VideoCapture cap_c(path_c);

void display_cam(int n)
{

	Mat frame_c;
	cap_c >> frame_c;

	if(n==1)
	{
		gframe = frame_c.clone();
	}
}

