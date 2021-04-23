#include "opencv2/opencv.hpp"
#include <iostream>
#include "global.hpp"

using namespace std;
using namespace cv;

string path_m = "/home/zetawolfx2/Desktop/YS.mp4";
VideoCapture cap_m(path_m);

void display_mov(int n)
{

	Mat frame_m;
	
	if(n==1)
	{
		cap_m >> frame_m;
		gframe = frame_m.clone();
	}
}

