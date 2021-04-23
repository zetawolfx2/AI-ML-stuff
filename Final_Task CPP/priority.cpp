#include "opencv2/opencv.hpp"
#include <iostream>
#include "global.hpp"
#include "mov.hpp"
#include "cam.hpp"
#include <sstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

using namespace std;
using namespace cv;

Mat gframe;

int main()
{
	int priority = 1;
	int prev_priority = 1;

	int count = 0;
	int flag = 0;
	double eff = 0;

	int history = 45;
	float varThreshold = 200;
	bool bShadowDetection = true;

	string path = "/home/zetawolfx2/Desktop/Task_Video.mp4";
	VideoCapture cap(path);

	Ptr<BackgroundSubtractor> fgbg = createBackgroundSubtractorMOG2(history, varThreshold, bShadowDetection);

	while(1)
	{
		Mat frame;
		if(!(frame.dims>=2))
			cap >> frame;
		else
			break;

		int h = frame.size().height;
		int w = frame.size().width;

		Mat square = frame(Range((h/2)-70, (h/2)+120), Range((w/2)-155 ,(w/2)+155));
	
		Mat fgmask;
		fgbg->apply(square, fgmask);
	
		threshold(fgmask, fgmask, 30, 255, THRESH_BINARY);
    		Mat element = getStructuringElement(MORPH_RECT, Size(2, 2), Point(-1, -1));
		dilate(fgmask, fgmask, element, Point(-1, -1), 2);

		Mat grey;
		cvtColor(frame, grey, CV_BGR2GRAY);
		GaussianBlur(grey, grey, Size(3,3), 0, 0);
		Canny(grey, grey, 150, 250);

		Mat mask = Mat::zeros(grey.size().height, grey.size().width, CV_8U);
		Point poly_mask_points[1][4];
		poly_mask_points[0][0] = Point(50, h);
		poly_mask_points[0][1] = Point(1150, h);
		poly_mask_points[0][2] = Point(850, 480);
		poly_mask_points[0][3] = Point(450, 480);
		const Point* ppt[1] = { poly_mask_points[0] };
		int npt[] = { 4 };
		fillPoly(mask, ppt, npt, 1, Scalar( 255, 255, 255 ), LINE_8);
		Mat masked_img;
		bitwise_and(grey, mask, masked_img);

		Mat mask_left = Mat::zeros(grey.size().height, grey.size().width, CV_8U);
		Point poly_mask_left_points[1][4];
		poly_mask_left_points[0][0] = Point(50, h);
		poly_mask_left_points[0][1] = Point((w/2)-60, h);
		poly_mask_left_points[0][2] = Point((w/2)-60, 450);
		poly_mask_left_points[0][3] = Point(450, 450);
		const Point* ppt_l[1] = { poly_mask_left_points[0] };
		fillPoly(mask_left, ppt_l, npt, 1, Scalar( 255, 255, 255 ), LINE_8);
		Mat masked_img_left;
		bitwise_and(grey, mask_left, masked_img_left);

		Mat mask_right = Mat::zeros(grey.size().height, grey.size().width, CV_8U);
		Point poly_mask_right_points[1][4];
		poly_mask_right_points[0][0] = Point((w/2)+170, h);
		poly_mask_right_points[0][1] = Point(1150, h);
		poly_mask_right_points[0][2] = Point(850, 450);
		poly_mask_right_points[0][3] = Point((w/2)+170, 450);
		const Point* ppt_r[1] = { poly_mask_right_points[0] };
		fillPoly(mask_right, ppt_r, npt, 1, Scalar( 255, 255, 255 ), LINE_8);
		Mat masked_img_right;
		bitwise_and(grey, mask_right, masked_img_right);

		vector<Point> locations_left;
		findNonZero(masked_img_left, locations_left);
		Point l_pnt_l = locations_left[0];

		vector<Point> locations_right;
		findNonZero(masked_img_right, locations_right);
		Point l_pnt_r = locations_right[0];

		Point u_pnt_l = l_pnt_l;
		Point u_pnt_r = l_pnt_r;
		u_pnt_l.y -= 150;
		u_pnt_r.y -= 200;

		/*
		imshow("Mask", masked_img);
		imshow("LMask", masked_img_left);
		imshow("RMask", masked_img_right);
		imshow("Square", fgmask);
		imshow("Gray", grey);
		*/

		int h_ar = u_pnt_r.y - l_pnt_r.y;
		int w_ar = u_pnt_l.x - u_pnt_r.x;
		
		vector<Point> white;
		findNonZero(fgmask, white);
		double white_pixels = white.size();
		double ROI_Area = h_ar*w_ar;

		count = count + 1;

		if(count == 60)
		{
			eff = white_pixels/ROI_Area;
			eff = round( eff * 1000.0 ) / 1000.0;
			prev_priority = priority;
			if (eff<0.050)
				{
					priority = 1;
				}
			else if((eff>0.050)&&(eff<0.090))
			    	{
					priority = 2;
				}
			else if (eff>0.090)
			    	{
					priority = 3;
				}
			count = 0;
		}

		if(priority==1)
		{
			display_mov(1);
			display_cam(0);
		}
		else if ((priority == 2)&&(prev_priority == 1))
		{
			display_mov(1);
			display_cam(0);
			flag = 1;
		}
		else if ((priority == 2)&&(prev_priority == 2))
		{
			if (flag == 1)
			{
				display_cam(1);
				if (count == 59)
					flag = 0;
			}
			else if (flag == 0)
			{
				display_mov(1);
				display_cam(0);
				if (count == 59)
					flag = 1;
			}
		}
		else if(priority==3)
			display_cam(1);
		
		imshow("Stream", gframe);
		waitKey(15);
	}

	cap.release();

	destroyAllWindows();

	return 0;
		
}
