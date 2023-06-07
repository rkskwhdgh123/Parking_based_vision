#ifndef _MYLIB_H_
#define _MYLIB_H_
#include<iostream>
#include "opencv2/opencv.hpp"
#include "dxl.hpp"
#include <fstream>
#include <cmath>
using namespace std;
using namespace cv;
using namespace cv::dnn;
VideoCapture camera_open();
Mat get_roi(Mat frame);
Mat light_control(Mat gray_Area);
Mat get_binary(Mat light_roi);
double detect_line(Mat frame, Mat binary,double *old_x,double *old_y);
double get_error(double num1, double num2);
char follow_line(Dxl dxl,double error,bool findornot,bool *lineorpark,bool *detect_pront,double front_error,int front_mid_y,float *detect_threshold, bool *robot_st,bool right_direction,bool hand_find);

Mat get_park_roi(Mat frame);
Mat park_area_light_control(Mat gray_Area);
Mat get_park_binary(Mat light_roi);
float angle_calculation(int ax,int ay,int bx,int by,int cx,int cy);
float get_park_distance(int ax,int ay,int bx,int by);
Mat detect_line(Mat binary,Mat light_roi,vector<Point> &pts);
Mat draw_point(Mat frame2,int *distance,int *max_distance,int *ignore_x,int *x_sum,int *y_sum, vector<Point2f> corners,  Point2f &mid);
vector<Point2f> sortPoint(Point2f &p1,Point2f &p2,Point2f &p3,Point2f &p4);
vector<Point2f> caculation_save(vector<Point2f> corners,Mat frame,Point2f mid,int *mid_up_x,int *mid_up_y,vector<float> &point_x,Point2f points[], vector<float>&deg,vector<float>&park_dist,Mat dst,bool findornot);
void Parking_space_judgment(vector<Point2f> corners,Mat dst,Point2f mid,int mid_up_y,int mid_up_x,vector<float>deg,vector<float>park_dist,vector<float>point_x,bool *findornot,vector<Point2f> sortedPoint);
void front_Parking_space_judgment(vector<Point2f> corners,Mat dst,Point2f mid,int mid_up_y,int mid_up_x,vector<float>deg,vector<float>park_dist,vector<float>point_x,bool *findornot,vector<Point2f> sortedPoint );
float get_retangle_Area(vector<Point2f> points,Point2f mid);

Mat detect_hand( Mat frame,Mat binary,Point2f *pt1,Point2f *pt2);
Mat light_control_hand(Mat gray_Area);
Mat get_binary_hand(Mat light_roi);
Mat get_hand_roi(Net net,Mat frame,Point2f pt1,Point2f pt2,bool *hand_find,bool *right_direction,int *right_count,int *left_count);

#endif
