#include"line.hpp"
#include <signal.h>
int main()
{
 
    Dxl dxl;
    Point2f pt1,pt2,pt3,pt4;
    //일반
    //string src = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360, format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    //string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=203.234.62.58 port=8001 sync=false";

    //string src2 = "nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360, format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    //string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=203.234.62.58 port=8002 sync=false";
    
    // pc실
      string src = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360, format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
      string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=203.234.58.164 port=8001 sync=false";

      string src2 = "nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360, format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
      string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=203.234.58.164 port=8002 sync=false";

    VideoCapture cap(src, CAP_GSTREAMER);
    VideoCapture cap2(src2, CAP_GSTREAMER);
    
    VideoWriter writer(dst, 0, (double)15, cv::Size(640, 360), true);
    VideoWriter writer2(dst2, 0, (double)15, cv::Size(640, 360), true);

    dxl.dxl_open(); //dxl 오픈.
    double diff1;
    double old_x,old_y;
    float detect_threshold=0.01;
    bool findornot=false,lineorpark=true,detect_pront=false,robot_st=false;

    Mat frame;
    Mat frame2;
   while(1){

    struct timeval start,end;
    gettimeofday(&start,NULL); // 시간측정함수
    cap >> frame;
    cap2 >> frame2;

    char key;
    double error=0;
    int front_mid_x,front_mid_y;
    if(lineorpark){
        Mat roi_gray=get_roi(frame); // 그레이영상 받아오고
        Mat light_roi=light_control(roi_gray); //이미지 밝기 조절 
        Mat binary=get_binary(light_roi); //이진화
        double line_x=detect_line(frame,binary,&old_x,&old_y); //라인 찾고 박스 그리고 중심점 반환
        error=get_error(frame.cols/2, line_x); // 라인에서 벗어난정도 반환
        writer<<frame;
    }else{
        Mat front_park_roi=get_park_roi(frame); 
        Mat front_park_light_roi=park_area_light_control(front_park_roi); //이미지 밝기 조절 
        Mat front_park_binary=get_park_binary(front_park_light_roi);
        vector<Point2f> front_corners,front_sortedPoint;
        Mat front_mask = Mat::zeros(frame.size(),CV_8UC1);
        Mat front_Area=front_mask(Rect(0,frame.rows/2,frame.cols,frame.rows/2));
        vector<Point> front_pts(4);
        Mat front_Area2=detect_line(front_park_binary,front_Area,front_pts);
        fillPoly(front_Area, front_pts, cv::Scalar(255), LINE_AA);
        // imshow("front_Area",front_Area);
        Point2f front_mid,front_points[4];
        int front_x_sum=0,front_y_sum=0,front_ignore_x=0,front_mid_up_y=0,front_mid_up_x=0;
        int front_max_distance=0,front_distance=0;

        goodFeaturesToTrack(front_park_light_roi,front_corners,4,detect_threshold,50,front_Area); //코너 검출 4개제한,임계값0.05
        Mat front_dst=draw_point(frame,&front_distance,&front_max_distance,&front_ignore_x,&front_x_sum,&front_y_sum,front_corners,front_mid); //꼿짓점,중심점 그리기.
        vector<float>front_deg,front_park_dist,front_point_x;
        front_sortedPoint=caculation_save(front_corners,frame,front_mid,&front_mid_up_x,&front_mid_up_y,front_point_x,front_points,front_deg,front_park_dist,front_dst,findornot); //주차영역인지 판단하는데 필요한 값 계산후 저장
        front_Parking_space_judgment(front_corners,front_dst,front_mid,front_mid_up_y,front_mid_up_x,front_deg,front_park_dist,front_point_x,&detect_pront,front_sortedPoint); //주차영역이 검출되면 bool 타입 변수 true
        writer<<front_dst;
        front_mid_x=front_mid.x;
        front_mid_y=front_mid.y;
    }
    double front_error=get_error(frame.cols/2,front_mid_x );
    key=follow_line(dxl,error,findornot,&lineorpark,&detect_pront,front_error,front_mid_y,&detect_threshold,&robot_st);


    Mat park_roi=get_park_roi(frame2); 
    Mat park_light_roi=park_area_light_control(park_roi); //이미지 밝기 조절 
    Mat park_binary=get_park_binary(park_light_roi);
        
    vector<Point2f> corners,sortedPoint;
    Mat mask = Mat::zeros(frame2.size(),CV_8UC1);
    Mat Area=mask(Rect(0,frame2.rows/2,frame2.cols,frame2.rows/2));
    vector<Point> pts(4);
    Mat Area2=detect_line(park_binary,Area,pts);

 
    fillPoly(Area, pts, cv::Scalar(255), LINE_AA);
    // imshow("Area",Area);
    Point2f mid,points[4];
    int x_sum=0,y_sum=0,ignore_x=0,mid_up_y=0,mid_up_x=0;
    int max_distance=0,distance=0;

    goodFeaturesToTrack(park_light_roi,corners,4,detect_threshold,50,Area); //코너 검출 4개제한,임계값0.05
    Mat dst=draw_point(frame2,&distance,&max_distance,&ignore_x,&x_sum,&y_sum,corners,mid); //꼿짓점,중심점 그리기.


    vector<float>deg,park_dist,point_x;


    sortedPoint=caculation_save(corners,frame2,mid,&mid_up_x,&mid_up_y,point_x,points,deg,park_dist,dst,findornot); //주차영역인지 판단하는데 필요한 값 계산후 저장
    Parking_space_judgment(corners,dst,mid,mid_up_y,mid_up_x,deg,park_dist,point_x,&findornot,sortedPoint); //주차영역이 검출되면 bool 타입 변수 true

    if(findornot)cout<<"주차 공간 찾기 성공"<<endl;

    writer2<<dst;
 
    if (waitKey(2) == 27) key='q';
    if(key=='w')robot_st=false;
    if(key=='s'){robot_st=true;lineorpark=true; findornot=false; detect_pront=false; key=' '; cout<<"주차 완료"<<endl;}
    if(key=='q')break;
    
    gettimeofday(&end,NULL); // 시간측정함수
    diff1 = end.tv_sec + end.tv_usec / 1000000.0- start.tv_sec - start.tv_usec / 1000000.0;
    cout <<" 시간측정 :" << diff1 << endl;
   }
   	dxl.dxl_close();
	return 0;
}
