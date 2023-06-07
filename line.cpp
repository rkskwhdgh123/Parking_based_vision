

#include"line.hpp"

VideoCapture camera_open(){
    string src = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), \
    width=(int)640, height=(int)360, framerate=15/1, format=(string)NV12 ! \
    nvvidconv flip-method=0 ! video/x-raw, width=(int)320, height=(int)180, \
    format=(string)BGRx ! videoconvert ! \
    video/x-raw, format=(string)BGR !appsink";
    
    VideoCapture source(src, CAP_GSTREAMER); //Gstreamer 명령어로 카메라를 열어줌


    return source;
}

Mat get_roi(Mat frame){
    Mat gray_Area;
    Mat Area=frame(Rect(0,3*frame.rows/4,frame.cols,frame.rows/4));
    cvtColor(Area, gray_Area, COLOR_BGR2GRAY);
    return gray_Area;
}

Mat light_control(Mat gray_Area){
    Scalar avr=mean(gray_Area);

    Mat light_roi;
    light_roi=gray_Area+(125-avr[0]);

    return light_roi;
}

Mat get_binary(Mat light_roi){
    int threshold=200;
    Mat binary;
    light_roi.copyTo(binary);
    for(int j=0; j<light_roi.rows; j++){
        for(int i=0; i<light_roi.cols; i++){
            if(light_roi.at<uchar>(j,i)>threshold)
                binary.at<uchar>(j,i)=255;
            else
                binary.at<uchar>(j,i)=0;
        }
    }
    return binary;
}

double detect_line(Mat frame, Mat binary,double *old_x,double *old_y){
    Mat img_labels, stats, centroids;

    int numOfLables = connectedComponentsWithStats(binary, img_labels, stats, centroids, 8, CV_32S);
    int big_num=1;
    int big_value=0;
    for (int j = 1; j < numOfLables; j++) {
    int area = stats.at<int>(j, CC_STAT_AREA);
    if((area>big_value)&&(area>300)&&(area<10000)){big_value=area; big_num=j;}
    }

    int left = stats.at<int>(big_num, CC_STAT_LEFT);
    int top = stats.at<int>(big_num, CC_STAT_TOP);
    int width = stats.at<int>(big_num, CC_STAT_WIDTH);
    int height = stats.at<int>(big_num, CC_STAT_HEIGHT);
    if((stats.at<int>(big_num, CC_STAT_AREA)>300)&&(stats.at<int>(big_num, CC_STAT_AREA)<10000)){
    rectangle( frame, Point(left, 3*frame.rows/4+top), Point(left + width, 3*frame.rows/4+top+ height),Scalar(0, 0, 255), 2);
    rectangle( binary, Point(left, top), Point(left + width, top+ height),Scalar(125), 2);
    }
    double x = centroids.at<double>(big_num, 0);
    double y = centroids.at<double>(big_num, 1);

    if(numOfLables!=1){
        *old_x=centroids.at<double>(big_num, 0);
        *old_y=centroids.at<double>(big_num, 1);}
    else if(numOfLables==1){x=*old_x; y=*old_y;}
    
    circle(frame, Point(x, 3*frame.rows/4+y), 5, Scalar(255,0,0), 2, 8);
    circle(binary, Point(x, y), 5, Scalar(125), 1, 8);

    return x;

}

double get_error(double num1, double num2){
    return num1-num2;
}

char follow_line(Dxl dxl,double error, bool findornot,bool *lineorpark, bool *detect_pront,double front_error,int front_mid_y,float *detect_threshold, bool *robot_st,bool right_direction,bool hand_find){
   
    int rpwm,lpwm;
    double gain;
    char key=' ';
    if(dxl.kbhit() ){
        key=dxl.getch();
    }

    //모터 속도 제어
    int basic_speed=50;
    gain=0.1+0.002*abs(error/2);
    rpwm = basic_speed + gain*(error/2);
    lpwm = basic_speed - gain*(error/2);


    if(*detect_pront){
        int parking_speed=35;
        gain=0.005+0.001*abs(front_error/4);
        // gain=0;
        rpwm = parking_speed + gain*(front_error/10);
        lpwm = parking_speed - gain*(front_error/10);
        cout<<front_mid_y<<endl;
        if(front_mid_y>250){*detect_threshold=0.2;}
        // if(front_mid_y>310){lpwm=0; rpwm=0;}
        if(front_mid_y>330)key='s';
    }
    else{
        if(findornot){
            *lineorpark=false;
            if(hand_find){
                rpwm=0; lpwm=0;
            }else{
                if(right_direction==true){rpwm=-10; lpwm=10;}
                else{rpwm=10; lpwm=-10;}
            }
        }
    }
    
    if(key=='q'){lpwm=0; rpwm=0;}
    
    if(*robot_st==true){
       lpwm=0; rpwm=0;  
    }
    dxl.dxl_set_velocity(lpwm, -rpwm);
    usleep(10000);
    return key;
 
}


Mat get_park_roi(Mat frame){
    Mat gray_Area;
    Mat Area=frame(Rect(0,frame.rows/2,frame.cols,frame.rows/2));
    cvtColor(Area, gray_Area, COLOR_BGR2GRAY);
    return gray_Area;
}

Mat park_area_light_control(Mat gray_Area){
    Scalar avr=mean(gray_Area);

    Mat light_roi;
    light_roi=gray_Area+(128-avr[0]);

    return light_roi;
}

Mat get_park_binary(Mat light_roi){
    Mat binary;
    threshold(light_roi, binary, 0, 255, THRESH_BINARY_INV|THRESH_OTSU);
    return binary;
}

Mat detect_line(Mat binary,Mat light_roi, vector<Point> &pts){
    Mat img_labels, stats, centroids;
    Point2f pt1,pt2,pt3,pt4;
    int area;
    int numOfLables = connectedComponentsWithStats(binary, img_labels, stats, centroids, 8, CV_32S);
    int big_num=1;
    int big_value=0;

    for (int j = 1; j < numOfLables; j++) {
    int width = stats.at<int>(j, CC_STAT_WIDTH);
    int height = stats.at<int>(j, CC_STAT_HEIGHT);
    area = stats.at<int>(j, CC_STAT_AREA);
    if((area>big_value)&&(area>3000)&&(area<70000)&&(width!=binary.cols)&&(width>2*height)){big_value=area; big_num=j; }
    }

    int left = stats.at<int>(big_num, CC_STAT_LEFT);
    int top = stats.at<int>(big_num, CC_STAT_TOP);
    int width = stats.at<int>(big_num, CC_STAT_WIDTH);
    int height = stats.at<int>(big_num, CC_STAT_HEIGHT);
    if((stats.at<int>(big_num, CC_STAT_AREA)>4000)&&(stats.at<int>(big_num, CC_STAT_AREA)<70000)){
    // rectangle( binary, Point(left, 2*binary.rows/3+top), Point(left + width, 2*binary.rows/3+top+ height),Scalar(125), 2);
    pt1=Point(left+40, top-10);
    pt2=Point(left+ width-40, top-10);
    pt3=Point(left + width+25, top+ height+25);
    pt4=Point(left-25, top+ height+25);

    pts[0]=pt1;
    pts[1]=pt2;
    pts[2]=pt3;
    pts[3]=pt4;
    

    rectangle( binary, Point(left, top-5), Point(left + width, top+ height),Scalar(125), 2);
    }
    double x = centroids.at<double>(big_num, 0);
    double y = centroids.at<double>(big_num, 1);

    // circle(frame, Point(x, 2*frame.rows/3+y), 5, Scalar(255,0,0), 2, 8);
    circle(binary, Point(x, y), 5, Scalar(0,0,255), 1, 8);

    return binary;
}



float angle_calculation(int ax,int ay,int bx,int by,int cx,int cy){
    float d1 = sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
    float d2 = sqrt((bx - cx) * (bx - cx) + (by - cy) * (by - cy));
    float d3 = sqrt((ax - cx) * (ax - cx) + (ay - cy) * (ay - cy));
    float cos_theta = (d1 * d1 + d2 * d2 - d3 * d3) / (2 * d1 * d2);
    float angle_in_degrees = acos(cos_theta) * 180 / M_PI;

    return angle_in_degrees;
}

float get_park_distance(int ax,int ay,int bx,int by){
    float distance;
    distance=sqrt(pow(ax-bx,2)+pow(ay-by,2));
    return distance;
}

Mat draw_point(Mat frame2,int *distance,int *max_distance,int *ignore_x,int *x_sum,int *y_sum, vector<Point2f> corners,  Point2f &mid){

    Mat dst;
    frame2.copyTo(dst);
    for(unsigned int i=0; i<corners.size(); i++ ){
        float X = corners[i].x; 
        float Y = corners[i].y+frame2.rows/2;

        *distance=Y;
        if(max_distance<distance){max_distance=distance;
        *ignore_x=corners[i].x;
        }
        *x_sum+=X;
        *y_sum+=Y;
        circle(dst,Point(X,Y),5,Scalar(0,0,255),2);
    }

    if(corners.size()==3){mid.x =(*x_sum-*ignore_x)/(corners.size()-1);}
    else mid.x=*x_sum/corners.size();
    mid.y=*y_sum/corners.size();
    if(corners.size()!=1)
    circle(dst,mid,5,Scalar(0,255,0),2);
    return dst;
}

vector<Point2f> sortPoint(Point2f &p1,Point2f &p2,Point2f &p3,Point2f &p4){
    Point2f center(0,0);
    center.x=(p1.x+p2.x+p3.x+p4.x)/4;
    center.y=(p1.x+p2.x+p3.x+p4.x)/4;
    vector<pair<double,Point2f>> angles;
    angles.reserve(4);
    angles.emplace_back(atan2(p1.y-center.y,p1.x-center.x),p1);
    angles.emplace_back(atan2(p2.y-center.y,p2.x-center.x),p2);
    angles.emplace_back(atan2(p3.y-center.y,p3.x-center.x),p3);
    angles.emplace_back(atan2(p4.y-center.y,p4.x-center.x),p4);
    sort(angles.begin(),angles.end(),[](const auto& a,const auto& b){return a.first < b.first;});
    vector<Point2f> sortedpoints;
    sortedpoints.reserve(4);
    for(const auto& anglePoint: angles){
        sortedpoints.push_back(anglePoint.second);
    }
    return sortedpoints;
    
}

vector<Point2f> caculation_save(vector<Point2f> corners,Mat frame,Point2f mid,int *mid_up_x,int *mid_up_y,vector<float> &point_x,Point2f points[], vector<float>&deg,vector<float>&park_dist,Mat dst,bool findornot){
    vector<Point2f> sortedPoint;
    if(corners.size()==4){
        int ax,ay,bx,by;
        for(unsigned int i=0; i<corners.size(); i++){
            ax=corners[i].x;
            ay=corners[i].y+frame.rows/2;
            bx=mid.x;
            by=mid.y;
            if(abs(by-ay)>20)*mid_up_y+=1;
            if(abs(bx-ax)>40)*mid_up_x+=1;
            point_x.push_back(ax);
            points[i].x=ax;
            points[i].y=ay;
        }
        sort(point_x.begin(), point_x.end());
        sortedPoint=sortPoint(points[0],points[1],points[2],points[3]);
    
        for(int i=0; i<4; i++){
            float degree;
            if(i!=3)
                degree=angle_calculation(points[i].x,points[i].y,bx,by,points[i+1].x,points[i+1].y);
            else
                degree=angle_calculation(sortedPoint[i].x,sortedPoint[i].y,bx,by,sortedPoint[0].x,sortedPoint[0].y);
            float park_distance=get_park_distance(sortedPoint[i].x,sortedPoint[i].y,bx,by);
            deg.push_back(degree); //각도를 벡터에 저장
            park_dist.push_back(park_distance); 
        }
        for(int i=0; i<4; i++){
        if(findornot){
            if(i==3)
                line(dst, Point(sortedPoint[i]), Point(sortedPoint[0]), Scalar(255,0,0), 1, 8, 0);
            else
                line(dst, Point(sortedPoint[i]), Point(sortedPoint[i+1]), Scalar(255,0,0), 1, 8, 0);
        }else{
            if(i==3)
                line(dst, Point(sortedPoint[i]), Point(sortedPoint[0]), Scalar(0,0,255), 1, 8, 0);
            else
                line(dst, Point(sortedPoint[i]), Point(sortedPoint[i+1]), Scalar(0,0,255), 1, 8, 0);
        }
        }
    //    float area=get_retangle_Area(sortedPoint,mid);
    //    cout<<"area"<<area<<endl;
    }
    return sortedPoint;
}

void Parking_space_judgment(vector<Point2f> corners,Mat dst,Point2f mid,int mid_up_y,int mid_up_x,vector<float>deg,vector<float>park_dist,vector<float>point_x,bool *findornot,vector<Point2f> sortedPoint ){
    //float area=get_retangle_Area(sortedPoint,mid);
    Point2f sorted_mid;
    vector<float> sorted_deg;
    vector<int> sorte_x;
    float sorted_degree;
    float w1,w2,w3,diagonal_1,diagonal_2;
    int h1,h2;
    if(corners.size()==4){
        //각도 계산
        sorted_mid.x=(sortedPoint[0].x+sortedPoint[1].x+sortedPoint[2].x+sortedPoint[3].x)/4; //30이상
        sorted_mid.y=(sortedPoint[0].y+sortedPoint[1].y+sortedPoint[2].y+sortedPoint[3].y)/4; //30이상
        for(int i=0; i<4; i++){
            if(i!=3)
            {
                sorted_degree=angle_calculation(sortedPoint[i].x,sortedPoint[i].y,sorted_mid.x,sorted_mid.y,sortedPoint[i+1].x,sortedPoint[i+1].y);
            }else{
                sorted_degree=angle_calculation(sortedPoint[i].x,sortedPoint[i].y,sorted_mid.x,sorted_mid.y,sortedPoint[i+1].x,sortedPoint[i+1].y);
            }
                sorted_deg.push_back(sorted_degree);
                sorte_x.push_back(sortedPoint[i].x);
        }
        sort(sorte_x.begin(),sorte_x.end());
        w1=sorte_x[1]-sorte_x[0]; //85
        w2=sorte_x[2]-sorte_x[1]; //195
        w3=sorte_x[3]-sorte_x[2]; //91

        h1=abs(sortedPoint[0].y-sortedPoint[2].y); //79
        h2=abs(sortedPoint[1].y-sortedPoint[3].y); //87

        diagonal_1=get_park_distance(sortedPoint[0].x,sortedPoint[0].y,sortedPoint[2].x,sortedPoint[2].y); //297.8
        diagonal_2=get_park_distance(sortedPoint[1].x,sortedPoint[1].y,sortedPoint[3].x,sortedPoint[3].y); //299.5
        
        // cout<<"sorted_deg[0]"<<sorted_deg[0]<<endl;   
        // cout<<"sorted_deg[1]"<<sorted_deg[1]<<endl;   
        // cout<<"sorted_deg[2]"<<sorted_deg[2]<<endl;   
        // cout<<"sorted_deg[3]"<<sorted_deg[3]<<endl;   
        // cout<<"w1"<<w1<<endl;
        // cout<<"w2"<<w2<<endl;
        // cout<<"w3"<<w3<<endl;
        // cout<<"h1"<<h1<<endl;
        // cout<<"h2"<<h2<<endl;
        // cout<<"diagonal_1"<<diagonal_1<<endl;
        // cout<<"diagonal_2"<<diagonal_2<<endl;
        
        if(abs(dst.cols/2-mid.x)<100)
            if((sorted_deg[0]>25)&&(sorted_deg[1]>25)&&(sorted_deg[2]>25)&&(sorted_deg[3]>25))
                if((w1>60)&&(w2>155)&&(w3>60))
                    if((h1>50)&&(h1>50)&&(abs(h1-h2)<35))
                        if((diagonal_1>230)&&(diagonal_2>230)&&(fabs(diagonal_1-diagonal_2)<30))
                            *findornot=true;

    }                            
}


void front_Parking_space_judgment(vector<Point2f> corners,Mat dst,Point2f mid,int mid_up_y,int mid_up_x,vector<float>deg,vector<float>park_dist,vector<float>point_x,bool *findornot,vector<Point2f> sortedPoint )
{
    Point2f sorted_mid;
    vector<float> sorted_deg;
    vector<int> sorte_x;
    float sorted_degree;
    float w1,w2,w3,diagonal_1,diagonal_2;
    int h1,h2;
    if(corners.size()==4){
        //각도 계산
        sorted_mid.x=(sortedPoint[0].x+sortedPoint[1].x+sortedPoint[2].x+sortedPoint[3].x)/4; //30이상
        sorted_mid.y=(sortedPoint[0].y+sortedPoint[1].y+sortedPoint[2].y+sortedPoint[3].y)/4; //30이상
        for(int i=0; i<4; i++){
            if(i!=3)
            {
                sorted_degree=angle_calculation(sortedPoint[i].x,sortedPoint[i].y,sorted_mid.x,sorted_mid.y,sortedPoint[i+1].x,sortedPoint[i+1].y);
            }else{
                sorted_degree=angle_calculation(sortedPoint[i].x,sortedPoint[i].y,sorted_mid.x,sorted_mid.y,sortedPoint[i+1].x,sortedPoint[i+1].y);
            }
                sorted_deg.push_back(sorted_degree);
                sorte_x.push_back(sortedPoint[i].x);
        }
        sort(sorte_x.begin(),sorte_x.end());
        w1=sorte_x[1]-sorte_x[0]; //85
        w2=sorte_x[2]-sorte_x[1]; //195
        w3=sorte_x[3]-sorte_x[2]; //91

        h1=abs(sortedPoint[0].y-sortedPoint[2].y); //79
        h2=abs(sortedPoint[1].y-sortedPoint[3].y); //87

        diagonal_1=get_park_distance(sortedPoint[0].x,sortedPoint[0].y,sortedPoint[2].x,sortedPoint[2].y); //297.8
        diagonal_2=get_park_distance(sortedPoint[1].x,sortedPoint[1].y,sortedPoint[3].x,sortedPoint[3].y); //299.5
        // cout<<"abs(dst.cols/2-mid.x)"<<abs(dst.cols/2-mid.x)<<endl;
        // cout<<"sorted_deg[0] "<<sorted_deg[0]<<endl;   
        // cout<<"sorted_deg[1] "<<sorted_deg[1]<<endl;   
        // cout<<"sorted_deg[2] "<<sorted_deg[2]<<endl;   
        // cout<<"sorted_deg[3] "<<sorted_deg[3]<<endl;   
        // cout<<"w1 "<<w1<<endl;
        // cout<<"w2 "<<w2<<endl;
        // cout<<"w3 "<<w3<<endl;
        // cout<<"h1 "<<h1<<endl;
        // cout<<"h2 "<<h2<<endl;
        // cout<<"diagonal_1  "<<diagonal_1<<endl;
        // cout<<"diagonal_2  "<<diagonal_2<<endl;
        
        if(abs(dst.cols/2-mid.x)<100)
            if((sorted_deg[0]>25)&&(sorted_deg[1]>25)&&(sorted_deg[2]>25)&&(sorted_deg[3]>25))
                if((w1>35)&&(w2>155)&&(w3>35))
                    if((h1>50)&&(h2>50)&&(abs(h1-h2)<30))
                        if((diagonal_1>210)&&(diagonal_2>210)&&(fabs(diagonal_1-diagonal_2)<70))
                            *findornot=true;

    }     
}

float get_retangle_Area(vector<Point2f> points,Point2f mid){
    float area;
    int mid_pass_x1=(points[0].x+points[2].x)/2;
    int mid_pass_y1=(points[0].y+points[2].y)/2;
    float cross_mid1=get_park_distance(mid_pass_x1,mid_pass_y1,mid.x,mid.y);

    int mid_pass_x2=(points[1].x+points[3].x)/2;
    int mid_pass_y2=(points[1].y+points[3].y)/2;
    float cross_mid2=get_park_distance(mid_pass_x2,mid_pass_y2,mid.x,mid.y);

    if((cross_mid2<50)&&(cross_mid1<50)){

    float dist_diagonal=get_park_distance(points[0].x,points[0].y,points[2].x,points[2].y);
    float dist_diagonal_height1=get_park_distance(mid.x,mid.y,points[1].x,points[1].y);
    float dist_diagonal_height2=get_park_distance(mid.x,mid.y,points[3].x,points[3].y);
    
     area=((dist_diagonal* dist_diagonal_height1)/2)+((dist_diagonal* dist_diagonal_height2)/2);
    }
    else  area=0;
    cout<<"area"<<area<<endl;
    return area;
}

//아래 손 인식 관련 함수

Mat detect_hand( Mat frame,Mat binary,Point2f *pt1,Point2f *pt2){
    Mat img_labels, stats, centroids;

    int numOfLables = connectedComponentsWithStats(binary, img_labels, stats, centroids, 8, CV_32S);
    int big_num=1;
    int big_value=0;
    for (int j = 1; j < numOfLables; j++) {
    int area = stats.at<int>(j, CC_STAT_AREA);
    if((area>big_value)&&(area>3000)&&(area<30000)){big_value=area; big_num=j;}
    }

    int left = stats.at<int>(big_num, CC_STAT_LEFT);
    int top = stats.at<int>(big_num, CC_STAT_TOP);
    int width = stats.at<int>(big_num, CC_STAT_WIDTH);
    int height = stats.at<int>(big_num, CC_STAT_HEIGHT);
    rectangle( frame, Point(left, top), Point(left + width, top+ height),Scalar(0, 255, 0), 2);
    rectangle( binary, Point(left, top), Point(left + width, top+ height),Scalar(125), 2);
    *pt1=Point(left, top);
    *pt2=Point(left + width, top+ height);

    
    // circle(frame, Point(x, 3*frame.rows/4+y), 5, Scalar(255,0,0), 2, 8);
    // circle(binary, Point(x, y), 5, Scalar(125), 1, 8);

    return frame;

}
Mat light_control_hand(Mat gray_Area){
    Scalar avr=mean(gray_Area);

    Mat light_roi;
    light_roi=gray_Area+(125-avr[0]);

    return light_roi;
}
Mat get_binary_hand(Mat light_roi){
    int threshold=180;
    Mat binary;
    light_roi.copyTo(binary);
    for(int j=0; j<light_roi.rows; j++){
        for(int i=0; i<light_roi.cols; i++){
            if(light_roi.at<uchar>(j,i)>threshold)
                binary.at<uchar>(j,i)=255;
            else
                binary.at<uchar>(j,i)=0;
        }
    }
    return binary;
}

Mat get_hand_roi(Net net,Mat frame,Point2f pt1,Point2f pt2,bool *hand_find,bool *right_direction, int *right_count,int *left_count){
    vector<String> classNames = { "Right","Left" };
    Mat ycrcb;
    cvtColor(frame,ycrcb,COLOR_BGR2YCrCb);
    Mat skin_mask;
    inRange(ycrcb,Scalar(40,133,77),Scalar(150,173,127),skin_mask);
    vector<vector<Point>> contours;
    findContours(skin_mask,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    double max_cA=0;
    int max_cn=-1;
    for(unsigned int i=0; i<contours.size(); ++i){
        double area=contourArea(contours[i]);
        if(max_cA<area){
            max_cA=area;
            max_cn=i;
        }
    }
    Mat contourimage=Mat::zeros(frame.size(),CV_8UC1);
    if(max_cn>=0){
    drawContours(contourimage,contours, max_cn,Scalar(255),FILLED);
    }
    Mat dst2;
    frame.copyTo(dst2,contourimage);
    Mat dst2_gray;
    cvtColor(dst2,dst2_gray,COLOR_BGR2GRAY);

    Mat light_roi=light_control_hand(dst2_gray); //이미지 밝기 조절 
    Mat binary=get_binary_hand(light_roi); //이진화
    Mat hand_detect=detect_hand(frame,binary,&pt1,&pt2); //라인 찾고 박스 그리고 중심점 반환
    Mat img_roi_hand = frame(Rect(pt1,pt2));  // 손영역 추출

    Mat inputBlob = blobFromImage(img_roi_hand, 1.0 / 127.5, Size(224, 224), Scalar(-1, -1, -1));
    net.setInput(inputBlob);
    Mat prob = net.forward();
    // Check results & Display
    double maxVal;
    Point maxLoc;
    minMaxLoc(prob, NULL, &maxVal, NULL, &maxLoc);
    String str = classNames[maxLoc.x] + format("(%4.2lf%%)", maxVal *100);
    putText(img_roi_hand, str, Point(10, 30), 3,1.3, Scalar(0,0,255));

    if(maxVal *100>=70){
        if(maxLoc.x==0)*right_count+=1;
        else if(maxLoc.x==1)*left_count+=1;

        if(*right_count>=50){
            *hand_find=false;
        }else if(*left_count>=50){
            *hand_find=false;
            *right_direction=false;
        }
        cout<<"우회전 스택" <<*right_count<<endl;
        cout<<"좌회전 스택" <<*left_count<<endl;
    } 
    return frame;
}
