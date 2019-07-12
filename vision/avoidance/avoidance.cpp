#include "avoidance.h"
#define DEG2RAD  M_PI/180
Vision::Vision()
{
    df_1=0;
    df_2=0;
    far_good_angle=0;
    dd_1=0;
    dd_2=0;
    good_angle=0;
    final_angle=0;
}
Vision::~Vision()
{
    interface_map.release();
    destroyAllWindows();
}
double Vision::Rate()
{
    double ALPHA = 0.5;
    double dt;
    static int frame_counter = 0;
    static double frame_rate = 0.0;
    static double StartTime = ros::Time::now().toNSec();
    double EndTime;

    frame_counter++;
    if (frame_counter == 10)
    {
        EndTime = ros::Time::now().toNSec();
        dt = (EndTime - StartTime) / frame_counter;
        StartTime = EndTime;
        if (dt != 0)
        {
            frame_rate = (1000000000.0 / dt) * ALPHA + frame_rate * (1.0 - ALPHA);
            //cout << "FPS: " << frame_rate << endl;
        }

        frame_counter = 0;
    }
    return frame_rate;
}
void draw_ellipse(Mat &iframe,double angle_min, double angle_max, double radius, Scalar color, double width){
    int x;
    int y;
    int center_x = iframe.cols/2;
    int center_y = iframe.rows/2;
    x = center_x+radius*cos(angle_max*DEG2RAD);
    y = center_y-radius*sin(angle_max*DEG2RAD);
    line(iframe, Point(center_x, center_y), Point(x, y), color, width);
    x = center_x+radius*cos(angle_min*DEG2RAD);
    y = center_y-radius*sin(angle_min*DEG2RAD);
    line(iframe, Point(center_x, center_y), Point(x, y), color, width);
    ellipse(iframe, Point(center_x, center_y), Size(radius, radius), 0, 360 - angle_max, 360 - angle_min, color, width);
}
cv::Mat Vision::draw_interface()
{
    Rate();
    Mat visual_map(500, 500, CV_8UC3, Scalar(255,255,255));
    int center_x = visual_map.cols/2;
    int center_y = visual_map.rows/2;
    int robot_radius = 20;
    //======draw avoid route=====
    int angle_max;
    int angle_min;
    int angle;
    int far_line = 250;
    int close_line = 200;
    int final_line = 230;
    int af_line = 50;
    int x;
    int y;
    //===============
    draw_ellipse(visual_map,(360-(df_2*3)-90),(360-(df_1)*3-90),far_line,Scalar(18,116,54),2);
    
    //===============
    draw_ellipse(visual_map,(360-(dd_2*3)-90),(360-(dd_1*3)-90),close_line,Scalar(250,0,0),2);
    angle = 360-(good_angle*3)-90;
    x = center_x+(close_line-20)*cos(angle*DEG2RAD);
    y = center_y-(close_line-20)*sin(angle*DEG2RAD);
    line(visual_map, Point(center_x, center_y), Point(x, y), Scalar(250,0,0),2);
    //===============
    //cout<<"af_angle"<<af_angle<<endl;
    angle = int(360-(af_angle*3))-90;
    x = center_x+af_line*cos(angle*DEG2RAD);
    y = center_y-af_line*sin(angle*DEG2RAD);
    line(visual_map, Point(center_x, center_y), Point(x, y), Scalar(0,0,0), 6);
    //==================
    //===============
    //cout<<"final_angle"<<final_angle<<endl;
    angle = int(360-(final_angle*3))-90;
    x = center_x+final_line*cos(angle*DEG2RAD);
    y = center_y-final_line*sin(angle*DEG2RAD);
    line(visual_map, Point(center_x, center_y), Point(x, y), Scalar(255,200,0), 6);
    //==================
    //===========================
    //======draw the robot=======
    circle(visual_map, Point(center_x, center_y), robot_radius, Scalar(0, 0, 0), 2);
    line(visual_map, Point(center_x, center_y), Point(center_x, center_y-robot_radius), Scalar(0,0,0), 2);
    //===========================
    //====draw the black item====
    if(black_item_distance.size()>0){
        int black_angle = 360/black_item_distance.size();
        for(int i=0; i<black_item_distance.size(); i++){
            int angle = black_angle*i+90;
            int distance = black_item_distance.at(i);
            int x = center_x+distance*cos(angle*DEG2RAD);
            int y = center_y-distance*sin(angle*DEG2RAD);
            circle(visual_map, Point(x, y), 2, Scalar(0, 0, 0), -1);
        }
    }
    //===========================
    //====draw the red line======
    if(red_line_distance.size()>0){
        int red_angle = 360/red_line_distance.size();
        for(int i=0; i<red_line_distance.size(); i++){
            int angle = red_angle*i+90;
            int distance = red_line_distance.at(i);
            int x = center_x+distance*cos(angle*DEG2RAD);
            int y = center_y-distance*sin(angle*DEG2RAD);
            circle(visual_map, Point(x, y), 2, Scalar(0, 0, 200), -1);
        }
    }
    //===========================
    imshow("visual_map",visual_map);
    waitKey(10);
    return visual_map;
}

Mat Vision::convertTo3Channels(const Mat &binImg)
{
    Mat three_channel = Mat::zeros(binImg.rows, binImg.cols, CV_8UC3);
    vector<Mat> channels;
    for (int i = 0; i < 3; i++)
    {
        channels.push_back(binImg);
    }
    merge(channels, three_channel);
    return three_channel;
}

