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
void Vision::draw_ellipse(Mat &iframe, double main_angle, double angle_min, double angle_max, double ang_min_dis, double ang_max_dis, Scalar color, double width){
    int x1;
    int y1;
    int x2;
    int y2;
    int center_x = iframe.cols/2;
    int center_y = iframe.rows/2;
    x1 = center_x+ang_max_dis*cos(angle_max*DEG2RAD);
    y1 = center_y-ang_max_dis*sin(angle_max*DEG2RAD);
    line(iframe, Point(center_x, center_y), Point(x1, y1), color, width);

    x2 = center_x+ang_min_dis*cos(angle_min*DEG2RAD);
    y2 = center_y-ang_min_dis*sin(angle_min*DEG2RAD);
    line(iframe, Point(center_x, center_y), Point(x2, y2), color, width);
    //if(main_angle>angle_min&& main_angle<angle_max){
    //    int angle = main_angle-180;
    //    //int angle = 360-(good_angle*3)-90-180;
    //    //int angle = (angle_min+angle_max)/2-180;
    //    int x1_,y1_;
    //    int x2_,y2_;
    //    x1_ = x1+(ang_max_dis+50)*cos(angle*DEG2RAD);
    //    y1_ = y1-(ang_max_dis+50)*sin(angle*DEG2RAD);
    //    line(iframe, Point(x1_, y1_), Point(x1, y1), color, width);
    //    x2_ = x2+(ang_min_dis+50)*cos(angle*DEG2RAD);
    //    y2_ = y2-(ang_min_dis+50)*sin(angle*DEG2RAD);
    //    line(iframe, Point(x2_, y2_), Point(x2, y2), color, width);
    //}

    line(iframe, Point(x1, y1), Point(x2, y2), color, width);
    //ellipse(iframe, Point(center_x, center_y), Size(ang_max_dis, ang_max_dis), 0, 360 - angle_max, 360 - angle_min, color, width);
}
cv::Mat Vision::draw_interface()
{
    Rate();
    Mat visual_map(500, 500, CV_8UC3, Scalar(255,255,255));
    int center_x = visual_map.cols/2;
    int center_y = visual_map.rows/2;
    int robot_radius = 20;
    //======draw avoid route=====
    double angle_max;
    double angle_min;
    double angle;
    int far_line = 250;
    int close_line = 200;
    int final_line = 200;
    int af_line = 50;
    int x;
    int y;
    //===============
    //cout<<"final_angle"<<final_angle<<endl;
    angle = int(360-(final_angle*3))-90;
    x = center_x+final_line*cos(angle*DEG2RAD);
    y = center_y-final_line*sin(angle*DEG2RAD);
    line(visual_map, Point(center_x, center_y), Point(x, y), Scalar(255,200,0), 7);
    line(visual_map, Point(center_x, center_y), Point(x, y), Scalar(255,255,255), 6);    
    //cout<<"v_fast  "<<v_fast<<endl;
    if(v_fast>0&&v_fast<30)v_fast=30;
    x = center_x+v_fast*2*cos(angle*DEG2RAD);
    y = center_y-v_fast*2*sin(angle*DEG2RAD);
    line(visual_map, Point(center_x, center_y), Point(x, y), Scalar(255,200,0), 5);
    int x1,y1,x2,y2;
    int final_rl = 100;
    x1 = center_x+robot_radius*cos((angle+90)*DEG2RAD);
    y1 = center_y-robot_radius*sin((angle+90)*DEG2RAD);
    x2 = x1+final_rl*cos(angle*DEG2RAD);
    y2 = y1-final_rl*sin(angle*DEG2RAD);
    line(visual_map, Point(x1, y1), Point(x2, y2), Scalar(255,200,0), 1);
    x2 = x1+robot_radius*cos((angle+180)*DEG2RAD);
    y2 = y1-robot_radius*sin((angle+180)*DEG2RAD);
    line(visual_map, Point(x1, y1), Point(x2, y2), Scalar(255,200,0), 1);

    x1 = center_x+robot_radius*cos((angle-90)*DEG2RAD);
    y1 = center_y-robot_radius*sin((angle-90)*DEG2RAD);
    x2 = x1+final_rl*cos(angle*DEG2RAD);
    y2 = y1-final_rl*sin(angle*DEG2RAD);
    line(visual_map, Point(x1, y1), Point(x2, y2), Scalar(255,200,0), 1);
    x2 = x1+robot_radius*cos((angle+180)*DEG2RAD);
    y2 = y1-robot_radius*sin((angle+180)*DEG2RAD);
    line(visual_map, Point(x1, y1), Point(x2, y2), Scalar(255,200,0), 1);
    //==================
    //===============
    draw_ellipse(visual_map,(360-(far_good_angle*3)-90),(360-(df_2*3)-90),(360-(df_1)*3-90),df_2_dis,df_1_dis,Scalar(18,116,54),2);
    angle = 360-(far_good_angle*3)-90;
    x1 = center_x+robot_radius*cos((angle+90)*DEG2RAD);
    y1 = center_y-robot_radius*sin((angle+90)*DEG2RAD);
    x2 = x1+final_rl*cos(angle*DEG2RAD);
    y2 = y1-final_rl*sin(angle*DEG2RAD);
    line(visual_map, Point(x1, y1), Point(x2, y2), Scalar(18,116,54), 1);
    x2 = x1+robot_radius*cos((angle+180)*DEG2RAD);
    y2 = y1-robot_radius*sin((angle+180)*DEG2RAD);
    line(visual_map, Point(x1, y1), Point(x2, y2), Scalar(18,116,54), 1);

    x1 = center_x+robot_radius*cos((angle-90)*DEG2RAD);
    y1 = center_y-robot_radius*sin((angle-90)*DEG2RAD);
    x2 = x1+final_rl*cos(angle*DEG2RAD);
    y2 = y1-final_rl*sin(angle*DEG2RAD);
    line(visual_map, Point(x1, y1), Point(x2, y2), Scalar(18,116,54), 1);
    x2 = x1+robot_radius*cos((angle+180)*DEG2RAD);
    y2 = y1-robot_radius*sin((angle+180)*DEG2RAD);
    line(visual_map, Point(x1, y1), Point(x2, y2), Scalar(18,116,54), 1);
    //==================
    //===============
    draw_ellipse(visual_map,(360-(good_angle*3)-90),(360-(dd_2*3)-90),(360-(dd_1*3)-90),dd_2_dis,dd_1_dis,Scalar(250,0,0),2);
    angle = 360-(good_angle*3)-90;
    x = center_x+(close_line-20)*cos(angle*DEG2RAD);
    y = center_y-(close_line-20)*sin(angle*DEG2RAD);
    line(visual_map, Point(center_x, center_y), Point(x, y), Scalar(250,0,0),2);

    x1 = center_x+robot_radius*cos((angle+90)*DEG2RAD);
    y1 = center_y-robot_radius*sin((angle+90)*DEG2RAD);
    x2 = x1+final_rl*cos(angle*DEG2RAD);
    y2 = y1-final_rl*sin(angle*DEG2RAD);
    line(visual_map, Point(x1, y1), Point(x2, y2), Scalar(250,0,0), 1);
    x2 = x1+robot_radius*cos((angle+180)*DEG2RAD);
    y2 = y1-robot_radius*sin((angle+180)*DEG2RAD);
    line(visual_map, Point(x1, y1), Point(x2, y2), Scalar(250,0,0), 1);

    x1 = center_x+robot_radius*cos((angle-90)*DEG2RAD);
    y1 = center_y-robot_radius*sin((angle-90)*DEG2RAD);
    x2 = x1+final_rl*cos(angle*DEG2RAD);
    y2 = y1-final_rl*sin(angle*DEG2RAD);
    line(visual_map, Point(x1, y1), Point(x2, y2), Scalar(250,0,0), 1);
    x2 = x1+robot_radius*cos((angle+180)*DEG2RAD);
    y2 = y1-robot_radius*sin((angle+180)*DEG2RAD);
    line(visual_map, Point(x1, y1), Point(x2, y2), Scalar(250,0,0), 1);
    //===============
    //cout<<"af_angle"<<af_angle<<endl;
    angle = int(360-(af_angle*3))-90;
    x = center_x+v_af*cos(angle*DEG2RAD);
    y = center_y-v_af*sin(angle*DEG2RAD);
    line(visual_map, Point(center_x, center_y), Point(x, y), Scalar(211,102,160), 2);
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
    //imshow("visual_map", visual_map);
    //waitKey(10);
    Pub_avoidframe(visual_map);
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

