#include "razer.h"
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
        int angle = main_angle-180;
        //int angle = 360-(good_angle*3)-90-180;
        //int angle = (angle_min+angle_max)/2-180;
        int x1_,y1_;
        int x2_,y2_;
        x1_ = x1+(ang_max_dis+50)*cos(angle*DEG2RAD);
        y1_ = y1-(ang_max_dis+50)*sin(angle*DEG2RAD);
        line(iframe, Point(x1_, y1_), Point(x1, y1), color, width);
        x2_ = x2+(ang_min_dis+50)*cos(angle*DEG2RAD);
        y2_ = y2-(ang_min_dis+50)*sin(angle*DEG2RAD);
        line(iframe, Point(x2_, y2_), Point(x2, y2), color, width);
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
    int razer_offset = 15;
    //===========================
    //======draw the robot=======
    circle(visual_map, Point(center_x, center_y), 2, Scalar(255, 0, 0), -1);
    circle(visual_map, Point(center_x, center_y+razer_offset), robot_radius, Scalar(255, 0, 0), 1);
    line(visual_map, Point(center_x, center_y+razer_offset), Point(center_x, center_y+razer_offset-robot_radius), Scalar(255,0,0), 1);
    //===========================
    //====draw the black item====
    if(ranges.size()>0){
        double black_angle = (double)270/ranges.size();
        for(int i=0; i<ranges.size(); i++){
            double angle = black_angle*i-45;
            double distance = ranges.at(i)*100;
            double x = center_x+distance*cos(angle*DEG2RAD);
            double y = center_y-distance*sin(angle*DEG2RAD);
            circle(visual_map, Point(x, y), 1, Scalar(0, 0, 0), -1);
        }
    }
    //===========================
    int center_front = 90;
    int black_angle = 3;
    int center_inner = 20;
    int center_outer = 250;
    Mat binarization_map = visual_map.clone();
    std_msgs::Int32MultiArray blackItem;
    for (int angle = 0; angle < 360; angle = angle + black_angle)
    {
        int angle_be = angle + center_front;

        if (angle_be >= 360)
            angle_be -= 360;

        double x_ = Angle_cos[angle_be];
        double y_ = Angle_sin[angle_be];
        //cout<<angle<<endl;
        if(angle>90&&angle<270){
            blackItem.data.push_back(999);
            continue;
        }
        for (int r = center_inner; r <= center_outer; r++)
        {
            int dis_x = x_ * r;
            int dis_y = y_ * r;

            int image_x = Frame_Area(center_x + dis_x, binarization_map.cols);
            int image_y = Frame_Area(center_y+razer_offset - dis_y, binarization_map.rows);

            if (binarization_map.data[(image_y * binarization_map.cols + image_x) * 3 + 0] == 0 
             && binarization_map.data[(image_y * binarization_map.cols + image_x) * 3 + 1] == 0 
             && binarization_map.data[(image_y * binarization_map.cols + image_x) * 3 + 2] == 0)
            {
                blackItem.data.push_back(hypot(dis_x, dis_y));
                break;
            }
            else
            {
                visual_map.data[(image_y * visual_map.cols + image_x) * 3 + 0] = 0;
                visual_map.data[(image_y * visual_map.cols + image_x) * 3 + 1] = 0;
                visual_map.data[(image_y * visual_map.cols + image_x) * 3 + 2] = 255;
            }
            if (r >= center_outer)
            {
                blackItem.data.push_back(999);
            }
        }
    }
    imshow("visual_map", visual_map);
    waitKey(10);
    Pub_blackdis(blackItem);
    //Pub_avoidframe(visual_map);
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

