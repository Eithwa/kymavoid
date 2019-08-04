#include "laser.h"
#define DEG2RAD  M_PI/180
Vision::Vision()
{
    //Parameter_getting();
    image_sub = nh.subscribe("/camera/image_raw", 1, &Vision::imageCb, this);
    FrameRate = 0.0;
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
void Vision::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //convert image data
        if (!cv_ptr->image.empty())
        {
            Source = cv_ptr->image.clone();
            cv::flip(Source, Source, 1); // reverse image
            
            Red_Mask = Source.clone();

            //================Red line detection==============
            red_binarization();
            red_line();
            Pub_redframe(Red_Mask);
            //cv::imshow("black_item", Black_Mask);
            //cv::imshow("red_line", Red_Mask);
            //cv::waitKey(1);
            //=======================FPS======================
            FrameRate = Rate();
            //================================================
            to_strategy.mpicture++;
            to_strategy.gray_ave = 10;
            mpicture.publish(to_strategy);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert to image!");
        return;
    }
}

void draw_scan(Mat &visual_map, int sensor_angle, int sensor_distance, int laser_angle, vector<double> ranges, Scalar color){
    int center_x = visual_map.cols/2;
    int center_y = visual_map.rows/2;
    Point tf;
    tf.x = sensor_distance*cos((sensor_angle+90)*DEG2RAD);
    tf.y = -sensor_distance*sin((sensor_angle+90)*DEG2RAD);
    circle(visual_map, Point(center_x+tf.x, center_y+tf.y), 4, Scalar(255,0,0), -1);
    int scan_range = 180;
    if(ranges.size()>0){
        //if(device_number<2){
        //    double black_angle = (double)270/ranges.size();
        //}
        double black_angle = (double)scan_range/ranges.size();
        for(int i=0; i<ranges.size(); i++){
            double angle = black_angle*i+sensor_angle-laser_angle;
            double distance = ranges.at(i)*100;
            double x = center_x+tf.x+distance*cos(angle*DEG2RAD);
            double y = center_y+tf.y-distance*sin(angle*DEG2RAD);
            circle(visual_map, Point(x, y), 1, color, -1);
        }
    }
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
    //===========================
    //======draw the robot=======
    
    circle(visual_map, Point(center_x, center_y), robot_radius, Scalar(255, 0, 0), 1);
    line(visual_map, Point(center_x, center_y), Point(center_x, center_y-robot_radius), Scalar(255,0,0), 1);
    //===========================
    //====draw the black item====
    if(scan_enable.at(0)==1){
        //int sensor1_angle = 0;
        //int sensor1_distance = 15;
        draw_scan(visual_map, robot_angle_1, robot_distance_1, scan_angle_1, ranges, Scalar(0,0,0));
    }
    //============
    if(scan_enable.at(1)==1){
        //int sensor2_angle = 243;
        //int sensor2_distance = 12;
        draw_scan(visual_map, robot_angle_2, robot_distance_2, scan_angle_2, ranges2, Scalar(0,0,0));
    }
    //============
    if(scan_enable.at(2)==1){
        //int sensor3_angle = 120;
        //int sensor3_distance = 15;
        draw_scan(visual_map, robot_angle_3, robot_distance_3, scan_angle_3, ranges3, Scalar(0,0,0));
    }
    //============
    //int sensor4_angle = 270;
    //int sensor4_distance = 15;
    //draw_scan(visual_map, sensor3_angle, sensor4_distance, ranges4, Scalar(0,0,0));
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
        //if(angle>90&&angle<270){
        //    blackItem.data.push_back(999);
        //    continue;
        //}
        for (int r = center_inner; r <= center_outer; r++)
        {
            int dis_x = x_ * r;
            int dis_y = y_ * r;

            int image_x = Frame_Area(center_x + dis_x, binarization_map.cols);
            int image_y = Frame_Area(center_y - dis_y, binarization_map.rows);

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
    //===============
    //add red line
    if(blackItem.data.size()==red_line_distance.size()){
        for(int i=0; i<blackItem.data.size(); i++){
            if(red_line_distance.at(i)<blackItem.data.at(i)){
                blackItem.data.at(i) = red_line_distance.at(i);
            }        
        }    
    }
    //===============
    //imshow("visual_map", visual_map);
    //waitKey(10);
    Pub_blackdis(blackItem);
    Pub_blackframe(visual_map);
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


//================================================
void Vision::red_binarization()
{
    Mat inputMat = Red_Mask.clone();
    Mat hsv(inputMat.rows, inputMat.cols, CV_8UC3, Scalar(0, 0, 0));
    Mat mask(inputMat.rows, inputMat.cols, CV_8UC1, Scalar(0, 0, 0));
    Mat mask2(inputMat.rows, inputMat.cols, CV_8UC1, Scalar(0, 0, 0));
    Mat dst(inputMat.rows, inputMat.cols, CV_8UC3, Scalar(0, 0, 0));
    Mat white(inputMat.rows, inputMat.cols, CV_8UC3, Scalar(255, 255, 255));
    int hmin, hmax, smin, smax, vmin, vmax;
    cvtColor(inputMat, hsv, CV_BGR2HSV);
    hmin = HSV_red[0]/2;
    hmax = HSV_red[1]/2;
    smin = HSV_red[2]*2.56;
    smax = HSV_red[3]*2.56;
    vmin = HSV_red[4]*2.56;
    vmax = HSV_red[5]*2.56;
    //for(int i =0; i<HSV_red.size(); i++)
    //{
    //    cout<<HSV_red[i]<<" ";
    //}
    //cout<<endl;
    if (HSV_red[0] <= HSV_red[1])
    {
        inRange(hsv, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), mask);
    }
    else
    {
        inRange(hsv, Scalar(hmin, smin, vmin), Scalar(255, smax, vmax), mask);
        inRange(hsv, Scalar(0, smin, vmin), Scalar(hmax, smax, vmax), mask2);
        mask = mask + mask2;
    }
    convertTo3Channels(mask);
    //開操作 (去除噪點)
    //Mat element = getStructuringElement(MORPH_RECT, Size(2, 2));
    //morphologyEx(mask, mask, MORPH_OPEN, element);
    
    white.copyTo(dst, (cv::Mat::ones(mask.size(), mask.type()) * 255 - mask));
    Red_Mask = dst;
    ///////////////////Show view/////////////////
    //cv::imshow("dst", dst);
    //cv::imshow("Red_Mask", Red_Mask);
    //cv::waitKey(1);
    /////////////////////////////////////////////
}
void Vision::red_line()
{
    int object_dis;
    std::vector<double>redItem_pixel;
    std_msgs::Int32MultiArray redRealDis;
    Mat binarization_map = Red_Mask.clone();
    red_line_distance.clear();
    //int black_angle = BlackAngleMsg;
    int black_angle = 3;//避障策略只能三度一條掃描線
    int center_front = FrontMsg;
    int center_inner = InnerMsg;
    int center_outer = OuterMsg;
    int center_x = CenterXMsg;
    int center_y = CenterYMsg;
    //std::cout<<"OuterMsg  "<<OuterMsg<<endl;

    for (int angle = 0; angle < 360; angle = angle + black_angle)
    {
        int angle_be = angle + center_front;

        if (angle_be >= 360)
            angle_be -= 360;

        double x_ = Angle_cos[angle_be];
        double y_ = Angle_sin[angle_be];
        for (int r = center_inner; r <= center_outer; r++)
        {
            int dis_x = x_ * r;
            int dis_y = y_ * r;

            int image_x = Frame_Area(center_x + dis_x, binarization_map.cols);
            int image_y = Frame_Area(center_y - dis_y, binarization_map.rows);

            if (binarization_map.data[(image_y * binarization_map.cols + image_x) * 3 + 0] == 0 
             && binarization_map.data[(image_y * binarization_map.cols + image_x) * 3 + 1] == 0 
             && binarization_map.data[(image_y * binarization_map.cols + image_x) * 3 + 2] == 0)
            {
                redItem_pixel.push_back(hypot(dis_x, dis_y));
                break;
            }
            else
            {
                Red_Mask.data[(image_y * Red_Mask.cols + image_x) * 3 + 0] = 0;
                Red_Mask.data[(image_y * Red_Mask.cols + image_x) * 3 + 1] = 0;
                Red_Mask.data[(image_y * Red_Mask.cols + image_x) * 3 + 2] = 255;
            }
            if (r >= center_outer)
            {
                redItem_pixel.push_back(hypot(999, 999));
            }
        }
    }

    for (int j = 0; j < redItem_pixel.size(); j++)
    {
        object_dis = Omni_distance(redItem_pixel[j]);
        red_line_distance.push_back(object_dis);
    }
    //new_vector.assign(original.begin(), original.end());
    redRealDis.data.assign(red_line_distance.begin(), red_line_distance.end());
    red_pub.publish(redRealDis);

    ///////////////////Show view/////////////////
    //cv::imshow("red_line", Red_Mask);
    //cv::waitKey(1);
    /////////////////////////////////////////////
}
