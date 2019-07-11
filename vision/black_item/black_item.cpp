#include "black_item.h"
#include "omp.h"
#define DEG2RAD  M_PI/180
Vision::Vision()
{
    //image_sub = nh.subscribe(VISION_TOPIC, 1,static_cast<void (Vision::*)(const sensor_msgs::ImageConstPtr&)>(&Vision::imageCb),this);
    image_sub = nh.subscribe(VISION_TOPIC, 1, &Vision::imageCb, this);
}
Vision::Vision(string topic)
{
    image_sub = nh.subscribe(topic, 1, &Vision::imageCb, this);
    FrameRate = 0.0;
}
Vision::~Vision()
{
    Source.release();
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
//int mousex,mousey;
//void onMouse(int Event, int x, int y, int flags, void* param)
//{
//	if (Event == CV_EVENT_LBUTTONDOWN) {
//		mousex = x;
//		mousey = y;
//		cout<<"("<<x<<","<<y<<")"<<endl;
//		onclick = 1;
//	}
//}
//=============================影像接收=======================================
void Vision::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //convert image data
        if (!cv_ptr->image.empty())
        {
            #pragma omp parallel sections
            {
                #pragma omp section
                {
                    //================Black obstacle detection========
                    black_binarization();
                    black_filter();
                    black_item();
                }
                #pragma omp section
                {
                    //================Red line detection==============
                    red_binarization();
                    red_line();
                }
            }
            cv::imshow("black_item", Black_Mask);
            cv::imshow("red_line", Red_Mask);
            cv::waitKey(1);
            //=======================FPS======================
            FrameRate = Rate();
            //================================================
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert to image!");
        return;
    }
}
cv::Mat Vision::Black_Line(const cv::Mat iframe)
{
    cv::Mat threshold(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    cv::Mat edge;
    cv::Mat oframe(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    Mat visual_map(550, 750, CV_8UC3, Scalar(0,0,0));
    int ground = OuterMsg-100;
    if(HorizonMsg>0&&HorizonMsg<OuterMsg){
        ground = HorizonMsg;
    }
    vector<int> black_dis;
    blackdis.data.clear();
    //======================threshold===================
    for (int i = 0; i < iframe.rows; i++)
    {
        for (int j = 0; j < iframe.cols; j++)
        {
            unsigned char gray = (iframe.data[(i * iframe.cols * 3) + (j * 3) + 0] + iframe.data[(i * iframe.cols * 3) + (j * 3) + 1] + iframe.data[(i * iframe.cols * 3) + (j * 3) + 2]) / 3;
            if (gray < BlackGrayMsg)
            {
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 0] = 0;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 1] = 0;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 2] = 0;
            }
            else
            {
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 0] = 255;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 1] = 255;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 2] = 255;
            }
        }
    }
    //cv::imshow("threshold", threshold);
    //Canny(threshold, edge, 50, 150, 3);
    //edge=convertTo3Channels(edge);
    //cv::imshow("edge", edge);

//=========================
    int hmin, hmax, smin, smax, vmin, vmax;
    Mat hsv(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    Mat mask(iframe.rows, iframe.cols, CV_8UC1, Scalar(0, 0, 0));
    Mat mask2(iframe.rows, iframe.cols, CV_8UC1, Scalar(0, 0, 0));
    Mat dst(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    if (HSV_red.size() == 6)
    {
        hmin = HSV_red[0]/2;
        hmax = HSV_red[1]/2;
        smin = HSV_red[2]*2.55;
        smax = HSV_red[3]*2.55;
        vmin = HSV_red[4]*2.55;
        vmax = HSV_red[5]*2.55;
        /*for(int i =0; i<HSV_red.size(); i++)
        {
            cout<<HSV_red[i]<<" ";
        }
        cout<<endl;*/
        cvtColor(iframe, hsv, CV_BGR2HSV);

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

        //開操作 (去除一些噪點)
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(mask, mask, MORPH_OPEN, element);

        //閉操作 (連接一些連通域)
        morphologyEx(mask, mask, MORPH_CLOSE, element);
        //cv::imshow("mask", mask);
        iframe.copyTo(dst, (cv::Mat::ones(mask.size(), mask.type()) * 255 - mask));
        //cv::imshow("dst", dst);
        //waitKey(10);
    }
    else
    {
        cout << "HSV vector size: " << HSV_red.size() << " error\n";
    }
    int red_range[360];
    for(int i=0; i<360; i++){
        red_range[i]=0;
    }
    mask=convertTo3Channels(mask);
    for (double angle = FrontMsg; angle < 360 + FrontMsg; angle = angle + BlackAngleMsg)
    {
        for (int r = ground; r >= InnerMsg; r--)
        {
            int angle_be = Angle_Adjustment(angle);

            int x_ = r * Angle_cos[angle_be];
            int y_ = r * Angle_sin[angle_be];
            int x = Frame_Area(CenterXMsg + x_, oframe.cols);
            int y = Frame_Area(CenterYMsg - y_, oframe.rows);
            if (mask.data[(y * mask.cols + x)*3 + 0] == 255)
            {
                red_range[angle_be] = r-3;
                break;
            }
        }
    }
    //=====================draw the scan line===========
    Mat input = threshold.clone();
    oframe = threshold.clone();
    //oframe = threshold.clone();
    int line_count = 0;
    for (double angle = FrontMsg; angle < 360 + FrontMsg; angle = angle + BlackAngleMsg)
    {
        int count = 0;
        for (int r = InnerMsg; r <= ground; r++)
        {

            if(line_count%2==0&&r<(InnerMsg+ground)/2){
                r=(InnerMsg+ground)/2;
            }
            int angle_be = Angle_Adjustment(angle);

            int x_ = r * Angle_cos[angle_be];
            int y_ = r * Angle_sin[angle_be];
            int x = Frame_Area(CenterXMsg + x_, oframe.cols);
            int y = Frame_Area(CenterYMsg - y_, oframe.rows);
            if (input.data[(y * input.cols + x) * 3 + 0] != 255)
            {
                if (angle_be == FrontMsg)
                {
                    oframe.data[(y * oframe.cols + x) * 3 + 0] = 255;
                    oframe.data[(y * oframe.cols + x) * 3 + 1] = 150;
                    oframe.data[(y * oframe.cols + x) * 3 + 2] = 0;
                }
                else
                {
                    oframe.data[(y * oframe.cols + x) * 3 + 0] = 0;
                    oframe.data[(y * oframe.cols + x) * 3 + 1] = 0;
                    oframe.data[(y * oframe.cols + x) * 3 + 2] = 255;
                }
            }
            if(r>=red_range[angle_be]||r>=ground){
                //cout<<"fuck"<<endl;
                blackdis.data.push_back(250);
                break;
            }
            if (input.data[(y * input.cols + x) * 3 + 0] == 255)
            {
                circle(oframe, Point(x, y), 2, Scalar(255, 0, 0), 1);
                
                x=visual_map.cols/2+Omni_distance(r)*cos((angle-FrontMsg)*DEG2RAD);
                y=visual_map.rows/2-Omni_distance(r)*sin((angle-FrontMsg)*DEG2RAD);
                circle(visual_map, Point(x, y), 2, Scalar(0, 255, 255), -1);

                x=Omni_distance(r)*cos((angle-FrontMsg)*DEG2RAD);
                y=-Omni_distance(r)*sin((angle-FrontMsg)*DEG2RAD);
                //blackdis.data.push_back(x);
                //blackdis.data.push_back(y);
                //>>>>>>>>>>20190621>>>>>>>>>>>>>>>>>>
                double realdis = Omni_distance(r);
                if(realdis>=250)realdis=250;
                blackdis.data.push_back(Omni_distance(r));
                //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                count++;
                //if(count>10)break;
                //blackdis.pusht_back(Omni_distance(r));
                break;
            }
        }
        line_count++;
    }
    line(oframe, Point(CenterXMsg, CenterYMsg - InnerMsg), Point(CenterXMsg, CenterYMsg + InnerMsg), Scalar(0, 255, 0), 1);
    line(oframe, Point(CenterXMsg - InnerMsg, CenterYMsg), Point(CenterXMsg + InnerMsg, CenterYMsg), Scalar(0, 255, 0), 1);
    circle(oframe, Point(CenterXMsg, CenterYMsg), InnerMsg, Scalar(0, 255, 0), 0);
    circle(oframe, Point(CenterXMsg, CenterYMsg), ground, Scalar(0, 255, 0), 0);
    circle(oframe, Point(CenterXMsg, CenterYMsg), OuterMsg, Scalar(0, 255, 0), 0);

    //draw the robot
    circle(visual_map, Point(visual_map.cols/2, visual_map.rows/2), 5, Scalar(0, 0, 255), 1);
    line(visual_map, Point(visual_map.cols/2,visual_map.rows/2), Point(visual_map.cols/2+5,visual_map.rows/2), Scalar(0,0,255), 1);
    //cv::imshow("visual_map", visual_map);
    //cv::imshow("blackdis", oframe);
    //cv::waitKey(10);

    return oframe;
}
//==================================
void Vision::black_binarization()
{
    cv::Mat iframe = Source.clone();
    cv::Mat threshold(iframe.rows, iframe.cols, CV_8UC3, Scalar(0, 0, 0));
    for (int i = 0; i < iframe.rows; i++)
    {
        for (int j = 0; j < iframe.cols; j++)
        {
            unsigned char gray = (iframe.data[(i * iframe.cols * 3) + (j * 3) + 0] + iframe.data[(i * iframe.cols * 3) + (j * 3) + 1] + iframe.data[(i * iframe.cols * 3) + (j * 3) + 2]) / 3;
            if (gray < BlackGrayMsg)
            {
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 0] = 0;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 1] = 0;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 2] = 0;
            }
            else
            {
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 0] = 255;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 1] = 255;
                threshold.data[(i * threshold.cols * 3) + (j * 3) + 2] = 255;
            }
        }
    }
    Black_Mask = threshold.clone();
}

class Coord
{
public:
    Coord(int x_, int y_) : x(x_), y(y_) {}
    Coord operator+(const Coord &addon) const { return Coord(x + addon.x, y + addon.y); }
    int get_x() const { return x; }
    int get_y() const { return y; }

private:
    int x, y;
};

Coord directions[8] = {
    Coord(0, 1),
    Coord(0, -1),
    Coord(-1, 0),
    Coord(1, 0),
    Coord(-1, 1),
    Coord(1, 1),
    Coord(-1, -1),
    Coord(1, -1)};
bool is_black(Mat &img, const Coord &c) 
{ 
    if(img.data[(c.get_y() * img.cols + c.get_x()) * 3 + 0] == 0
     &&img.data[(c.get_y() * img.cols + c.get_x()) * 3 + 1] == 0
     &&img.data[(c.get_y() * img.cols + c.get_x()) * 3 + 2] == 0)
    {
        return true;
    }else{
        return false;
    }
}
void is_checked(Mat &img, const Coord &c){
    img.data[(c.get_y() * img.cols + c.get_x()) * 3 + 0] = 255;
    img.data[(c.get_y() * img.cols + c.get_x()) * 3 + 1] = 255;
    img.data[(c.get_y() * img.cols + c.get_x()) * 3 + 2] = 255;
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
void Vision::black_filter()
{
    int inner = InnerMsg, outer = OuterMsg, centerx = CenterXMsg, centery = CenterYMsg;
    int detection_range = HorizonMsg;
    int width = Black_Mask.cols-1, length = Black_Mask.rows-1;
    int obj_size = obj_filter_size;

    vector<vector<Coord> > obj; //物件列表

    Mat check_map = Mat(Size(Black_Mask.cols, Black_Mask.rows), CV_8UC3, Scalar(0, 0, 0));//確認搜尋過的pix

    //inner中心塗白 outer切斷與外面相連黑色物體
    circle(Black_Mask, Point(centerx, centery), inner, Scalar(255, 255, 255), -1);
    circle(Black_Mask, Point(centerx, centery), outer, Scalar(255, 0, 0), 2);
    //搜尋範圍顯示
    circle(Black_Mask, Point(centerx, centery), detection_range, Scalar(255, 0, 0), 1);
    //rectangle(Black_Mask, Point(centerx - detection_range, centery - detection_range), Point(centerx + detection_range, centery + detection_range), Scalar(255, 0, 0), 1);
    //選取的範圍做搜尋
    //0為黑
    for (int i = centerx - detection_range; i < centerx + detection_range; i++)
    {
        for (int j = centery - detection_range; j < centery + detection_range; j++)
        {
            //std::cout << i << " "	<< j << std::endl;
            if(hypot(centerx-i,centery-j)>detection_range) continue;
            if (is_black(check_map,Coord(i, j)))
            {
                is_checked(check_map,Coord(i, j));

                if (is_black(Black_Mask, Coord(i, j)))
                {

                    queue<Coord> bfs_list;
                    bfs_list.push(Coord(i, j));

                    vector<Coord> dot;
                    dot.push_back(Coord(i, j));

                    //放入佇列
                    while (!bfs_list.empty())
                    {

                        Coord ori = bfs_list.front();
                        bfs_list.pop();

                        //搜尋八方向
                        for (int k = 0; k < 8; k++)
                        {
                            Coord dst = ori + directions[k];

                            //處理邊界
                            if ((dst.get_x() < 0) || (dst.get_x() >= width) || (dst.get_y() < 0) || (dst.get_y() >= length)) continue;
                            if(hypot(centerx-dst.get_x(),centery-dst.get_y())>outer) continue;
                            if (!is_black(check_map,Coord(dst.get_x(), dst.get_y()))) continue;

                            if (is_black(Black_Mask, dst))
                            {
                                bfs_list.push(dst);
                                dot.push_back(dst);
                            }
                            is_checked(check_map,Coord(dst.get_x(), dst.get_y()));
                        }
                    }
                    obj.push_back(dot);
                }
            }
        }
    }

    //上色
    for (int i = 0; i < obj.size(); i++)
    {
        //需要塗色的物體大小
        if (obj[i].size() > obj_size) continue;
        for (int j = 0; j < obj[i].size(); j++)
        {
            Coord point = obj[i][j];
            line(Black_Mask, Point(point.get_x(), point.get_y()), Point(point.get_x(), point.get_y()), Scalar(255, 0, 255), 1);
        }
    }

    //cv::imshow("black filter", Black_Mask);
    //cv::waitKey(1);
}
void Vision::black_item()
{
    int object_dis;
    std::vector<double>blackItem_pixel;
    std_msgs::Int32MultiArray BlackRealDis;
    Mat binarization_map = Black_Mask.clone();
    //int black_angle = BlackAngleMsg;
    int black_angle = 3;//避障策略只能三度一條掃描線
    int center_front = FrontMsg;
    int center_inner = InnerMsg;
    int center_outer = OuterMsg;
    int center_x = CenterXMsg;
    int center_y = CenterYMsg;
    for (int angle = 0; angle < 360; angle = angle + black_angle)
    {
        int angle_be = angle + center_front;

        if (angle_be >= 360)
            angle_be -= 360;

        double x_ = Angle_cos[angle_be];
        double y_ = Angle_sin[angle_be];
        for (int r = center_inner - 1; r <= HorizonMsg; r++)
        {
            int dis_x = x_ * r;
            int dis_y = y_ * r;

            int image_x = Frame_Area(center_x + dis_x, binarization_map.cols);
            int image_y = Frame_Area(center_y - dis_y, binarization_map.rows);

            if (binarization_map.data[(image_y * binarization_map.cols + image_x) * 3 + 0] == 0 
             && binarization_map.data[(image_y * binarization_map.cols + image_x) * 3 + 1] == 0 
             && binarization_map.data[(image_y * binarization_map.cols + image_x) * 3 + 2] == 0)
            {
                blackItem_pixel.push_back(hypot(dis_x, dis_y));
                break;
            }
            else
            {
                Black_Mask.data[(image_y * Black_Mask.cols + image_x) * 3 + 0] = 0;
                Black_Mask.data[(image_y * Black_Mask.cols + image_x) * 3 + 1] = 0;
                Black_Mask.data[(image_y * Black_Mask.cols + image_x) * 3 + 2] = 255;
            }
            if (r <= HorizonMsg)
            {
                blackItem_pixel.push_back(hypot(999, 999));
            }
        }
    }

    //ROS_INFO("%d , blackangle=%d",blackItem_pixel.size(),black_angle);
    for (int j = 0; j < blackItem_pixel.size(); j++)
    {
        object_dis = Omni_distance(blackItem_pixel[j]);
        BlackRealDis.data.push_back(object_dis);
    }
    blackdis_pub.publish(BlackRealDis);
    //cv::imshow("black_item", Black_Mask);
    //cv::waitKey(1);
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
    smin = HSV_red[2]*2.55;
    smax = HSV_red[3]*2.55;
    vmin = HSV_red[4]*2.55;
    vmax = HSV_red[5]*2.55;
    if (HSV_red[0] >= HSV_red[1])
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
    Mat element = getStructuringElement(MORPH_RECT, Size(2, 2));
    morphologyEx(mask, mask, MORPH_OPEN, element);
    
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
    //int black_angle = BlackAngleMsg;
    int black_angle = 3;//避障策略只能三度一條掃描線
    int center_front = FrontMsg;
    int center_inner = InnerMsg;
    int center_outer = OuterMsg;
    int center_x = CenterXMsg;
    int center_y = CenterYMsg;

    for (int angle = 0; angle < 360; angle = angle + black_angle)
    {
        int angle_be = angle + center_front;

        if (angle_be >= 360)
            angle_be -= 360;

        double x_ = Angle_cos[angle_be];
        double y_ = Angle_sin[angle_be];
        for (int r = center_inner; r <= HorizonMsg; r++)
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
            if (r <= HorizonMsg)
            {
                redItem_pixel.push_back(hypot(999, 999));
            }
        }
    }

    for (int j = 0; j < redItem_pixel.size(); j++)
    {
        object_dis = Omni_distance(redItem_pixel[j]);
        redRealDis.data.push_back(object_dis);
    }
    red_pub.publish(redRealDis);

    to_strategy.mpicture++;
    to_strategy.gray_ave = gray_ave;
    mpicture.publish(to_strategy);

    ///////////////////Show view/////////////////
    //cv::imshow("red_line", Red_Mask);
    //cv::waitKey(1);
    /////////////////////////////////////////////
}
