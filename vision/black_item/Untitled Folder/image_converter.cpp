#include "image_converter.hpp"
#include "math.h"
#include "omp.h"

using namespace std;
using namespace cv;

ImageConverter::ImageConverter(): 
    it_(nh),
    FrameRate(0.0),
    obj_filter_size(500)
{
    get_Camera();
    get_center();
    get_distance();
    get_whitedata();
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
    black_pub = nh.advertise<std_msgs::Int32MultiArray>("/vision/BlackRealDis", 1);
    red_pub = nh.advertise<std_msgs::Int32MultiArray>("/vision/redRealDis", 1);
    mpicture = nh.advertise<vision::visionlook>("/vision/picture_m", 1);

    double ang_PI;

    for (int ang = 0; ang < 360; ang++)
    {
        ang_PI = ang * M_PI / 180;
        Angle_sin.push_back(sin(ang_PI));
        Angle_cos.push_back(cos(ang_PI));
    }
}

ImageConverter::~ImageConverter()
{
}

int Frame_area(int num, int range)
{
    if (num < 0)
        num = 0;
    else if (num >= range)
        num = range - 1;
    return num;
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr &msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        cv::flip(cv_ptr->image, Main_frame, 1);
        Black_Mask = Main_frame.clone();
        Red_Mask = Main_frame.clone();
        static double StartTime = ros::Time::now().toSec();
        double EndTime = ros::Time::now().toSec();
        if(EndTime-StartTime>2){
            get_Camera();
            get_center();
            get_distance();
            get_whitedata();
            StartTime = EndTime;
        }
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
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}
double ImageConverter::Rate()
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
void ImageConverter::black_binarization()
{
    int gray_count = 0;
    gray_ave = 0;
    for (int i = 0; i < Main_frame.rows; i++)
    {
        for (int j = 0; j < Main_frame.cols; j++)
        {
            unsigned char gray = (Main_frame.data[(i * Main_frame.cols * 3) + (j * 3) + 0] 
                                + Main_frame.data[(i * Main_frame.cols * 3) + (j * 3) + 1] 
                                + Main_frame.data[(i * Main_frame.cols * 3) + (j * 3) + 2]) / 3;
            if (gray < black_gray)
            {
                gray_ave += gray;
                gray_count++;
            }
        }
    }
    gray_count = (gray_count == 0) ? 0.00001 : gray_count;
    gray_ave = gray_ave / gray_count;

    for (int i = 0; i < Main_frame.rows; i++)
    {
        for (int j = 0; j < Main_frame.cols; j++)
        {
            unsigned char gray = (Main_frame.data[(i * Main_frame.cols * 3) + (j * 3) + 0] 
                                + Main_frame.data[(i * Main_frame.cols * 3) + (j * 3) + 1] 
                                + Main_frame.data[(i * Main_frame.cols * 3) + (j * 3) + 2]) / 3;
            if (gray < black_gray - (setgray - gray_ave))
            {
                Black_Mask.data[(i * Black_Mask.cols * 3) + (j * 3) + 0] = 0;
                Black_Mask.data[(i * Black_Mask.cols * 3) + (j * 3) + 1] = 0;
                Black_Mask.data[(i * Black_Mask.cols * 3) + (j * 3) + 2] = 0;
            }
            else
            {
                Black_Mask.data[(i * Black_Mask.cols * 3) + (j * 3) + 0] = 255;
                Black_Mask.data[(i * Black_Mask.cols * 3) + (j * 3) + 1] = 255;
                Black_Mask.data[(i * Black_Mask.cols * 3) + (j * 3) + 2] = 255;
            }
        }
    }
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
Mat convertTo3Channels(const Mat &binImg)
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
void ImageConverter::black_filter()
{
    int inner = center_inner, outer = center_outer, centerx = center_x, centery = center_y;
    int detection_range = (inner+outer)/2;
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
void ImageConverter::black_item()
{
    int object_dis;
    blackItem_pixel.clear();
    BlackRealDis.data.clear();
    Mat binarization_map = Black_Mask.clone();
    for (int angle = 0; angle < 360; angle = angle + black_angle)
    {
        int angle_be = angle + center_front;

        if (angle_be >= 360)
            angle_be -= 360;

        double x_ = Angle_cos[angle_be];
        double y_ = Angle_sin[angle_be];
        for (int r = center_inner - 1; r <= center_outer; r++)
        {
            int dis_x = x_ * r;
            int dis_y = y_ * r;

            int image_x = Frame_area(center_x + dis_x, binarization_map.cols);
            int image_y = Frame_area(center_y - dis_y, binarization_map.rows);

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
            if (r == center_outer)
            {
                blackItem_pixel.push_back(hypot(dis_x, dis_y));
            }
        }
    }

    //ROS_INFO("%d , blackangle=%d",blackItem_pixel.size(),black_angle);
    for (int j = 0; j < blackItem_pixel.size(); j++)
    {
        object_dis = Omni_distance(blackItem_pixel[j]);
        BlackRealDis.data.push_back(object_dis);
    }
    black_pub.publish(BlackRealDis);
    //cv::imshow("black_item", Black_Mask);
    //cv::waitKey(1);
}
void ImageConverter::red_binarization()
{
    Mat inputMat = Red_Mask.clone();
    Mat hsv(inputMat.rows, inputMat.cols, CV_8UC3, Scalar(0, 0, 0));
    Mat mask(inputMat.rows, inputMat.cols, CV_8UC1, Scalar(0, 0, 0));
    Mat mask2(inputMat.rows, inputMat.cols, CV_8UC1, Scalar(0, 0, 0));
    Mat dst(inputMat.rows, inputMat.cols, CV_8UC3, Scalar(0, 0, 0));
    Mat white(inputMat.rows, inputMat.cols, CV_8UC3, Scalar(255, 255, 255));
    int hmin, hmax, smin, smax, vmin, vmax;
    cvtColor(inputMat, hsv, CV_BGR2HSV);
    hmax = double(red[0]*0.5);
    hmin = double(red[1]*0.5);
    smax = red[2];
    smin = red[3];
    vmax = red[4];
    vmin = red[5];
    if (red[0] >= red[1])
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
void ImageConverter::red_line()
{
    int object_dis;
    redItem_pixel.clear();
    redRealDis.data.clear();
    Mat binarization_map = Red_Mask.clone();
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

            int image_x = Frame_area(center_x + dis_x, binarization_map.cols);
            int image_y = Frame_area(center_y - dis_y, binarization_map.rows);

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
            if (r == center_outer)
            {
                redItem_pixel.push_back(hypot(dis_x, dis_y));
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
double ImageConverter::Omni_distance(double dis_pixel)
{
    double Z = -1 * Camera_H;
    double c = 83.125;
    double b = c * 0.8722;

    double f = Camera_f;

    double dis;

    //double pixel_dis = sqrt(pow(object_x,2)+pow(object_y,2));

    double pixel_dis = dis_pixel;

    double r = atan2(f, pixel_dis * 0.0099);

    dis = Z * (pow(b, 2) - pow(c, 2)) * cos(r) / ((pow(b, 2) + pow(c, 2)) * sin(r) - 2 * b * c);

    if (dis / 10 < 0 || dis / 10 > 999)
    {
        dis = 9990;
    }

    return dis / 10;
}

void ImageConverter::get_center()
{
    nh.setParam("/FIRA/gray_ave", gray_ave);
    nh.getParam("/AvoidChallenge/GraySet", setgray);
    nh.getParam("/FIRA/Center/X", center_x);
    nh.getParam("/FIRA/Center/Y", center_y);
    nh.getParam("/FIRA/Center/Inner", center_inner);
    nh.getParam("/FIRA/Center/Outer", center_outer);
    nh.getParam("/FIRA/Center/Front", center_front);
}
void ImageConverter::get_distance()
{
    nh.getParam("/FIRA/Distance/Gap", dis_gap);
    nh.getParam("/FIRA/Distance/Space", dis_space);
    nh.getParam("/FIRA/Distance/Pixel", dis_pixel);
}
void ImageConverter::get_Camera()
{
    nh.getParam("/FIRA/Camera/High", Camera_H);
    nh.getParam("/FIRA/Camera/Focal", Camera_f);
}
void ImageConverter::get_whitedata()
{
    red.clear();
    if (nh.hasParam("/FIRA/blackItem/obj_filter_size"))
    {
        nh.getParam("/FIRA/blackItem/obj_filter_size", obj_filter_size);
    }
    nh.getParam("/FIRA/blackItem/gray", black_gray);
    nh.getParam("/FIRA/blackItem/angle", black_angle);
    nh.getParam("/FIRA/HSV/Redrange", red);
    //std::cout<<red[0]<<" "<<red[1]<<" "<<red[2]<<" "<<red[3]<<" "<<red[4]<<" "<<red[5]<<std::endl;
}
