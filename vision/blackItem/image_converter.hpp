#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <deque>
#include <vector>
#include <vision/visionlook.h>
using std::vector;

#include <queue>
using std::queue;

#include "std_msgs/Int32MultiArray.h"

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;

class ImageConverter
{
public:
ImageConverter();
~ImageConverter();

double Omni_distance(double dis_pixel);

private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    vision::visionlook to_strategy;
    std_msgs::Int32MultiArray BlackRealDis;
    std_msgs::Int32MultiArray redRealDis;
    ros::Publisher black_pub;
    ros::Publisher red_pub;
    ros::Publisher mpicture;
    //==========parameter===========
    void get_center();
    void get_distance();
    void get_whitedata();
    void get_Camera();
    int center_x, center_y, center_inner, center_outer, center_front;
    int dis_gap;
    int black_gray,black_angle,gray_ave,setgray;
    double Camera_H,Camera_f;
    std::vector<int> red;
    std::vector<int>dis_space, dis_pixel;
    std::vector<double>blackItem_pixel;
    std::vector<double>redItem_pixel;
    //======Image processing=========
    Mat Main_frame;
    Mat Black_Mask;
    Mat Red_Mask;
    void imageCb(const sensor_msgs::ImageConstPtr&);
    double Rate();
    double FrameRate;
    int obj_filter_size;
    void black_binarization();
    void black_filter();
    void black_item();
    void red_binarization();
    void red_line();

    std::vector<double> Angle_sin;
    std::vector<double> Angle_cos;

};

