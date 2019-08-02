#include "nodehandle.h"

class Vision : protected NodeHandle
{
  public:
    Vision();
    ~Vision();
    cv::Mat draw_interface();
    double Rate();

  private:
    ros::NodeHandle nh;
    ros::Subscriber image_sub;
    void imageCb(const sensor_msgs::ImageConstPtr &msg);
    Mat convertTo3Channels(const Mat &binImg);
    void draw_ellipse(Mat &iframe, double main_angle, double angle_min, double angle_max, double ang_min_dis, double ang_max_dis, Scalar color, double width);
    double FrameRate;
    void red_binarization();
    void red_line();
    //==========================================
    cv::Mat interface_map;
    cv::Mat Source;
    cv::Mat Red_Mask;
};
