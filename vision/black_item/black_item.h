#include "nodehandle.h"

class Vision : protected NodeHandle
{
  public:
    Vision();
    Vision(string topic);
    ~Vision();
    void release();
    cv::Mat Black_Line(const cv::Mat iframe);

  private:
    ros::NodeHandle nh;
    ros::Subscriber image_sub;
    void imageCb(const sensor_msgs::ImageConstPtr &msg);
    //void imageCb(const sensor_msgs::CompressedImageConstPtr& msg);
    Mat convertTo3Channels(const Mat &binImg);
    double Rate();
    double FrameRate;

    int obj_filter_size;
    void black_binarization();
    void black_filter();
    void black_item();
    void red_binarization();
    void red_line();
    //==========================================
    cv::Mat Source;
    cv::Mat Black_Mask;
    cv::Mat Red_Mask;
};
