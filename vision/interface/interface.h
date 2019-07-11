#include "nodehandle.h"

class Vision : protected NodeHandle
{
  public:
    Vision(string topic);
    ~Vision();
    //===========================================
    double GetFrameRate() { return FrameRate; }
    cv::Mat GetSource() { return Source; }
    cv::Mat CameraModel(const cv::Mat iframe);
    cv::Mat CenterModel(const cv::Mat iframe);
    cv::Mat ScanModel(const cv::Mat iframe);
    cv::Mat ColorModel(const cv::Mat iframe);
    cv::Mat White_Line(const cv::Mat iframe);
    cv::Mat Black_Line(const cv::Mat iframe);

  private:
    ros::NodeHandle nh;
    ros::Subscriber image_sub;
    void imageCb(const sensor_msgs::ImageConstPtr &msg);
    double Rate();
    double FrameRate;
    cv::Mat Source;
};
