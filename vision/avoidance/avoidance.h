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
    Mat convertTo3Channels(const Mat &binImg);
    double FrameRate;
    //==========================================
    cv::Mat interface_map;
};
