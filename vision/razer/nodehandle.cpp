#include "nodehandle.h"

NodeHandle::NodeHandle()
{
    Readyaml();
    AngleLUT();

    save_sub = nh.subscribe("/interface/bin_save", 1000, &NodeHandle::SaveButton_setting, this);
    scan_sub = nh.subscribe("/hokuyo_1/scan", 1000, &NodeHandle::scancall, this);
    scan_sub2 = nh.subscribe("/hokuyo_2/scan", 1000, &NodeHandle::scancall2, this);
    scan_sub3 = nh.subscribe("/hokuyo_3/scan", 1000, &NodeHandle::scancall3, this);
    scan_sub4 = nh.subscribe("/hokuyo_4/scan", 1000, &NodeHandle::scancall4, this);
    blackdis_sub = nh.subscribe("/vision/BlackRealDis", 1000, &NodeHandle::blackdiscall, this);;
    reddis_sub = nh.subscribe("/vision/redRealDis", 1000, &NodeHandle::reddiscall, this);
    avoid_sub = nh.subscribe("/avoid/route", 1000, &NodeHandle::avoidcall, this);
    avoidframe_pub = nh.advertise<sensor_msgs::Image>("/camera/avoid", 1);
    blackdis_pub = nh.advertise<std_msgs::Int32MultiArray>("/vision/BlackRealDis", 1);
    //http://localhost:8080/stream?topic=/camera/image_monitor webfor /camera/image
}
void NodeHandle::AngleLUT()
{
    double ang_PI;
    for (int ang = 0; ang <= 360; ang++)
    {
        ang_PI = ang * PI / 180;
        Angle_sin.push_back(sin(ang_PI));
        Angle_cos.push_back(cos(ang_PI));
    }
}
void NodeHandle::Readyaml()
{
    std::string param = YAML_PATH;
    const char *parampath = param.c_str();
    if (ifstream(parampath))
    {
        std::string temp = "rosparam load " + param + " /FIRA/vision";
        const char *load = temp.c_str();
        system(load);
        cout << "Read the yaml file" << endl;
        Parameter_getting();
    }
    else
    {
        ROS_ERROR("yaml file does not exist");
    }
}
void NodeHandle::Parameter_getting()
{
    cout << "get parameter" << endl;
    //===================中心參數=========================
    nh.getParam("/FIRA/vision/Center/Center_X", CenterXMsg);
    nh.getParam("/FIRA/vision/Center/Center_Y", CenterYMsg);
    nh.getParam("/FIRA/vision/Center/Inner", InnerMsg);
    nh.getParam("/FIRA/vision/Center/Outer", OuterMsg);
    nh.getParam("/FIRA/vision/Center/Front", FrontMsg);
    nh.getParam("/FIRA/vision/Center/Camera_high", Camera_HighMsg);
    nh.getParam("/FIRA/vision/Center/Horizon", HorizonMsg);
}
//======================前置處理結束=========================
int NodeHandle::Frame_Area(int coordinate, int range)
{
    if (coordinate < 0)
        coordinate = 0;
    else if (coordinate >= range)
        coordinate = range - 1;
    return coordinate;
}
//角度調整
//修正大於或小於360的角度
int NodeHandle::Angle_Adjustment(int angle)
{
    if (angle < 0)
        return angle + 360;
    else if (angle >= 360)
        return angle - 360;
    else
        return angle;
}
//========================save=============================
void NodeHandle::SaveButton_setting(const vision::bin msg)
{
    //cout<<"Save\n";
    //SaveButton = msg.bin;
    Parameter_getting();
}
void NodeHandle::blackdiscall(const std_msgs::Int32MultiArray msg){
    //new_vector.assign(original.begin(), original.end());
    black_item_distance.clear();
    black_item_distance.assign(msg.data.begin(), msg.data.end());
}
void NodeHandle::reddiscall(const std_msgs::Int32MultiArray msg){
    red_line_distance.clear();
    red_line_distance.assign(msg.data.begin(), msg.data.end());
}
void NodeHandle::avoidcall(const vision::avoid msg){
    df_1=msg.df_1;
    df_2=msg.df_2;
    df_1_dis=msg.df_1_dis;
    df_2_dis=msg.df_2_dis;
    //far_good_angle=msg.far_good_angle;
    far_good_angle = (df_1+df_2)/2;
    dd_1=msg.dd_1;
    dd_2=msg.dd_2;
    dd_1_dis=msg.dd_1_dis;
    dd_2_dis=msg.dd_2_dis;
    good_angle=msg.good_angle;
    final_angle=msg.final_angle;
    af_angle=msg.af_angle;
    v_fast = msg.v_fast;
    v_af=msg.v_af;
}
void NodeHandle::scancall(const sensor_msgs::LaserScan msg){
    ranges.clear();
    ranges.assign(msg.ranges.begin(), msg.ranges.end());
    //cout<<"size: "<<ranges.size()<<endl;
}
void NodeHandle::scancall2(const sensor_msgs::LaserScan msg){
    ranges2.clear();
    ranges2.assign(msg.ranges.begin(), msg.ranges.end());
    //cout<<"size: "<<ranges.size()<<endl;
}
void NodeHandle::scancall3(const sensor_msgs::LaserScan msg){
    ranges3.clear();
    ranges3.assign(msg.ranges.begin(), msg.ranges.end());
    //cout<<"size: "<<ranges.size()<<endl;
}
void NodeHandle::scancall4(const sensor_msgs::LaserScan msg){
    ranges4.clear();
    ranges4.assign(msg.ranges.begin(), msg.ranges.end());
    //cout<<"size: "<<ranges.size()<<endl;
}
//========================distance=========================
double NodeHandle::camera_f(double Omni_pixel)
{
    double m = (Omni_pixel * 0.0099) / 60; // m = H1/H0 = D1/D0    D0 + D1 = 180
    double D0 = 180 / (1 + m);             // D1 = m   *D0
    double D1 = 180 / (1 + (1 / m));       // D0 = 1/m *D1
    double f = 1 / (1 / D0 + 1 / D1);
    //ROS_INFO("m = %f D0 = %f D1 = %f F = %f",m,D0,D1,f);
    return D1;
}
double NodeHandle::Omni_distance(double pixel_dis)
{
    double Z = -1 * Camera_HighMsg; //Camera_HighMsg=650mm;
    //double c  =  D0/2;
    double c = 83.125;
    double b = c * 0.8722;
    double f = camera_f(OuterMsg * 2 * 0.9784);
    double r = atan2(f, pixel_dis * 0.0099);
    double dis = Z * (pow(b, 2) - pow(c, 2)) * cos(r) / ((pow(b, 2) + pow(c, 2)) * sin(r) - 2 * b * c) * 0.1;
    if (dis < 0 || dis > 999)
    {
        dis = 999;
    }
    //ROS_INFO("%f %f %f %f",Z,c,r,dis);
    return dis;
}
//======================publisher==========================
void NodeHandle::Pub_avoidframe(Mat frame)
{
    sensor_msgs::ImagePtr frameMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    avoidframe_pub.publish(frameMsg);
}
void NodeHandle::Pub_blackdis(std_msgs::Int32MultiArray distance)
{
    blackdis_pub.publish(distance);
}
