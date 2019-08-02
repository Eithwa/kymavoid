#include "nodehandle.h"

NodeHandle::NodeHandle()
{
    device_number = 0;
    robot_angle_1 = 0;
    robot_distance_1 = 15;
    scan_angle_1 = 0;
    robot_angle_2 = 120;
    robot_distance_2 = 15;
    scan_angle_2 = 0;
    robot_angle_3 = 240;
    robot_distance_3 = 15;
    scan_angle_3 = 0;

    scan_enable.push_back(1);
    scan_enable.push_back(1);
    scan_enable.push_back(1);

    Readyaml();
    AngleLUT();

    save_sub = nh.subscribe("/interface/bin_save", 1000, &NodeHandle::SaveButton_setting, this);
    scan_sub = nh.subscribe("/hokuyo_1/scan", 1000, &NodeHandle::scancall, this);
    scan_sub2 = nh.subscribe("/hokuyo_2/scan", 1000, &NodeHandle::scancall2, this);
    scan_sub3 = nh.subscribe("/hokuyo_3/scan", 1000, &NodeHandle::scancall3, this);
    scan_sub4 = nh.subscribe("/hokuyo_4/scan", 1000, &NodeHandle::scancall4, this);
    scan_enable_sub = nh.subscribe("/scan/scan_enable", 1000, &NodeHandle::scan_enable_call, this);
    scan_parameter_sub = nh.subscribe("/scan/scan_parameter", 1000, &NodeHandle::scan_parameter_call, this);

    blackframe_pub = nh.advertise<sensor_msgs::Image>("/camera/black", 1);
    blackdis_pub = nh.advertise<std_msgs::Int32MultiArray>("/vision/BlackRealDis", 1);
    redframe_pub = nh.advertise<sensor_msgs::Image>("/camera/red", 1);
    red_pub = nh.advertise<std_msgs::Int32MultiArray>("/vision/redRealDis", 1);
    mpicture = nh.advertise<vision::visionlook>("/vision/picture_m", 1);
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

    nh.getParam("/FIRA/vision/HSV/Ball", HSV_red);

    nh.getParam("/FIRA/vision/laser/scan_enable", scan_enable);
    nh.getParam("/FIRA/vision/laser/scan_parameter", scan_parameter);
    if(scan_parameter.size()>=9){
        robot_angle_1 = scan_parameter.at(0);
        robot_distance_1 = scan_parameter.at(1);
        scan_angle_1 = scan_parameter.at(2);
        robot_angle_2 = scan_parameter.at(3);
        robot_distance_2 = scan_parameter.at(4);
        scan_angle_2 = scan_parameter.at(5);
        robot_angle_3 = scan_parameter.at(6);
        robot_distance_3 = scan_parameter.at(7);
        scan_angle_3 = scan_parameter.at(8);
    }
    
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
//========================scan=============================
void NodeHandle::scancall(const sensor_msgs::LaserScan msg){
    if(device_number<1)device_number=1;
    ranges.clear();
    ranges.assign(msg.ranges.begin(), msg.ranges.end());
    //cout<<"size: "<<ranges.size()<<endl;
}
void NodeHandle::scancall2(const sensor_msgs::LaserScan msg){
    if(device_number<2)device_number=2;
    ranges2.clear();
    ranges2.assign(msg.ranges.begin(), msg.ranges.end());
    //cout<<"size: "<<ranges.size()<<endl;
}
void NodeHandle::scancall3(const sensor_msgs::LaserScan msg){
    if(device_number<3)device_number=3;
    ranges3.clear();
    ranges3.assign(msg.ranges.begin(), msg.ranges.end());
    //cout<<"size: "<<ranges.size()<<endl;
}
void NodeHandle::scancall4(const sensor_msgs::LaserScan msg){
    if(device_number<4)device_number=4;
    ranges4.clear();
    ranges4.assign(msg.ranges.begin(), msg.ranges.end());
    //cout<<"size: "<<ranges.size()<<endl;
}
void NodeHandle::scan_enable_call(const std_msgs::Int32MultiArray msg){
    if(msg.data.size()>=3){
        scan_enable.clear();
        scan_enable.assign(msg.data.begin(), msg.data.end()); 
    }
}
void NodeHandle::scan_parameter_call(const std_msgs::Int32MultiArray msg){
    if(msg.data.size()>=9){
        scan_parameter.clear();
        scan_parameter.assign(msg.data.begin(), msg.data.end()); 
        robot_angle_1 = scan_parameter.at(0);
        robot_distance_1 = scan_parameter.at(1);
        scan_angle_1 = scan_parameter.at(2);
        robot_angle_2 = scan_parameter.at(3);
        robot_distance_2 = scan_parameter.at(4);
        scan_angle_2 = scan_parameter.at(5);
        robot_angle_3 = scan_parameter.at(6);
        robot_distance_3 = scan_parameter.at(7);
        scan_angle_3 = scan_parameter.at(8);
    }
    
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
void NodeHandle::Pub_blackframe(Mat frame)
{
    sensor_msgs::ImagePtr blackframeMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    blackframe_pub.publish(blackframeMsg);
}
void NodeHandle::Pub_redframe(Mat frame)
{
    sensor_msgs::ImagePtr redframeMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    redframe_pub.publish(redframeMsg);
}
void NodeHandle::Pub_blackdis(std_msgs::Int32MultiArray distance)
{
    blackdis_pub.publish(distance);
}
