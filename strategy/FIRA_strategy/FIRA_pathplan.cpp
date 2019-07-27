#include "FIRA_pathplan.h"
#include "math.h"
#include "time.h"
#define DEG2RAD  M_PI/180
#define RAD2DEG 180.0/M_PI
//=========Environment init=============//
FIRA_pathplan_class::FIRA_pathplan_class(){
    opponent = false;
    v_fast = 50;
    not_good_p=0;
    far_good_angle = 60;
    good_angle = 60;
    af_angle = 0;
    for(int i=0; i<360; i++){
        env.blackdis[i]=0;
    }
    //回傳策略線向量給影像
    tovision  = nh.advertise<strategy::strategylook>("/vision/strategy_line",1);
    route_pub = nh.advertise<vision::avoid>("/avoid/route",1);
    nh.setParam("/AvoidChallenge/avoid_go",0);
}
//start---simulator---
void FIRA_pathplan_class::setEnv(Environment iEnv){
    env = iEnv;
    if(opponent){
        for(int i = 0;i < PLAYERS_PER_SIDE;i++){
            env.home[i] = iEnv.opponent[i];
            env.opponent[i] = iEnv.home[i];
        }
    }
}
//end  ---simulator---

//車頭追球
void FIRA_pathplan_class::strategy_head2ball(int i){
    Vector3D ball = env.currentBall.pos;
    Vector3D robot = env.home[i].pos;
    // double robot_rotation = env.home[0].v_yaw;
    double robot_rotation = env.home[i].rotation;
    double x = ball.x - robot.x;
    if (x==0) x= very_small;
    double y = ball.y - robot.y;
    double a = atan2(y,x)*rad2deg; //*/
    //double a = 120;
    //if(x<0)       a = a-180;
    double        angle = a - robot_rotation;
    if(angle<-180)angle = angle+360;
    if(angle>180)angle = angle-360;
    env.home[i].v_yaw = angle*5;
}
//跑定點
void FIRA_pathplan_class::strategy_dst(double target_x,double target_y){
    //printf("dstX=%lf,dstY=%lf\n",dstX,dstY);
    double v_x = target_x - env.home[0].pos.x;
    double v_y = target_y - env.home[0].pos.y;
    env.home[0].v_x = v_x;
    env.home[0].v_y = v_y;
    //env.home[0].v_yaw = angle;
}
//車頭朝球＋走定點
void FIRA_pathplan_class::strategy_dst_head2ball(double target_x,double target_y){

    Vector3D ball = env.currentBall.pos;
    Vector3D robot = env.home[2].pos;

    double x = target_x - robot.x;
    double y = target_y - robot.y;
    double angle;
    double robot_rotation = env.home[2].rotation;
    double tar_dis = sqrt(x*x + y*y);
    double tar_ang = atan2(y,x)*rad2deg;
    double v,w;

    double x_ball = ball.x - robot.x;
    if (x_ball==0) x_ball = very_small;
    double y_ball = ball.y - robot.y;

    //v = 0.2;//1 * (1-exp(-distance/3));
    //a = atan(y/x) / pi * 180;
    double a = atan2(y_ball,x_ball) * rad2deg;
    //if(x_ball<0) a = a - 180;  //??
    angle = a - robot_rotation ;

    if(angle<-180) angle += 360;
    if(angle>=180) angle -= 360;

    //if(fabs(angle)==0)angle = very_small;

    //---speed planning---
    if(tar_dis < 0.1){
        v=0;
        if(fabs(angle) < 5) w=0;
        else w = angle/fabs(angle) * 200 * (1-exp(-1*fabs(angle)/10))+10;  //angular velocity
    }else{
        v = 250 * (1-exp(-1*tar_dis/10)) +10; //velocity
        w = 0;
    }
    env.home[2].v_x = x;
    env.home[2].v_y = y;
    env.home[2].v_yaw = w;//angle;
}

void FIRA_pathplan_class::personalStrategy(int robotIndex,int action){
    switch(action){
    case action_AvoidBarrier:
        strategy_AvoidBarrier(robotIndex);
        break;
    case Role_Halt:
        strategy_AvoidBarrier(robotIndex);
        //strategy_Halt(robotIndex);
        break;
    }
}
//=========================避障挑戰賽=================================
double Rate()
{
    double ALPHA = 0.5;
    double dt;
    static int frame_counter = 0;
    static double frame_rate = 0.0;
    static double StartTime = ros::Time::now().toNSec();
    double EndTime;

    frame_counter++;
    if (frame_counter == 1)
    {
        EndTime = ros::Time::now().toNSec();
        dt = (EndTime - StartTime) / frame_counter;
        StartTime = EndTime;
        //if (dt != 10)
        if (dt != 0)
        {
            frame_rate = (1000000000.0 / dt) * ALPHA + frame_rate * (1.0 - ALPHA);
            std::cout << "Rate: " << frame_rate << std::endl;
        }

        frame_counter = 0;
    }
    return frame_rate;
}
ScanInfo::ScanInfo()
{
    scan_main = 60;
    scan_right = scan_main+20;
    scan_left =  scan_main-20;
    move_main = scan_main;
    move_right = scan_right;
    move_left = scan_left;
    max_vacancy_number = 0;
    for(int i=0; i<30; i++){
        for(int j=0; j<2; j++){
            obstacle[i][j]=0;
            vacancy[i][j]=0;
        }
    }
}
void FIRA_pathplan_class::connected(){
    //影像持續接收判斷
    not_good_p=(env.picture_m==b_picture_m)?not_good_p+1:0;
    b_picture_m=env.picture_m;
    //std::cout<<"not_good_p  "<<not_good_p<<std::endl;
    if(not_good_p>20){
        //v_fast=0;
        //預設為模擬模式 無接收blackitem 連接網頁後會關閉模擬模式才能正常啟動
        printf("\n\n[WORNING] 未接收blackitem資料 / 攝影機連線中斷 / 未連接網頁介面\n\n\n");
    }
}
#define OUTER 0
#define INNER 1
#define ARTIFICIAL_FIELD 2
void FIRA_pathplan_class::RoutePlan(ScanInfo &THIS){
    double close_dis = Distant[0];//60 speed (10) //54  speed (30,10)
    double halfclose_dis = Distant[1];//80
    double far_dis=Distant[2];//200
    int size_ignore = 4;
    if(THIS.type == ARTIFICIAL_FIELD)size_ignore=1;
    bool is_vacancy=true;

    bool obstacle_flag=0;//b_ok=1可以走b_ok=0黑色
    int obstacle[30][2]={0};
    int obstacle_number=0;
    int obstacle_size=99;

    bool vacancy_flag=0;
    int vacancy[30][2]={0};//最多儲存30個空間
    int vacancy_number=0;
    int vacancy_size=99;

    for(int i= THIS.scan_left ; i<=THIS.scan_right ; i++){
    //for(int i= 0; i<=118 ; i++){
        //若黑線小於far dis(250) 且黑線大於中層的距離 或者紅線小於最遠距離 b_ok = false
        if(THIS.type == OUTER){
            is_vacancy=((env.blackdis[i] <= far_dis)/*&&(env.blackdis[i] >= halfclose_dis)||(env.reddis[i]<=far_dis)*/)?false:true;
        }else if(THIS.type == INNER){
            is_vacancy=((env.blackdis[i] <= halfclose_dis)/* ||(env.reddis[i]<=250)*/)?false:true;
            //is_vacancy=((env.blackdis[i] <= 80))?false:true;
        }else if(THIS.type == ARTIFICIAL_FIELD){
            is_vacancy=(env.blackdis[i] <= close_dis+20)?false:true;
        }else{
            is_vacancy=(env.blackdis[i] <= close_dis+20)?false:true;
        }
        //is_vacancy=(env.blackdis[i] <= close_dis+20)?false:true;
        //is_vacancy=((env.blackdis[i] <= halfclose_dis)||(env.reddis[i]<=250))?false:true;
        if(is_vacancy==true){//若可以走的話
            obstacle_flag=false;//障礙物計算flag關閉
            if(vacancy_flag==false){//新的可走空間
                vacancy_flag=true;//可以走的flag開啟
                vacancy_number++;
                vacancy_size=1;
                vacancy[vacancy_number][0]=i;//可以走的空間起始
                vacancy[vacancy_number][1]=i;//可以走的空間結尾
            }else{//如果continuedline_ok=1 如可以走的flag開啟
                vacancy_size++;//可以走的空間寬度++
                vacancy[vacancy_number][1]=i;//更新結尾
            }
            //如果掃到攝影機支架黑色障礙物有可能中斷?(可走空間小於4也會被清除所以沒問題)
            if(obstacle_size<size_ignore){//如果前一個障礙物掃線數小於4 初始99
                vacancy_number--;//可走空間減1 合併前一個空間
                if(vacancy_number==0){
                    vacancy_number=1;
                    vacancy[vacancy_number][0]=THIS.scan_left;//Ok_place初始改成起始最左
                }
                obstacle_number--;//障礙物減1 合併前一個障礙物
                vacancy[vacancy_number][1]=i;//可以走的空間結尾
                vacancy_size=vacancy[vacancy_number][1]-vacancy[vacancy_number][0]+1;//可以走的寬度 （為什麼要+1)
                obstacle_size=99;//返回障礙物初始值 是否不需要?
            }
        }else{
            vacancy_flag=false;//可以走的flag關閉
            if(obstacle_flag==false){
                obstacle_flag=true;//障礙物計算flag開啟
                obstacle_number++;
                obstacle_size=1;//障礙物寬度計算
                obstacle[obstacle_number][0]=i;//障礙物起始
                obstacle[obstacle_number][1]=i;//障礙物結尾
            }else{
                obstacle_size++;//更新障礙物寬度
                obstacle[obstacle_number][1]=i;//更新結尾
            }
            if(vacancy_size<size_ignore){//如果可走空間小於4
                obstacle_number--;//障礙物減1
                if(obstacle_number==0){
                    obstacle_number=1;
                    obstacle[obstacle_number][0]=THIS.scan_left;
                }
                vacancy_number--;//可以走的空間去除(小於4條線)
                obstacle[obstacle_number][1]=i;//改變障礙結尾 合併前一個障礙物
                obstacle_size=obstacle[obstacle_number][1]-obstacle[obstacle_number][0]+1;//計算障礙物寬度
                vacancy_size=99;//返回可走空間初始值
            }
        }
    }
    THIS.obstacle_number = obstacle_number;
    THIS.vacancy_number = vacancy_number;
    for(int i=0; i<30; i++){
        for(int j=0; j<2; j++){
            THIS.obstacle[i][j]=obstacle[i][j];
            THIS.vacancy[i][j]=vacancy[i][j];
        }
    }
    //找到最大可走空間
    int size=0;
    int center_angle;
    int max_size=0;
    int max_vacancy_number=0;
    int pre_angle;
    if(THIS.type==OUTER){
        pre_angle = far_good_angle;
    }
    if(THIS.type==INNER){
        pre_angle = good_angle;
    }
    if(THIS.type ==ARTIFICIAL_FIELD){
        pre_angle = af_angle;
    }
    for(int i=1 ; i<=vacancy_number ;i++){
        size=vacancy[i][1]-vacancy[i][0];
        center_angle = (vacancy[i][1]+vacancy[i][0])/2;
        if(size>max_size&&
            center_angle>THIS.scan_left&&
            center_angle<THIS.scan_right&&
            center_angle<90+15&&
            center_angle>30-15&&
            abs(center_angle-pre_angle)<50){
            max_size=size;
            max_vacancy_number=i;
        }
    }
    // int angle=60;
    // int closest_angle = 999;
    // if(THIS.type==OUTER){
    //     for(int i=1 ; i<=vacancy_number ;i++){
    //         size=vacancy[i][1]-vacancy[i][0];
    //         center_angle = (vacancy[i][1]+vacancy[i][0])/2;
    //         if(abs(center_angle-angle)<closest_angle&&
    //             center_angle>THIS.scan_left&&
    //             center_angle<THIS.scan_right&&
    //             center_angle<90+15&&
    //             center_angle>30-15&&
    //             abs(center_angle-pre_angle)<50){
    //             max_size=size;
    //             max_vacancy_number=i;
    //         }
    //     }
    // }
    //std::cout<<"center_angle   "<<center_angle<<std::endl;
    //解決區域最佳解,若兩空間大小相差小於4條線 選擇離goodangle較近的空間
    int close_vacancy_number=0;
    int close_angle=999;
    int close_size=0;
    size = 0;
    if(THIS.type==INNER||THIS.type==OUTER){
        for(int i=1 ; i<=vacancy_number ;i++){
            size=vacancy[i][1]-vacancy[i][0];
            center_angle = (vacancy[i][1]+vacancy[i][0])/2;

            if(abs(center_angle-good_angle)<close_angle&&
                center_angle>THIS.scan_left&&
                center_angle<THIS.scan_right&&
                center_angle<(90+15)&&
                center_angle>(30-15)&&
                abs(center_angle-pre_angle)<50){
                close_angle = center_angle;
                close_vacancy_number=i;
                close_size=size;
            }
        }
    }
    if(abs(max_size-close_size)<4){
        max_vacancy_number = close_vacancy_number;
    }
    //==========================================
    THIS.max_vacancy_number = max_vacancy_number;
    THIS.move_left  = vacancy[max_vacancy_number][0];
    THIS.move_right = vacancy[max_vacancy_number][1];
    // std::cout<<"THIS.move_left  "<<THIS.move_left<<std::endl;
    if(THIS.move_left>0)THIS.move_left_dis=env.blackdis[THIS.move_left-1];
    else THIS.move_left_dis=env.blackdis[THIS.move_left];
    if(THIS.move_right<119)THIS.move_right_dis=env.blackdis[THIS.move_right+1];
    else THIS.move_right_dis=THIS.move_right;
    //=================
    int left_obstacle_number=0;
    int right_obstacle_number=0;
    int obstacle_distance_error = 0;
    //計算空洞左右邊界與機器人距離

    int search_rangle = 2;
    int min_distance = 999;
    int min_distance_angle=THIS.move_left;
    for(int i=THIS.move_left-search_rangle ; i<(THIS.move_left+search_rangle); i++){
        if(i<119 && i>0){
            if(env.blackdis[i]<min_distance){
                min_distance = env.blackdis[i];
                min_distance_angle = i;
            }
        }
    }
    THIS.move_left = min_distance_angle;
    THIS.move_left_dis=min_distance;
    //==================
    min_distance = 999;
    min_distance_angle =  THIS.move_right;
    for(int i=THIS.move_right-search_rangle ; i<(THIS.move_right+search_rangle); i++){
        if(i<119 && i>0){
            if(env.blackdis[i]<min_distance){
                min_distance = env.blackdis[i];
                min_distance_angle = i;
            }
        }
    }
    THIS.move_right = min_distance_angle;
    THIS.move_right_dis=min_distance;
    //==================
    // if(THIS.move_right<119){
    //     for(int i=0 ; i<=obstacle_number; i++){
    //         if(obstacle[i][0]<=(THIS.move_right+1) && obstacle[i][1]>=(THIS.move_right+1)){
    //             right_obstacle_number =i;
    //             // std::cout<<"right_obstacle_number "<<right_obstacle_number<<std::endl;
    //             break;
    //         }
    //     }
    //     double average=0;
    //     double sum=0;
    //     for(int i=obstacle[right_obstacle_number][0]; i<=obstacle[right_obstacle_number][1]; i++){
    //         sum += env.blackdis[i];
    //     }
    //     average = sum/(obstacle[right_obstacle_number][1]-obstacle[right_obstacle_number][0]+1);
    //     if(abs(env.blackdis[THIS.move_right+1]-average)<obstacle_distance_error){
    //         //THIS.move_right_dis=env.blackdis[THIS.move_right+1];
    //         THIS.move_right_dis=average;
    //     }else{
    //         THIS.move_right_dis=average;
    //     }
    //     THIS.move_right_dis=env.blackdis[THIS.move_right+1];
    // }else {
    //     THIS.move_right_dis=env.blackdis[THIS.move_right];
    // }

    if(THIS.move_left_dis>150)THIS.move_left_dis=150;
    if(THIS.move_right_dis>150)THIS.move_right_dis=150;
    //std::cout<<"move_right: "<<THIS.move_right<<"  move_left: "<<THIS.move_left<<std::endl;
    // std::cout<<"move_right_dis: "<<THIS.move_right_dis<<"  move_left_dis: "<<THIS.move_left_dis<<std::endl;

    // if(THIS.type==INNER){
    //     //cout<<"sc"
    //     for(int i=0 ; i<=obstacle_number; i++){
    //         std::cout<<"obstacle  "<<obstacle_number<<"  "<<obstacle[i][0]<<"  "<<obstacle[i][1]<<std::endl;
    //     }
    //     for(int i=0 ; i<=vacancy_number; i++){
    //         std::cout<<"vacancy  "<<vacancy_number<<"  "<<vacancy[i][0]<<"  "<<vacancy[i][1]<<std::endl;
    //     }
    // }
    //==============
    //角度規劃 避免碰撞
    int robot_radius = 20;
    double left_angle_tmp=0;
    //double left_angle=0;
    double left_angle=THIS.move_left;
    double right_angle_tmp=0;
    //double right_angle=0;
    double right_angle=THIS.move_right;
    THIS.move_main = (THIS.move_right+THIS.move_left)/2;
    int x=robot_radius;
    int y=sqrt(pow(THIS.move_left_dis,2)-pow(x,2));
    left_angle_tmp = atan2(y,x)*RAD2DEG;
    //std::cout<<"y  "<<y<<"  leftangle  "<<left_angle<<std::endl;
    left_angle = (360-left_angle_tmp-90)/3+THIS.move_left-180;
    if(left_angle<0)left_angle+=360/3;
    if(left_angle>360/3)left_angle-=360/3;
    x=-robot_radius;
    y=sqrt(pow(THIS.move_right_dis,2)-pow(x,2));
    right_angle_tmp = atan2(y,x)*RAD2DEG;
    right_angle = (360-right_angle_tmp-90)/3+THIS.move_right-180;
    if(right_angle<0)right_angle+=360/3;
    if(right_angle>360/3)right_angle-=360/3;

    //y-y1=((y2-y1)/(x2-x1))*(x-x1)
    //m=(y2-y1)/(x2-x1)
    //m*x-m*x1=y-y1
    //m*x-m*x1-y+y1=0
    //(y2-y1)x-(x2-x1)y-(y2-y1)x1+(x2-x1)y1=0
    //ax+by+c=0
    //a=(y2-y1)
    //b=-(x2-x1)
    //c=-(y2-y1)x1+(x2-x1)y1
    //線外一點P到直線L距離為d(P,L)
    //d(P,L)=abs(ax0+by0+c)/sqrt(a*a+b*b)

    //std::cout<<"move_right_dis: "<<THIS.move_right_dis<<"  move_left_dis: "<<THIS.move_left_dis<<std::endl;
    //std::cout<<"move_main  "<<THIS.move_main<<"   left_angle  "<<left_angle<<"  right_angle "<<right_angle<<std::endl;
    //THIS.move_main = (THIS.move_left+THIS.move_right)/2;
    // double offset = 1.1;
    // double angle_offset = 7;
    // if((THIS.move_left+THIS.move_right)/2<(60-angle_offset)){
    //     if (left_angle*offset<right_angle){
    //         left_angle = left_angle*offset;
    //     }
    // }
    // if((THIS.move_left+THIS.move_right)/2>(60+angle_offset)){
    //     if (right_angle/offset>left_angle){
    //         right_angle = right_angle/offset;
    //     }
    // }
    THIS.move_main = (right_angle+left_angle)/2;
    //THIS.move_main = THIS.move_left;
    //THIS.move_main = left_angle;
    //THIS.move_main = right_angle;
    //THIS.move_main = move_main;

}
void FIRA_pathplan_class::strategy_AvoidBarrier(int Robot_index){
    std::cout<<"===============Avoid Obstacles Information===============\n";
    Rate();
    connected();
    int r_number = Robot_index;
    double FB_x = env.home[Robot_index].FB_x;
    double FB_y = env.home[Robot_index].FB_y;
    double FB_imu = env.home[Robot_index].FB_yaw;
    //-------------Distant-----------//
    double close_dis = Distant[0];//60 speed (10) //54  speed (30,10)
    double halfclose_dis = Distant[1];//80
    double far_dis=Distant[2];//200
    static int RedLine = 0; //紅線判斷flag 1left 2right
    ////主要向量範圍
    static int main_vec=60;//60*3 定180為車頭角

    ///////redline
    //==========這邊有疑問========
    static double fb_error=0;
    static double FB_XX=0;
    static double count=0;
    FB_XX=(fabs(FB_x-fb_error)>0.1)?FB_XX:FB_XX+FB_x-fb_error;

    //if(fabs(FB_x-fb_error)<0.001){std::cout<<"機器人停止"<<"main_vecor: "<<main_vec<<"  "<<"imu_angle: "<<FB_imu<<"\n";}
    fb_error=FB_x;
    int r_place_x = -FB_XX*100;
    r_place_x= (r_place_x==0)?very_small:r_place_x;
    //  main_vec = (90-(atan2(150,r_place_x)*180/pi)/3)+1;

    //<<<<<<<<<<<<<<<<<<<<<HEAD  Outer dynamic window<<<<<<<<<<<<<<<<<<<<<
    //main_vec = (90-(atan2(150,r_place_x)*180/pi)/3)+1;//? 60前方(180度) 30 (90度) 90(270度)
    main_vec = 60;
    int mainRight=(main_vec+20>90)?90:main_vec+20;
    int mainLeft=(main_vec-20<30)?30:main_vec-20;
    //=====================
    int Boj_place[30][2]={0};
    int Ok_place[30][2];//最多儲存30個空間
    int line_cont_b=99,line_cont_ok=99,b_ok=1,continuedline_ok=0,continuedline_b=0;//b_ok=1可以走b_ok=0黑色
    int HowManyBoj=0,HowManyOk=0;
    #define close_oj_ignore 4
    //找到最大可走空間
    int more_ok_line=0,save_ok_line=0,right_ok=0;
    //=====================

    ScanInfo outer;
    outer.type = OUTER;
    outer.scan_main = main_vec;
    outer.scan_left = mainLeft;
    outer.scan_right = mainRight;
    RoutePlan(outer);

    int x1,x2,y1,y2;
    x1 = df_1_dis*cos(df_1*3*DEG2RAD);
    y1 = df_1_dis*sin(df_1*3*DEG2RAD);
    x2 = df_2_dis*cos(df_2*3*DEG2RAD);
    y2 = df_2_dis*sin(df_2*3*DEG2RAD);
    int hole_size = (int)(outer.max_vacancy_number==0)?0:hypot(x1-x2,y1-y2);
    //std::cout<<"洞口寬度 "<<hole_size<<"cm"<<std::endl;

    if(outer.max_vacancy_number==0){
        outer.scan_left=(main_vec+30+15>(90+15))?(90+15):main_vec+30+15;
        outer.scan_right=(main_vec-30-15<(30-15))?(30-15):main_vec-30-15;
        RoutePlan(outer);
    }
    //std::cout<<"max_vancynameer     "<<outer.max_vacancy_number<<std::endl;
    df_1 = outer.move_left;
    df_2 = outer.move_right;
    df_1_dis=outer.move_left_dis;
    df_2_dis=outer.move_right_dis;
    far_good_angle =(outer.max_vacancy_number==0)?90:outer.move_main;
    //far_good_angle=(far_good_angle+main_vec)/2;


    


    // std::cout<<"df_1: "<<df_1<<"  df_2:"<<df_2<<std::endl;
    // std::cout<<"df_1_dis: "<<df_1_dis<<"  df_2_dis:"<<df_2_dis<<std::endl;

    //>>>>>>>>>>>>>>>>>>>>>END   Outer dynamic window>>>>>>>>>>>>>>>>>>>>>
    //<<<<<<<<<<<<<<<<<<<<<HEAD  Inner dynamic window<<<<<<<<<<<<<<<<<<<<<
    line_cont_b=99;line_cont_ok=99;b_ok=1;continuedline_ok=0;continuedline_b=0;//b_ok=1可以走b_ok=0黑色
    HowManyBoj=0;HowManyOk=0;

    //mainRight=(int)(far_good_angle+20>90)?90:far_good_angle+20;
    //mainLeft=(int)(far_good_angle-20<30)?30:far_good_angle-20;
    //=======未始用outer window數值=========
    //far_good_angle=main_vec;
     //mainRight=(int)((main_vec+25)>(90+5))?(90+5):main_vec+25;
     //mainLeft=(int)((main_vec-25)<(30-5))?(30-5):main_vec-25;



    if(far_good_angle>(30-10) && far_good_angle<(90+10)){
        main_vec = far_good_angle;
    }
    int find_range = 25;
    mainRight=(main_vec+find_range>(90+5))?(90+5):main_vec+find_range;
    mainLeft=(main_vec-find_range<(30-5))?(30-5):main_vec-find_range;

    //mainRight=main_vec+find_range;
    //mainLeft=main_vec-find_range;
    //mainRight = 0;
    //mainLeft = 119;
    //====================================
    ScanInfo inner;
    inner.type = INNER;
    inner.scan_main = main_vec;
    inner.scan_left = mainLeft;
    inner.scan_right = mainRight;
    RoutePlan(inner);
    
    for(int i=0; i<30; i++){
        for(int j=0; j<2; j++){
            Boj_place[i][j]=0;
            Ok_place[i][j]=0;
        }
    }
    for(int i=0; i<30; i++){
        for(int j=0; j<2; j++){
            Boj_place[i][j] = inner.obstacle[i][j];
            Ok_place[i][j]  = inner.vacancy[i][j];
        }
    }
    dd_1 = inner.move_left;
    dd_2 = inner.move_right;
    dd_1_dis=inner.move_left_dis;
    dd_2_dis=inner.move_right_dis;
    good_angle = (int)(inner.max_vacancy_number==0)?90:(dd_1+dd_2)/2;
    HowManyOk = inner.vacancy_number;
    HowManyBoj = inner.obstacle_number;
    //std::cout<<"inner.move_right: "<<inner.move_right<<"  inner.move_left: "<<inner.move_left<<std::endl;

    more_ok_line=0;save_ok_line=0;right_ok=0;
    //printf("=====================================\n");
    printf("howmany_ok%d\n",HowManyOk);
    //print the obj and ok place
    for(int i=1 ; i<=HowManyOk ;i++){
        int ok_angle_text = (Ok_place[i][0]+Ok_place[i][1])/2;
        printf("ok=%d,angle=%d,dis=%d\t",i,ok_angle_text,env.blackdis[ok_angle_text]);
        std::cout<<Ok_place[i][1]<<"\t"<<Ok_place[i][0]<<"\n";
    }
    int BoxInFront=0;//0=no 1=on
    for(int i=1 ; i<=HowManyOk ;i++){
        save_ok_line=Ok_place[i][1]-Ok_place[i][0];
        if(save_ok_line>=more_ok_line){
            if((HowManyBoj==1)&&(save_ok_line-more_ok_line<=2)){
                BoxInFront=1;

            }
            more_ok_line=save_ok_line;
            right_ok=i;
        }
    }
    //解決局部最佳解 強制走某一個方向
    //=================
    static int intoflag=0,tem_right_ok=0,okokcont=0,two_ok_right=60,b_goodangle=60;
    int near_angle=999,test_angle;
    // two_ok_right=good_angle;
    // if(intoflag==1){
    //     //初始連接黑線資料顯示14個cont的時間 之後都不會進入flag
    //     //printf("qpqpqpqpqpqpqpqpqpqpqpqpqpqp\n");
    //     printf("局部最佳解\n");
    //     intoflag=(count-okokcont<14)?1:0;
    //     for(int i=1;i<=HowManyOk ;i++){
    //         dd_1=Ok_place[i][1];
    //         dd_2=Ok_place[i][0];
    //         test_angle=(int)(i==0)?90:(dd_1+dd_2)/2;//跟good_angle一樣?
    //         std::cout<<test_angle<<"\t"<<tem_right_ok<<"\n";
    //         if(near_angle>abs(test_angle-tem_right_ok)){//test_angle-tem_right_ok=0?
    //             near_angle=abs(test_angle-tem_right_ok);
    //             good_angle=test_angle;
    //             std::cout<<near_angle<<"/////"<<good_angle<<"\n";
    //         }
    //     }
    // }else if((abs((int)good_angle-b_goodangle)>28)&&(intoflag==0)){//good_angle-b_goodangle=0? 不會進入判斷式
    //     //std::cout<<"fuuuuuuuuuuuuk"<<std::endl;
    //     if(main_vec<60){//如果主向量大於60*3 當前靠場地右邊走
    //         for(int i=1;i<=HowManyOk ;i++){//正算? //選擇左邊的洞
    //             if(Ok_place[i][1]-Ok_place[i][0]>5){//如果可走範圍大於5條線
    //                 right_ok=i;
    //                 //printf("qqqqqqqqqqqqqqq\n");
    //                 printf("選擇左邊的洞\n");
    //                 intoflag=1;
    //                 okokcont=count;
    //                 dd_1=Ok_place[right_ok][1];
    //                 dd_2=Ok_place[right_ok][0];
    //                 good_angle=(int)(right_ok==0)?90:(dd_1+dd_2)/2;
    //                 tem_right_ok=good_angle;
    //                 break;
    //             }
    //             tem_right_ok=good_angle;
    //        }
    //     }else{//如果主向量小於60*3
    //         for(int i=HowManyOk;i>=1 ;i--){//反算? 　//選擇右邊的洞
    //             if(Ok_place[i][1]-Ok_place[i][0]>5){
    //                 right_ok=i;
    //                 //printf("ppppppppppppp\n");
    //                 printf("選擇右邊的洞\n");
    //                 intoflag=1;
    //                 okokcont=count;
    //                 dd_1=Ok_place[right_ok][1];
    //                 dd_2=Ok_place[right_ok][0];
    //                 good_angle=(int)(right_ok==0)?90:(dd_1+dd_2)/2;
    //                 tem_right_ok=good_angle;
    //                 break;
    //             }
    //             tem_right_ok=good_angle;
    //         }
    //     }
    //     // good_angle=b_goodangle;
    // }
    // //==========================
    // b_goodangle=two_ok_right;//two_ok_right 等於 good_angle

    // dd_1=Ok_place[right_ok][0];
    // dd_2=Ok_place[right_ok][1];
    // if(inner.move_main>(90+5)||inner.move_main<(30-5)){
    //     inner.move_main = (dd_1+dd_2)/2;
    // }
    // if(dd_1>0)dd_1_dis=env.blackdis[dd_1-1];
    // else dd_1_dis=env.blackdis[dd_1];
    // if(dd_2<119)dd_2_dis=env.blackdis[dd_2+1];
    // else dd_2_dis=env.blackdis[dd_2];


    dd_1_dis = inner.move_left_dis;
    dd_2_dis = inner.move_right_dis;
    //inner.move_left=dd_1;
    //inner.move_right=dd_2;
    //int x1,x2,y1,y2;
    x1 = dd_1_dis*cos(dd_1*3*DEG2RAD);
    y1 = dd_1_dis*sin(dd_1*3*DEG2RAD);
    x2 = dd_2_dis*cos(dd_2*3*DEG2RAD);
    y2 = dd_2_dis*sin(dd_2*3*DEG2RAD);
    hole_size = (int)(right_ok==0)?0:hypot(x1-x2,y1-y2);
    std::cout<<"洞口寬度 "<<hole_size<<"cm"<<std::endl;
    
    int obj_count_left = 0;
    int obj_dis_sum_left = 0;
    int obj_dis_average_left = 999;
    int obj_count_right = 0;
    int obj_dis_sum_right = 0;
    int obj_dis_average_right = 999;
    for(int i = 30-3; i<30+3; i++){
        if(env.blackdis[i]<90){
            obj_count_left++;
            obj_dis_sum_left = env.blackdis[i];
        }
    }
    if(obj_count_left>0){
        obj_dis_average_left = obj_dis_sum_left/obj_count_left;
    }
    for(int i = 90-3; i<90+3; i++){
        if(env.blackdis[i]<90){
            obj_count_right++;
            obj_dis_sum_right = env.blackdis[i];
        }
    }
    if(obj_count_right>0){
        obj_dis_average_right = obj_dis_sum_right/obj_count_right;
    }
    //good_angle=(int)(hole_size<40)?90:(dd_1+dd_2)/2;
    //good_angle=(int)(hole_size<40)?90:inner.move_main;
    if(obj_dis_average_left > obj_dis_average_right){
        good_angle=(int)(hole_size<55)?30:inner.move_main;
    }else{
        good_angle=(int)(hole_size<55)?90:inner.move_main;
    }
    if(inner.max_vacancy_number>0 && outer.max_vacancy_number==0 && hole_size>55){
        far_good_angle = good_angle;
    }
    //>>>>>>>>>>>>>>>>>>>>>END   Inner dynamic window>>>>>>>>>>>>>>>>>>>>>
    ////////////////////////////////////////////////////////////test for strategy

    int left_dis_sum = 0;
    int left_dis_average = 999;
    int left_average_line = 0;
    int right_dis_sum = 0;
    int right_dis_average = 999;
    int right_average_line = 0;
    int forward_dis_sum = 0;
    int forward_dis_average = 999;
    int forward_average_line = 0;
    int smallfront=999;
    static int b_forward_dis_sum=0;
    int robot_radius = 20;
    int left_closest_dis = 999;
    int right_closest_dis = 999;
    //=========左面 側邊障礙物平均距離計算========
    //for(int i= 25 ; i<=40 ; i++){//left_dis_average //左側(75-120度) 車頭180
    for(int i= good_angle-30 ; i<=good_angle ; i++){
        if(env.blackdis[i]<50&&env.blackdis[i]>0){ //如果距離小於50
            left_average_line++;
            left_dis_sum+=env.blackdis[i];
            if(env.blackdis[i]<left_closest_dis)left_closest_dis=env.blackdis[i];
        }
    }
    if(left_average_line > 13){ //at least 2 lines //如果大於三條線都有掃到障礙物 計算平均距離
        left_dis_average = left_dis_sum/(left_average_line);
        if(left_dis_average>0)printf("左側障礙物接近 %dcm\n",left_dis_average-robot_radius);
    }else{//距離重置
        left_dis_average=999;
    }
    //=======================================
    for(int i= good_angle ; i<=good_angle+30 ; i++){//right_dis_average //右側計算
        if(env.blackdis[i]<50&&env.blackdis[i]>0){
            right_average_line++;
            right_dis_sum+=env.blackdis[i];
            if(env.blackdis[i]<right_closest_dis)right_closest_dis=env.blackdis[i];
        }
    }
    if(right_average_line > 13){ //at least 2 lines
        right_dis_average = right_dis_sum/(right_average_line);
        if(right_dis_average>0)printf("右側障礙物接近 %dcm\n",right_dis_average-robot_radius);
    }else{
        right_dis_average=999;
    }
    //=======================================
    //============正面障礙物平均距離計算=========
    for(int i=55 ; i<=65 ;i++){
        if(env.blackdis[i]<200){
            forward_average_line++;
            forward_dis_sum+=env.blackdis[i];
        }
        smallfront=(smallfront<env.blackdis[i])?smallfront:env.blackdis[i];
    }
    if(forward_average_line > 3){ //at least 4 lines
        forward_dis_average = forward_dis_sum/(forward_average_line);
        if(forward_dis_average>0)printf("正面障礙物接近 %dcm\n",forward_dis_average-robot_radius);
    }else{
        forward_dis_average=999;
    }

    //=======================================
    int condition;//0::0引力  1::1special box_middle
    int dis_sum = 0;
    int red_dis_average = 0;
    int red_line_dangerous_dis =70;
    int dangerous_dis =(int)close_dis; //dangerous_dis = close_dis //60 speed (10) //54  speed (30,10)
    RedLine = 0; //紅線判斷flag 1left 2right

    left_average_line = 0;
    right_average_line = 0;
    //========左側紅線平均距離計算=========
    for(int i= 25 ; i<=40 ; i++){//left_dis_average
        if(env.reddis[i]<red_line_dangerous_dis){
            left_average_line++;
            dis_sum+=env.reddis[i];
        }
    }
    if(left_average_line > 3){ //at least 2 lines
        red_dis_average = dis_sum/(left_average_line);
        RedLine = 1;//red in left
        FB_XX=(red_dis_average!=0)?(red_dis_average-150)*0.01:FB_XX;
        if(red_dis_average>0)printf("左側紅線接近 %dcm\n", red_dis_average-robot_radius);
    }
    //===================================
    //========右側紅線平均距離計算=========
    dis_sum=0;
    for(int i= 80 ; i<=95 ; i++){//right_dis_average
        if(env.reddis[i]<red_line_dangerous_dis){
            right_average_line++;
            dis_sum+=env.reddis[i];
        }
    }
    if(right_average_line > 3){ //at least 2 lines
        red_dis_average = dis_sum/(right_average_line);//左邊紅線距離被右邊取代
        RedLine = 2;//red in right
        FB_XX=(red_dis_average!=0)?(150-red_dis_average)*0.01:FB_XX;
        if(red_dis_average>0)printf("右側紅線接近 %dcm\n", red_dis_average-robot_radius);
    }
    //===================================
    //=======引力斥力與中間相子case切換======
    std::cout<<"left_closest_dis  "<<left_closest_dis<<"    right_closest_dis  "<<right_closest_dis<<std::endl;
    //if(left_dis_average<45&&right_dis_average<45&&abs(left_closest_dis-right_closest_dis)<6){
    if((left_closest_dis+right_closest_dis)<90){
    //if(left_dis_average<45&&right_dis_average<45){//兩個箱子中間的case
        //std::cout<<"left_dis_average "<<left_dis_average<<"  right_dis_average  "<<right_dis_average<<std::endl;
        condition=box_in_between;
    }/*else if((smallfront <=42)&&(main_vec==40||main_vec==80)){
        condition = red_line;
    }*//*else if (intoflag==1||intoflag==2){
        condition=cannot_chose;
    }*/else{
        condition=N_S;
    }
    //condition=N_S;
    count+=1;
    static double before_error_x=0,I_error_x=0;
    static double Fx_pid,Fy_pid,PID_I_tem[20]={0} ;
    double error_x,D_error;
    static int pre_condition = 0;
   // condition=pid_control;
    af_angle = 0;
    v_af = 0;
    switch(condition){
    case N_S://人工勢場
        final_angle = Artificial_field(inner, outer, main_vec, close_dis, RedLine, red_dis_average, dangerous_dis, red_line_dangerous_dis, condition);
        printf("人工勢場 S/N========  ");
        break;
    case box_in_between:

        if(left_dis_average < right_dis_average){//走在兩個箱子正中間 如果遇到兩邊箱子不平行會撞到?
            //final_angle = 90-(90-(right_dis_average-left_dis_average))/3;//turn right
            final_angle = good_angle+3;
        }else if(left_dis_average > right_dis_average){
            //final_angle = 90-(90+(left_dis_average-right_dis_average))/3;//turn left
            final_angle = good_angle-3;
        }else{
            //final_angle = 60;//go forward
            final_angle = good_angle;
        }
        //final_angle = Artificial_field(inner, outer, main_vec, close_dis, RedLine, red_dis_average, dangerous_dis, red_line_dangerous_dis, condition);
        printf("middle box========  ");
        break;
    // case red_line:
    //     if(main_vec == 40){
    //         if(b_forward_dis_sum-smallfront<0){final_angle = 29;}
    //         else if(b_forward_dis_sum-smallfront>0){final_angle = 31;}
    //         else{final_angle=30;}
    //     }else if(main_vec == 80){
    //         if(b_forward_dis_sum-smallfront<0){final_angle = 91;}
    //         else if(b_forward_dis_sum-smallfront>0){final_angle = 89;}
    //         else{final_angle=90;}
    //     }
    //     printf("red front========  ");
    //     b_forward_dis_sum=smallfront;
    //     break;
    // case pid_control:
    //     before_error_x=Fx_pid;
    //     Fx_pid=0;Fy_pid=0;
    //     //============人工勢場=============
    //     for(int i=1 ; i<=HowManyBoj ;i++){ //repulsive force
    //         for(int j=Boj_place[i][0] ; j<=Boj_place[i][1] ; j++){
    //             if(env.blackdis[j]>=close_dis){
    //                 ignore_average_line++;
    //             }else{
    //                 dis_sum+=env.blackdis[j];
    //             }
    //         }
    //         if(Boj_place[i][1]-Boj_place[i][0]+1!=ignore_average_line){
    //             dis_average = dis_sum/(Boj_place[i][1]-Boj_place[i][0]+1-ignore_average_line);
    //         }else{
    //             dis_average=999;
    //         }
    //         ignore_average_line=0;
    //         Obj_angle = (Boj_place[i][0]+Boj_place[i][1])/2;
    //         printf("warn_B=%d,angle=%d\t,dis=%d\t",i,Obj_angle,dis_average);
    //         angle_average = (90-(Obj_angle))*3;//(90-(56+50)/2)*3=111
    //         Fx_pid += (dangerous_dis-dis_average)*cos(angle_average*deg2rad);//[53]->angle 53
    //         Fy_pid += (dangerous_dis-dis_average)*sin(angle_average*deg2rad);
    //         dis_sum=0;
    //     }
    //     //================================
    //     I_error_x=0;
    //     PID_I_tem[(int)count%20]=Fx_pid;
    //     for(int i=0;i<19;i++){
    //         I_error_x+=PID_I_tem[i];
    //     }
    //     I_error_x=I_error_x/20;
    //     D_error=Fx_pid-before_error_x;
    //     error_x =kd*(D_error)+ki*(I_error_x)+kp*Fx_pid;

    //     final_Fx = F_Max*cos((90-(60))*3*deg2rad)-error_x;
    //     final_Fy = F_Max*sin((90-(/*good_angle*/60))*3*deg2rad)-adjust_ojF*Fy;
    //     final_angle= 90-(atan2(final_Fy,final_Fx)*180/pi)/3;
    //     printf("\nkp=%lf,\nki=%lf\nkd%lf\nerror_x%lf\nI_error_x%lf\nD_error%lf\nP_error%lf\n",kp,ki,kd,error_x,I_error_x,D_error,Fx_pid);
    //     break;
    }
    /////////////////////////////////////////////////////////////////////////////////////
    printf("final_angle=%lf\n",final_angle);
    static int b_not_good_p=0;
    int max_speed = SPlanning_Velocity[2];
    int min_speed = SPlanning_Velocity[3];
    double speed = 0;
    if(v_fast>0)speed = max_speed*(v_fast/100);
    double slowdown_distance = speed*2;
    if (slowdown_distance<50)slowdown_distance=50;
    //=========每5count做一次速度規劃=======
    //if((int)count%5==1){
        // if(b_not_good_p==not_good_p){
        //     v_fast=v_fast+5;
        //     v_fast=(v_fast<100)?v_fast:100;
        // } else{
        //     v_fast= v_fast-5;
        //     v_fast=(v_fast>1)?v_fast:1;
        // }

        if((forward_dis_average>100)&&(HowManyBoj<=1)&&((40<main_vec)&&(main_vec<80))/*||(condition==box_in_between)*/){
            v_fast=v_fast+5;v_fast=(v_fast<100)?v_fast:100;
        }else{
            v_fast=80;
            //v_fast=(v_fast>1)?v_fast:1;
            int angle_min = (good_angle-25>0)?good_angle-25:0;
            int angle_max = (good_angle+25>118)?good_angle+25:118;
            for(int i=angle_min; i<angle_max; i++){
                if(env.blackdis[i]<slowdown_distance)v_fast=v_fast-5;
                if(v_fast<1)v_fast=1;
                if(env.blackdis[i]<close_dis&&condition!=box_in_between){
                    v_fast=1;
                }

                //if(abs(good_angle-i)<20&&env.blackdis[i]<80){
                //    v_fast=1;
                //}
            }
            if(condition==box_in_between){
                angle_min = final_angle-3;
                angle_max = final_angle+3;
                bool speed_up_flag = true;
                for(int i=angle_min; i<angle_max; i++){
                    if(env.blackdis[i]<slowdown_distance){
                        if(v_fast>1){
                            v_fast=v_fast-3;
                            if(v_fast<1)v_fast=1;
                        }
                        // if(env.blackdis[i]<70){
                        //     v_fast=1;
                        // }
                        speed_up_flag = false;
                    }
                }
                if(speed_up_flag==true){
                    v_fast=v_fast+5;v_fast=(v_fast<80)?v_fast:60;
                }
                //v_fast=80;

            }

        }
    //}
    //===============================

    pre_condition = condition;
    b_not_good_p=not_good_p;
    // v_fast=1;
    //if(not_good_p>20)v_fast = 0;
    v_fast=(avoid_go==0)?0:v_fast;
    FB_XX=(avoid_go==0)?0:FB_XX;
    motor_place(v_fast,final_angle,r_number);
    printf("v_fast=%d\n",v_fast);
    printf("ave_gray=%d\n",env.gray_ave);
    printf("count=%lf\n",count);
    printf("main_vec=%d\n",main_vec);
    printf("far_good_angle=%f\n",far_good_angle);
    printf("good_angle=%f\n",good_angle);
    printf("final_angle=%lf\n",final_angle);
    printf("FB_x=%lf\t,FB_y=%lf\t",FB_x,FB_y);
    printf("FB_xx=%lf\t,FB_err=%lf\t\n",FB_XX,fb_error);
    Pub_route();
    std::cout<<"=========================END=============================\n";
}
double FIRA_pathplan_class::Artificial_field(ScanInfo inner, ScanInfo outer, int main_vec, int close_dis, int RedLine, int red_dis_average, int dangerous_dis, int red_line_dangerous_dis, int condition){
    int Boj_place[30][2]={0};
    int Ok_place[30][2];//最多儲存30個空間
    int line_cont_b=99,line_cont_ok=99,b_ok=1,continuedline_ok=0,continuedline_b=0;//b_ok=1可以走b_ok=0黑色
    int HowManyBoj=0,HowManyOk=0;

    //=================
    //什麼情況main_vec =40 80?
    int ssm_r,ssm_l;
    // if(main_vec==40){ssm_l=20;ssm_r=95;}
    // else if(main_vec==80){ssm_l=25;ssm_r=100;}
    // else{ssm_l=25;ssm_r=95;}
    // int gain = 10;
    // if(main_vec==40){ssm_l=20+gain;ssm_r=95+gain;}
    // else if(main_vec==80){ssm_l=25-gain;ssm_r=100+gain;}
    // else{ssm_l=25-gain;ssm_r=95+gain;}
    //ssm_l = good_angle - 30;
    //ssm_r = good_angle + 30;
    ssm_l=0;ssm_r=117;
    //====================
    ScanInfo artificial_field;
    artificial_field.type = ARTIFICIAL_FIELD;
    artificial_field.scan_main = main_vec;
    artificial_field.scan_left = ssm_l;
    artificial_field.scan_right = ssm_r;
    RoutePlan(artificial_field);
    HowManyOk = artificial_field.vacancy_number;
    HowManyBoj = artificial_field.obstacle_number;
    for(int i=0; i<30; i++){
        for(int j=0; j<2; j++){
            Boj_place[i][j]=0;
            Ok_place[i][j]=0;
        }
    }
    for(int i=0; i<30; i++){
        for(int j=0; j<2; j++){
            Boj_place[i][j] = artificial_field.obstacle[i][j];
            Ok_place[i][j]  = artificial_field.vacancy[i][j];
        }
    }
    // ///////////////////////////////////////////////////ssssssssssssssss
    // std::cout<<"=========ssssssss============\n";
    // printf("dis[%d]=%d\n",60,env.blackdis[60]);
    // printf("howmany_black object = %d\n",HowManyBoj);

    // for(int i=1 ; i<=HowManyBoj ;i++){
    //     int Obj_angle_text = (Boj_place[i][0]+Boj_place[i][1])/2;
    //     printf("Boj=%d, angle=%d, dis=%d\t", i, Obj_angle_text, env.blackdis[Obj_angle_text]);
    //     std::cout<<Boj_place[i][1]<<"\t"<<Boj_place[i][0]<<"\n";
    // }
    // std::cout<<"=========ssssssss end========\n";

    int dis_sum=0;
    int dis_average = 0;
    int angle_average;
    double Fx,Fy = 0;
    int F_Max = (int)close_dis;
    int Obj_angle,ignore_average_line=0 ;

    double final_Fx = 0;
    double final_Fy = 0;
    double adjust_ojF=1;

    double closest_angle=0;
    double closest_dis=999;

    // for(int i=1 ; i<=artificial_field.vacancy_number ;i++){
    //     std::cout<<"obstacle: "<<i<<"    "<<artificial_field.obstacle[i][0]<<"    "<<artificial_field.obstacle[i][1]<<std::endl;
    // }
    for(int i=1 ; i<=artificial_field.vacancy_number ;i++){ //repulsive force 斥力
        for(int j=artificial_field.obstacle[i][0] ; j<=artificial_field.obstacle[i][1] ; j++){
            if(env.blackdis[j]<close_dis && env.blackdis[j]<closest_dis){
                if(j>0&&j<112){
                    if(abs(env.blackdis[j]-env.blackdis[j-1])<5 && abs(env.blackdis[j]-env.blackdis[j+1])<5){
                        //std::cout<<"dis  "<<env.blackdis[j]<<" angle  "<<j<<std::endl;
                        closest_dis = env.blackdis[j];
                        closest_angle = j;
                    }
                }
            }
        }
    }
    std::cout<<"closest_angle  "<<closest_angle<<"  closest_dis  "<<closest_dis<<std::endl;
    dis_average = closest_dis;
    if(dis_average<=dangerous_dis){//dangerous_dis = close_dis //60 speed (10) //54  speed (30,10)
        Obj_angle = closest_angle;
        //Obj_angle = (artificial_field.obstacle[i][0]+artificial_field.obstacle[i][1])/2;//遠離障礙物平均角度 （是否需改成最近位置角度?)
        //printf("人工勢場 障礙物靠近 warn_B=%d,angle=%d\t,dis=%d\t",i,Obj_angle,dis_average);
        //angle_average = (90-(Obj_angle))*3;//(90-(56+50)/2)*3=111
        angle_average = (90-(closest_angle))*3;
        if(((Obj_angle<30)||(Obj_angle>90))/*&&((main_vec!=80)&&(main_vec!=40))*/){//障礙物角度大於左右90度 人工勢場*0.8
            //Fx += (dangerous_dis-dis_average)*cos(angle_average*deg2rad)*0.8;//[53]->angle 53  //角度乘以0.8？
            //Fy += (dangerous_dis-dis_average)*sin(angle_average*deg2rad)*0.8;
            Fx += (dangerous_dis-dis_average)*cos(angle_average*deg2rad);//[53]->angle 53  //角度乘以0.8？
            Fy += (dangerous_dis-dis_average)*sin(angle_average*deg2rad);
        }else{
            Fx += (dangerous_dis-dis_average)*cos(angle_average*deg2rad);//[53]->angle 53
            Fy += (dangerous_dis-dis_average)*sin(angle_average*deg2rad);
        }
        int find_angle_range = 30;
        int find_angle_max = closest_angle+find_angle_range;
        int find_angle_min = closest_angle-find_angle_range;
        if(find_angle_max>119)find_angle_max = 119;
        if(find_angle_min<0)find_angle_min = 0;

        af_angle = 90-(atan2(-Fy,-Fx)*180/pi)/3;
        if(af_angle>119)af_angle=af_angle-119;
        v_af = hypot(Fy,Fx);
        printf("Fx=%lf,Fy=%lf\n",Fx,Fy);//輸出斥力大小
    }
    dis_sum=0;
    dis_average=999;
    if(RedLine!=0){//有偵測到紅線
        //printf("紅線靠近 RedLine=%d\tred_dis_average=%d\n",RedLine,red_dis_average);
        if(RedLine==2){//左邊紅線
            int closest_distance = 999;
            for(int i= 60 ; i<=60+45 ; i++){//right_dis_average //右側計算
                if(env.blackdis[i]<closest_distance){
                    closest_distance = env.blackdis[i];
                }
            }
            if(closest_distance<60){
                red_dis_average=0;
            }else{
                red_dis_average=red_line_dangerous_dis-red_dis_average;
            }
        }else{//右邊紅線
            int closest_distance = 999;
            for(int i= 60-45 ; i<=60; i++){//right_dis_average //左側計算
                if(env.blackdis[i]<closest_distance){
                    closest_distance = env.blackdis[i];
                }
            }
            if(closest_distance<60){
                red_dis_average=0;
            }else{
                red_dis_average=red_dis_average-red_line_dangerous_dis;
            }
        }
    }
    //==============================
    int main_angle;
    if(outer.move_left>=inner.move_left&&outer.move_right<=inner.move_right){
       main_angle = far_good_angle;
    }else{
        main_angle = good_angle;
    }
    //if(closest_dis<80){
    final_Fx = F_Max*cos((90-(main_angle))*3*deg2rad)-adjust_ojF*Fx-(0.2*red_dis_average);
    final_Fy = F_Max*sin((90-(main_angle))*3*deg2rad)-adjust_ojF*Fy;
    final_angle= 90-(atan2(final_Fy,final_Fx)*180/pi)/3;

    double shift_flag = false;
    for(int i=(60-15);i<(60+15);i++){
        if(env.blackdis[i]<50)shift_flag=true;
    }

    if(final_angle>119)final_angle=final_angle-119;
    if(final_angle<0)final_angle=final_angle+119;
    int angle_range=3;
    if(condition==box_in_between){
        //std::cout<<"final_angle  "<<final_angle<<"  af_angle  "<<af_angle<<std::endl;
        
        // if(abs(good_angle-af_angle)>(30-5)){
        //     if(good_angle>af_angle)final_angle = good_angle-3;
        //     if(good_angle<af_angle)final_angle = good_angle+3;
        // }
        if(abs(final_angle-af_angle)<(30-angle_range) && abs(good_angle-af_angle)>30){

            if(final_angle>af_angle)final_angle=af_angle+30-3;
            if(final_angle<af_angle)final_angle=af_angle-30+3;
        }
    }
    
    if(final_angle>(90-5) && shift_flag==true){
        final_angle=90;
    }
    if(final_angle<(30+5) && shift_flag==true){
        final_angle=30;
    }

    //if(final_angle>90+3)final_angle=90+3;
    //if(final_angle<30-3)final_angle=30-3;

    //==============================
    return final_angle;
}
void FIRA_pathplan_class::Pub_route(){
    vision::avoid msg;
    msg.df_1 = df_1;
    msg.df_2 = df_2;
    msg.df_1_dis = df_1_dis;
    msg.df_2_dis = df_2_dis;
    msg.far_good_angle = far_good_angle;
    msg.dd_1 = dd_1;
    msg.dd_2 = dd_2;
    msg.dd_1_dis = dd_1_dis;
    msg.dd_2_dis = dd_2_dis;
    msg.good_angle = good_angle;
    msg.final_angle = final_angle;
    msg.af_angle = af_angle;
    msg.v_fast=v_fast;
    msg.v_af=v_af;
    route_pub.publish(msg);
}
//=========================避障挑戰賽結束=================================
void FIRA_pathplan_class::strategy_Halt(int Robot_index){
    env.home[Robot_index].v_x = 0;
    env.home[Robot_index].v_y = 0;
    env.home[Robot_index].v_yaw = 0;
}
void FIRA_pathplan_class::motor_place(int v,double num,int r_num){
    //num_change=num*3-270;//
    //go_where_x=-v*sin(num_change*deg2rad);// 50               70
    //go_where_y=v*cos(num_change*deg2rad);

    double angle = (int)(num*3+180)%360;//將策略的角度轉換回馬達角度 //num 60車頭角 30左邊 90右邊
    // for(int i = 0; i<120; i++){
    //     std::cout<<i<<"    "<<(int)(i*3+180) % 360<<std::endl;
    // }
    double vx = -v*sin(angle*deg2rad);
    double vy =  v*cos(angle*deg2rad);
    //std::cout<<vx<<"  "<<vy<<std::endl;
    env.home[r_num].v_x =  vx;
    env.home[r_num].v_y =  vy;
    env.home[r_num].v_yaw = 0;

}
double FIRA_pathplan_class::vecAngle(Vector2d a,Vector2d b){

    a.normalize();
    b.normalize();
    double c = a.dot(b);
    double rad = acos(c);

    Vector3d a3d = Vector3d(a(0),a(1),0);
    Vector3d b3d = Vector3d(b(0),b(1),0);

    Vector3d tCross = a3d.cross(b3d);

    if(tCross(2) < 0)       return (-1)*rad*rad2deg;
    else                    return rad*rad2deg;
}

//==========for ROS special===============//

void FIRA_pathplan_class::loadParam(ros::NodeHandle *n){
    n->setParam("/AvoidChallenge/GoodAngle",good_angle);
    n->setParam("/AvoidChallenge/FinalAngle",final_angle);
    n->setParam("/AvoidChallenge/dd_1",dd_1);
    n->setParam("/AvoidChallenge/dd_2",dd_2);
    n->setParam("/AvoidChallenge/far_good_angle",far_good_angle);
    n->setParam("/AvoidChallenge/df_1",df_1);
    n->setParam("/AvoidChallenge/df_2",df_2);
    //==============================
    n->getParam("/AvoidChallenge/kp",kp);
    n->getParam("/AvoidChallenge/ki",ki);
    n->getParam("/AvoidChallenge/kd",kd);
    n->getParam("/AvoidChallenge/avoid_go",avoid_go);


    if(n->getParam("/AvoidChallenge/Distant", Distant)){
        // for(int i=0;i<1;i++)
        //     std::cout<< "param AvoidChallenge Distant["<< i << "]=" << Distant[i] << std::endl;
        // std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/AvoidChallenge/SPlanning_Velocity", SPlanning_Velocity)){
//         for(int i=0;i<8;i++)
//             std::cout<< "param SPlanning_Velocity["<< i << "]=" << SPlanning_Velocity[i] << std::endl;
//     std::cout << "====================================" << std::endl;
     }
}
