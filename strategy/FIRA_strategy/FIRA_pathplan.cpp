#include "FIRA_pathplan.h"
#include "math.h"
#include "time.h"

//=========Environment init=============//
FIRA_pathplan_class::FIRA_pathplan_class(){
    opponent = false;
    v_fast = 50;
    //回傳策略線向量給影像
    tovision  = nh.advertise<strategy::strategylook>("/vision/strategy_line",1);
    route_pub = nh.advertise<vision::avoid>("/avoid/route",1);
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
        strategy_Halt(robotIndex);
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
    if(not_good_p>10){
        v_fast=0;
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
    int size_ignore = 1;
    if(THIS.type == ARTIFICIAL_FIELD)size_ignore=1;
    bool is_vacancy=true;

    bool obstacle_flag=0;//b_ok=1可以走b_ok=0黑色
    int obstacle[30][2]={0};
    int obstacle_number=0;
    int obstacle_size=99;

    bool vacancy_flag=0;
    int vacancy[30][2];//最多儲存30個空間
    int vacancy_number=0;
    int vacancy_size=99;

    for(int i= THIS.scan_left ; i<=THIS.scan_right ; i++){
        //若黑線小於far dis(250) 且黑線大於中層的距離 或者紅線小於最遠距離 b_ok = false
        if(THIS.type == OUTER){
            is_vacancy=((env.blackdis[i] <= far_dis)&&(env.blackdis[i] >= halfclose_dis)||(env.reddis[i]<=far_dis))?false:true;
        }else if(THIS.type == INNER){
            is_vacancy=((env.blackdis[i] <= halfclose_dis)||(env.reddis[i]<=250))?false:true;
        }else if(THIS.type == ARTIFICIAL_FIELD){
            is_vacancy=(env.blackdis[i] <= close_dis+20)?false:true;
        }else{
            is_vacancy=(env.blackdis[i] <= close_dis+20)?false:true;
        }
        is_vacancy=(env.blackdis[i] <= close_dis+20)?false:true;
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
    int max_size=0;
    int max_vacancy_number=0;
    for(int i=1 ; i<=vacancy_number ;i++){
        size=vacancy[i][1]-vacancy[i][0];
        if(size>max_size){
            max_size=size;
            max_vacancy_number=i;
        }
    }
    THIS.max_vacancy_number = max_vacancy_number;
    THIS.move_left  = vacancy[max_vacancy_number][0];
    THIS.move_right = vacancy[max_vacancy_number][1];
    //std::cout<<"move_right: "<<THIS.move_right<<"  move_left: "<<THIS.move_left<<std::endl;
}
void FIRA_pathplan_class::Pub_route(){
    vision::avoid msg;
    msg.df_1 = df_1;
    msg.df_2 = df_2;
    msg.far_good_angle = far_good_angle;
    msg.dd_1 = dd_1;
    msg.dd_2 = dd_2;
    msg.good_angle = good_angle;
    msg.final_angle = final_angle; 
    msg.af_angle = af_angle; 
    route_pub.publish(msg);
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
    main_vec = (90-(atan2(150,r_place_x)*180/pi)/3)+1;//? 60前方(180度) 30 (90度) 90(270度) 
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
    df_1 = outer.move_right;
    df_2 = outer.move_left;
    far_good_angle =(outer.max_vacancy_number==0)?90:(df_1+df_2)/2;
    far_good_angle=(far_good_angle+main_vec)/2;
    // std::cout<<"df_1: "<<df_1<<"  df_2:"<<df_2<<std::endl;

    //>>>>>>>>>>>>>>>>>>>>>END   Outer dynamic window>>>>>>>>>>>>>>>>>>>>>
    //<<<<<<<<<<<<<<<<<<<<<HEAD  Inner dynamic window<<<<<<<<<<<<<<<<<<<<<
    line_cont_b=99;line_cont_ok=99;b_ok=1;continuedline_ok=0;continuedline_b=0;//b_ok=1可以走b_ok=0黑色
    HowManyBoj=0;HowManyOk=0;

    mainRight=(int)(far_good_angle+20>90)?90:far_good_angle+20;
    mainLeft=(int)(far_good_angle-20<30)?30:far_good_angle-20;
    //=======未始用outer window數值=========
    far_good_angle=main_vec;
    mainRight=(int)((main_vec+25)>90)?90:main_vec+25;
    mainLeft=(int)((main_vec-25)<30)?30:main_vec-25;
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
    good_angle = (int)(inner.max_vacancy_number==0)?90:(dd_1+dd_2)/2;
    HowManyOk = inner.vacancy_number;
    HowManyBoj = inner.obstacle_number;
    //std::cout<<"inner.move_right: "<<inner.move_right<<"  inner.move_left: "<<inner.move_left<<std::endl;
    
    more_ok_line=0;save_ok_line=0;right_ok=0;
    printf("=====================================\nhowmany_ok%d\n",HowManyOk);
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
    two_ok_right=good_angle;
    if(intoflag==1){
        //初始連接黑線資料顯示14個cont的時間 之後都不會進入flag
        printf("qpqpqpqpqpqpqpqpqpqpqpqpqpqp\n");
        intoflag=(count-okokcont<14)?1:0;
        for(int i=1;i<=HowManyOk ;i++){
            dd_1=Ok_place[i][1];
            dd_2=Ok_place[i][0];
            test_angle=(int)(i==0)?90:(dd_1+dd_2)/2;//跟good_angle一樣?
            std::cout<<test_angle<<"\t"<<tem_right_ok<<"\n";
            if(near_angle>abs(test_angle-tem_right_ok)){//test_angle-tem_right_ok=0?
                near_angle=abs(test_angle-tem_right_ok);
                good_angle=test_angle;
                std::cout<<near_angle<<"/////"<<good_angle<<"\n";
            }
        }
    }else if((abs((int)good_angle-b_goodangle)>28)&&(intoflag==0)){//good_angle-b_goodangle=0? 不會進入判斷式
        //std::cout<<"fuuuuuuuuuuuuk"<<std::endl;
        if(main_vec<60){//如果主向量大於60*3 
            for(int i=1;i<=HowManyOk ;i++){//正算?
                if(Ok_place[i][1]-Ok_place[i][0]>5){//如果可走範圍大於5條線
                    right_ok=i;
                    printf("qqqqqqqqqqqqqqq\n");
                    intoflag=1;
                    okokcont=count;
                    dd_1=Ok_place[right_ok][1];
                    dd_2=Ok_place[right_ok][0];
                    good_angle=(int)(right_ok==0)?90:(dd_1+dd_2)/2;
                    tem_right_ok=good_angle;
                    break;
                }
                tem_right_ok=good_angle;
           }
        }else{//如果主向量小於60*3 
            for(int i=HowManyOk;i>=1 ;i--){//反算?
                if(Ok_place[i][1]-Ok_place[i][0]>5){
                    right_ok=i;
                    printf("ppppppppppppp\n");
                    intoflag=1;
                    okokcont=count;
                    dd_1=Ok_place[right_ok][1];
                    dd_2=Ok_place[right_ok][0];
                    good_angle=(int)(right_ok==0)?90:(dd_1+dd_2)/2;
                    tem_right_ok=good_angle;
                    break;
                }
                tem_right_ok=good_angle;
            }
        }
        // good_angle=b_goodangle;
    }
    //==========================
    b_goodangle=two_ok_right;//two_ok_right 等於 good_angle

    dd_1=Ok_place[right_ok][1];
    dd_2=Ok_place[right_ok][0];
    good_angle=(int)(right_ok==0)?90:(dd_1+dd_2)/2;
    //>>>>>>>>>>>>>>>>>>>>>END   Inner dynamic window>>>>>>>>>>>>>>>>>>>>>

    ///////////////////////////////////////////////////s
    //人工勢場使用
    line_cont_b=99;line_cont_ok=99;b_ok=1;continuedline_ok=0;continuedline_b=0;//b_ok=1可以走b_ok=0黑色
    HowManyBoj=0;HowManyOk=0;
    int ssm_r,ssm_l;
    //=================
    //什麼情況main_vec =40 80?
    //if(main_vec==40){ssm_r=20;ssm_l=95;}
    //else if(main_vec==80){ssm_r=25;ssm_l=100;}
    //else{ssm_r=25;ssm_l=95;}
    ssm_l = good_angle - 30;
    ssm_r = good_angle + 30; 
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

    ////////////////////////////////////////////////////////////test for strategy
    
    int left_dis_sum = 0;
    int left_dis_average = 0;
    int left_average_line = 0;
    int right_dis_sum = 0;
    int right_dis_average = 0;
    int right_average_line = 0;
    int forward_dis_sum = 0;
    int forward_dis_average = 0;
    int forward_average_line = 0;
    int smallfront=999;
    static int b_forward_dis_sum=0;
    int robot_radius = 30;
    //=========左面 側邊障礙物平均距離計算========
    for(int i= 25 ; i<=40 ; i++){//left_dis_average //左側(75-120度) 車頭180
        if(env.blackdis[i]<50){ //如果距離小於50
            left_average_line++;
            left_dis_sum+=env.blackdis[i];
        }
    }
    if(left_average_line > 3){ //at least 2 lines //如果大於三條線都有掃到障礙物 計算平均距離
        left_dis_average = left_dis_sum/(left_average_line);
        if(left_dis_average>0)printf("左側障礙物接近 %dcm\n",left_dis_average-robot_radius);
    }else{//距離重置
        left_dis_average=999;
    }
    //=======================================
    //========右面 側邊障礙物平均距離計算=========
    for(int i= 80 ; i<=95 ; i++){//right_dis_average //右側計算
        if(env.blackdis[i]<50){
            right_average_line++;
            right_dis_sum+=env.blackdis[i];
        }
    }
    if(right_average_line > 3){ //at least 2 lines
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
    if(left_dis_average+right_dis_average<=90){//兩個箱子中間的case
        condition=box_in_between;
    }/*else if((smallfront <=42)&&(main_vec==40||main_vec==80)){
        condition = red_line;
    }*//*else if (intoflag==1||intoflag==2){
        condition=cannot_chose;
    }*/else{
        condition=N_S;
    }

    count+=1;
    dis_sum=0;
    int dis_average = 0;
    int angle_average;
    double Fx,Fy = 0;
    int F_Max = (int)close_dis;
    int Obj_angle,ignore_average_line=0 ;

    int small_dis_box=999;
    double final_Fx = 0;
    double final_Fy = 0;

    static double before_error_x=0,I_error_x=0;
    static double Fx_pid,Fy_pid,PID_I_tem[20]={0} ;
    double error_x,D_error;
    double adjust_ojF=1;
   // condition=pid_control;
    switch(condition){
    case N_S://人工勢場 
        for(int i=1 ; i<=HowManyBoj ;i++){ //repulsive force 斥力
            for(int j=Boj_place[i][0] ; j<=Boj_place[i][1] ; j++){
                if(env.blackdis[j]>=close_dis){//障礙物線大於 判斷距離
                    ignore_average_line++;//忽略線++
                }else{
                    dis_sum+=env.blackdis[j];//計算此障礙物距離總和
                }
                small_dis_box=(small_dis_box<env.blackdis[j])?small_dis_box:env.blackdis[j];//找出最小障礙物距離
            }
            //==============未使用?==============
            if(Boj_place[i][1]-Boj_place[i][0]+1!=ignore_average_line){//若障礙物不是整個被忽略
                dis_average = dis_sum/(Boj_place[i][1]-Boj_place[i][0]+1-ignore_average_line);//計算平均障礙物距離
            }else{
                dis_average=999;
            }
            //==================================
            dis_average=small_dis_box;
            //dis_average=(dis_average<33)?0:dis_average;
            //======不需更新=======
            //small_dis_box=999;
            //ignore_average_line=0;
            //===================

            if(dis_average<=dangerous_dis){//dangerous_dis = close_dis //60 speed (10) //54  speed (30,10)
                Obj_angle = (Boj_place[i][0]+Boj_place[i][1])/2;//遠離障礙物平均角度 （是否需改成最近位置角度?)
                printf("人工勢場 障礙物靠近 warn_B=%d,angle=%d\t,dis=%d\t",i,Obj_angle,dis_average);
                angle_average = (90-(Obj_angle))*3;//(90-(56+50)/2)*3=111
                if(((Obj_angle<30)||(Obj_angle>90))/*&&((main_vec!=80)&&(main_vec!=40))*/){//障礙物角度大於左右90度 人工勢場*0.8
                    Fx += (dangerous_dis-dis_average)*cos(angle_average*deg2rad)*0.8;//[53]->angle 53  //角度乘以0.8？
                    Fy += (dangerous_dis-dis_average)*sin(angle_average*deg2rad)*0.8;
                }else{
                    Fx += (dangerous_dis-dis_average)*cos(angle_average*deg2rad);//[53]->angle 53
                    Fy += (dangerous_dis-dis_average)*sin(angle_average*deg2rad);
                }
                af_angle = 90-(atan2(-Fy,-Fx)*180/pi)/3;
                printf("Fx=%lf,Fy=%lf\n",Fx,Fy);//輸出斥力大小
            }
            dis_sum=0;
            dis_average=999;
        }
        if(RedLine!=0){//有偵測到紅線
            //printf("紅線靠近 RedLine=%d\tred_dis_average=%d\n",RedLine,red_dis_average);
            if(RedLine==2){
                red_dis_average=red_line_dangerous_dis-red_dis_average;
            }else{
                red_dis_average=red_dis_average-red_line_dangerous_dis;
            }
        }
        //==============================
        final_Fx = F_Max*cos((90-(good_angle))*3*deg2rad)-adjust_ojF*Fx-(0.2*red_dis_average);
        final_Fy = F_Max*sin((90-(good_angle))*3*deg2rad)-adjust_ojF*Fy;
        final_angle= 90-(atan2(final_Fy,final_Fx)*180/pi)/3;
        printf("人工勢場 S/N========  ");
        //==============================
        break;
    case box_in_between:
        if(left_dis_average < right_dis_average){//走在兩個箱子正中間 如果遇到兩邊箱子不平行會撞到?
            final_angle = 90-(90-(right_dis_average-left_dis_average))/3;//turn right
        }else if(left_dis_average > right_dis_average){
            final_angle = 90-(90+(left_dis_average-right_dis_average))/3;//turn left
        }else{
            final_angle = 60;//go forward
        }
        printf("middle box========  ");
        break;
    case red_line:
        if(main_vec == 40){
            if(b_forward_dis_sum-smallfront<0){final_angle = 29;}
            else if(b_forward_dis_sum-smallfront>0){final_angle = 31;}
            else{final_angle=30;}
        }else if(main_vec == 80){
            if(b_forward_dis_sum-smallfront<0){final_angle = 91;}
            else if(b_forward_dis_sum-smallfront>0){final_angle = 89;}
            else{final_angle=90;}
        }
        printf("red front========  ");
        b_forward_dis_sum=smallfront;
        break;
    case pid_control:
        before_error_x=Fx_pid;
        Fx_pid=0;Fy_pid=0;
        //============人工勢場=============
        for(int i=1 ; i<=HowManyBoj ;i++){ //repulsive force
            for(int j=Boj_place[i][0] ; j<=Boj_place[i][1] ; j++){
                if(env.blackdis[j]>=close_dis){
                    ignore_average_line++;
                }else{
                    dis_sum+=env.blackdis[j];
                }
            }
            if(Boj_place[i][1]-Boj_place[i][0]+1!=ignore_average_line){
                dis_average = dis_sum/(Boj_place[i][1]-Boj_place[i][0]+1-ignore_average_line);
            }else{
                dis_average=999;
            }
            ignore_average_line=0;
            Obj_angle = (Boj_place[i][0]+Boj_place[i][1])/2;
            printf("warn_B=%d,angle=%d\t,dis=%d\t",i,Obj_angle,dis_average);
            angle_average = (90-(Obj_angle))*3;//(90-(56+50)/2)*3=111
            Fx_pid += (dangerous_dis-dis_average)*cos(angle_average*deg2rad);//[53]->angle 53
            Fy_pid += (dangerous_dis-dis_average)*sin(angle_average*deg2rad);
            dis_sum=0;
        }
        //================================
        I_error_x=0;
        PID_I_tem[(int)count%20]=Fx_pid;
        for(int i=0;i<19;i++){
            I_error_x+=PID_I_tem[i];
        }
        I_error_x=I_error_x/20;
        D_error=Fx_pid-before_error_x;
        error_x =kd*(D_error)+ki*(I_error_x)+kp*Fx_pid;

        final_Fx = F_Max*cos((90-(60))*3*deg2rad)-error_x;
        final_Fy = F_Max*sin((90-(/*good_angle*/60))*3*deg2rad)-adjust_ojF*Fy;
        final_angle= 90-(atan2(final_Fy,final_Fx)*180/pi)/3;
        printf("\nkp=%lf,\nki=%lf\nkd%lf\nerror_x%lf\nI_error_x%lf\nD_error%lf\nP_error%lf\n",kp,ki,kd,error_x,I_error_x,D_error,Fx_pid);
        break;

    }
    /////////////////////////////////////////////////////////////////////////////////////
    printf("final_angle=%lf\n",final_angle);
    static int b_not_good_p=0;
    //=========每5秒做一次速度規劃=======
    if((int)count%5==1){
        if(b_not_good_p==not_good_p){ 
            v_fast=v_fast+10;
            v_fast=(v_fast<100)?v_fast:100;
        } else{ 
            v_fast= v_fast-10;
            v_fast=(v_fast>1)?v_fast:1;   
        }

        if((forward_dis_average>100)&&(HowManyBoj<=1)&&((40<main_vec)&&(main_vec<80))/*||(condition==box_in_between)*/){
            v_fast=v_fast+30;v_fast=(v_fast<100)?v_fast:100;
        }else{
            v_fast=1;
            v_fast=(v_fast>1)?v_fast:1;
            if(condition==box_in_between){
                v_fast=1;
            }
        }
    }
    //===============================
    Pub_route();

    b_not_good_p=not_good_p;
    // v_fast=1;
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
    std::cout<<"=========================END=============================\n";
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
   
    // if(v!=0){
    //     env.home[r_num].v_x =-5.0;
    //     env.home[r_num].v_y =0.0;
    //     env.home[r_num].v_yaw = 0.0;
    // }else{
    //     env.home[r_num].v_x =0.0;
    //     env.home[r_num].v_y =0.0;
    //     env.home[r_num].v_yaw = 0.0;
    // }

//    if((9<env.home[r_num].FB_yaw)&&(env.home[r_num].FB_yaw<180))
//    {env.home[r_num].v_yaw = -2;}
//    else if((180<env.home[r_num].FB_yaw)&&(env.home[r_num].FB_yaw<351)){env.home[r_num].v_yaw = 2;}
//    else{env.home[r_num].v_yaw=0;}
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
    /*
    if(n->getParam("/AvoidChallenge/Line", ScanLine)){
        //        for(int i=0;i<1;i++)
        //            std::cout<< "param Attack_Strategy["<< i << "]=" << Attack_Strategy[i] << std::endl;
        //    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/AvoidChallenge/ChooseSide", ChooseSide)){
        //        for(int i=0;i<1;i++)
        //            std::cout<< "param Attack_Strategy["<< i << "]=" << Attack_Strategy[i] << std::endl;
        //    std::cout << "====================================" << std::endl;
    }
   //==============================
    if(n->getParam("/FIRA/Attack_Strategy", Attack_Strategy)){
        //        for(int i=0;i<1;i++)
        //            std::cout<< "param Attack_Strategy["<< i << "]=" << Attack_Strategy[i] << std::endl;
        //    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA/Chase_Strategy", Chase_Strategy)){
        //        for(int i=0;i<1;i++)
        //            std::cout<< "param Chase_Strategy["<< i << "]=" << Chase_Strategy[i] << std::endl;
        //    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA/Zone_Attack", Zone_Attack)){
        //        for(int i=0;i<2;i++)
        //            std::cout<< "param Zone_Attack["<< i << "]=" << Zone_Attack[i] << std::endl;
        //    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA/TypeS_Attack", TypeS_Attack)){
        //       for(int i=0;i<2;i++)
        //           std::cout<< "param TypeS_Attack["<< i << "]=" << TypeS_Attack[i] << std::endl;
        //   std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA/TypeU_Attack", TypeU_Attack)){
        //       for(int i=0;i<8;i++)
        //           std::cout<< "param TypeU_Attack["<< i << "]=" << TypeU_Attack[i] << std::endl;
        //   std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA/SideSpeedUp", SideSpeedUp)){
        //       for(int i=0;i<5;i++)
        //           std::cout<< "param SideSpeedUp["<< i << "]=" << SideSpeedUp[i] << std::endl;
        //   std::cout << "====================================" << std::endl;
    }
    */
}
