#include <stdio.h>
#include <iostream>
#include <math.h>
#include "../common/Env.h"
#include <ros/ros.h>
#include <vector>
#include "strategy/strategylook.h"
#include "vision/avoid.h"

//攻擊防守,防守策略？
//沒有持球攻擊防守的策略

#define BallRadius 0.1055
#define CarRadius 0.34

#define N_S 0
#define box_in_between 1
#define red_line 2
#define pid_control 3
#define cannot_chose 4
struct ScanInfo
{
    ScanInfo();
    int type;
    int scan_main;
    int scan_right;
    int scan_left;
    int move_main;
    int move_right;
    int move_left;
    int obstacle[30][2];
    int vacancy[30][2];
    int vacancy_number;
    int obstacle_number;
    int max_vacancy_number;
    int move_right_dis;
    int move_left_dis;
};
class FIRA_pathplan_class
{
private:
    //start---simulator---
    bool opponent;
    Environment env;
    int mTeam;
    //    int roleAry[PLAYERS_PER_SIDE];
    //end  ---simulator---


    //start---utility---
    ros::NodeHandle nh;
    double head2Obj(Vector3D robot,Vector3D dst,double robotRot);
    double vecAngle(Vector2d a,Vector2d b);
    void RoutePlan(ScanInfo &THIS);
    void connected();
    
    //end---utility---


public:

    //start---simulator---
    void setOpponent(bool iBool){opponent = iBool;}
    void setTeam(int team){mTeam = team;}
    void setEnv(Environment iEnv);
    void shoot_init(){shoot = 0;}
    int getShoot(){return shoot;}
    Environment* getEnv(){return &env;}
    //end  ---simulator---

    FIRA_pathplan_class();
    void personalStrategy(int, int);
    //    void strategy();

    void strategy_dst(double destination_x,double destination_y);
    void strategy_dst_head2ball(double destination_x,double destination_y);

    //    void strategy_dontboom();

    void motor_place(int,double,int);
    void strategy_goalkeeper(int);
    void strategy_head2ball(int);

    //--------------各種strategy case-------------------
    void strategy_Goalkeeper(int);
    void strategy_Attack(int);
    void strategy_typeS_Attack(int);
    void strategy_Zone_Attack(int);
    void strategy_typeU_Attack(int);
    void strategy_Dorsad_Attack(int);
    void strategy_Support(int);
    void strategy_Halt(int);
    void strategy_PenaltyKick(int);
    void strategy_ThrowIn(int);
    void strategy_CornerKick(int);
    void strategy_AvoidBarrier(int);
    void strategy_Chase(int);
    void strategy_KO5_Chase(int);
    void strategy_KO5_Attack(int);
    void strategy_SideSpeedUp(int);
    //--------------------------------------------------

    //--------------各種role case-------------------
    void role_Play();
    void role_Halt();
    void role_FreeKick();
    void role_PenaltyKick();
    void role_FreeBall();
    void role_ThrowIn();
    void role_CornerKick();
    void role_GoalKick();
    void role_AvoidBarrier();
    //--------------------------------------------------

    //    void teamStrategy();

    //==========for ROS special===============//
    std::string teamColor;
    double beta_const = 0.9;
    double long_rush_alpha;
    double long_rush_dis_br;
    double long_rush_speed_const;
    double short_rush_dis_dr;
    double short_rush_alpha;
    double short_rush_dis_br;
    double short_rush_speed_const;
    double close_ball_dis_const;
    double close_ball_speed_const;
    double far_ball_speed_const;
    double head2ball_speed;
    double goalkeeper_radius;
    double goalkeeper_front_dis;
    double goalkeeper_mid_dis;
    double goalkeeper_side_dis;
    double goalkeeper_front_angle;
    double goalkeeper_mid_angle;
    double goalkeeper_front_speed;
    double goalkeeper_mid_speed;
    double goalkeeper_side_speed;
    
    //=======avoid=========
    int v_fast;
    int v_af;
    double good_angle;
    double far_good_angle;
    double final_angle;
    double af_angle;
    double num_change;
    double go_where_x;
    double go_where_y;
    int df_1;
    int df_1_dis;
    int df_2;
    int df_2_dis;
    int dd_1;
    int dd_1_dis;
    int dd_2;
    int dd_2_dis;
    int b_picture_m,avoid_go;
    int not_good_p;
    ros::Publisher tovision;
    ros::Publisher route_pub;
    strategy::strategylook vision_per;
    double kp,ki,kd;
    void Pub_route();
    double Artificial_field(int main_vec, int close_dis, int RedLine, int red_dis_average, int dangerous_dis, int red_line_dangerous_dis);
    //=====================
    

    std::vector<double> SPlanning_Velocity;
    std::vector<double> Distant;
    std::vector<int> ScanLine;
    std::vector<double> ChooseSide;
    std::vector<double> Distance_Settings;
    std::vector<double> Attack_Strategy;
    std::vector<double> Chase_Strategy;
    std::vector<double> Zone_Attack;
    std::vector<double> TypeS_Attack;
    std::vector<double> TypeU_Attack;
    std::vector<double> SideSpeedUp;


    // Robot shoot signal publisher
    int shoot = 0;
    bool bool_shoot(double goal_dis)
    {
        static ros::Time start = ros::Time::now();
        ros::Time current = ros::Time::now();
        double start_time = (double)(start.sec+(double)start.nsec/1000000000);
        double current_time = (double)(current.sec+(double)current.nsec/1000000000);
        //        if(current_time - start_time <1)
        //            return false;
        //        else
        //            return true;
        if(goal_dis <= 3){
            if(current_time-start_time > 1){
                start_time = current_time;
                return true;
            }
        }else{
            return false;
        }
    }
    void loadParam(ros::NodeHandle *n);
};



