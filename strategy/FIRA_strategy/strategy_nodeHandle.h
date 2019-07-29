/**
 * @file BaseNode.h
 *
 * @brief Ros communication central!
 *
 * @date February 2014
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef Strategy_nodeHandle_HPP_
#define Strategy_nodeHandle_HPP_


//#define GAZEBO_SIMULATOR

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <string>
#include <iostream>
#include "../common/Env.h"
#include "../common/BaseNode.h"

//message inclue
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "vision/Object.h"
#include "fira_status_plugin/RobotSpeedMsg.h"
#include "fira_status_plugin/ModelMsg.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Int32MultiArray.h"
#include "vision/visionlook.h"

#include "imu_3d/inertia.h"

#ifdef GAZEBO_SIMULATOR
#include "nubot_common/VelCmd.h"
#endif
/*****************************************************************************
** Define
*****************************************************************************/
#define Ball_Topic_Name         "/FIRA/Strategy/WorldMap/soccer"
#define ModelState_Topic_Name  "/gazebo/model_states"
#define IsSimulator_Topic "/FIRA/IsSimulator"
//robot prefixdefine IsSimulator_Topic "/FIRA/IsSimulator"
#define Robot_Topic_Prefix "/FIRA/R"
#define RobotOpt_Topic_Prefix "/FIRA/Opt_R"
#define GameState_Topic "/FIRA/GameState"
#define TeamColor_Topic "/FIRA/TeamColor"
#define Vision_Topic "/vision/object"
//BlackObject_distance
#define  BlackObject_Topic "/vision/BlackRealDis"
//one_Robot speed
#define Robot_Topic_Speed "/motion/cmd_vel"
//robot suffix
#define Robot_Position_Topic_Suffix "/Strategy/WorldMap/RobotPos"
#define Robot_Role_Topic_Suffix "/Strategy/Coach/role"
#define RobotSpeed_Topic_Suffix "/Strategy/PathPlan/RobotSpeed"

#define Node_Name "PersonalStrategy"

//RobotNumber
#define RobotNumber_Topic "/FIRA/RobotNumber"

#define VectorMax 1.42
#define VectorMin 0.05
/*****************************************************************************
** Class
*****************************************************************************/

class Strategy_nodeHandle :public BaseNode {
public:
    Strategy_nodeHandle(int argc, char** argv);
    bool web_connected;
    bool gazebo;
    virtual ~Strategy_nodeHandle(){}

    void pubGrpSpeed();

    void setEnv(Environment *inEnv){
        global_env = inEnv;

    }
    // shoot signal
    ros::Publisher shoot;
    // pub shoot signal
    void pubShoot(int shoot_value){
        ros::Time current = ros::Time::now();
        static double current_time =10000;
        static double record_time;
        double time = current_time-record_time;
        if(time>1){
            std_msgs::Int32 shoot_signal;
            shoot_signal.data = shoot_value;
            shoot.publish(shoot_signal);
            printf("current_time=%lf,record_time=%lf\n",current_time,record_time);
            record_time = current_time;
        }
        current_time = (double)(current.sec+(double)current.nsec/1000000000);

    }
    struct SaveAry{
            int distance;
            int location;
            int counter;
        };
    struct SaveAry Save[20];
    struct New_SaveAry{
            int distance;
            int location;
            int counter;
            int middle;
            int middle_place;
        };
    struct New_SaveAry New_Save[20];

    int* getRoleAry(){
//        std::cout << "roleAry[0]=" <<   roleAry[0] << std::endl;
        return roleAry;
    }

    void setOpponent(bool inBool){opponent = inBool;}

    ros::NodeHandle* getNodeHandle(){return n;}
    long getGameState(){return gamestate;}
    std::string getTeamColor(){return teamcolor;}
    int getIsSimulator(){return issimulator;}

    //BlackObject
    int Blackangle;
    int *blackobject;
    void loadParam(ros::NodeHandle *n);
    int* getBlackObject(){return blackobject;}
protected:
    void ros_comms_init();

private:
    int roleAry[PLAYERS_PER_SIDE];
    std::string model_array[11];
    bool opponent;

    Environment *global_env;


    ros::NodeHandle *n;
    long gamestate;
    std::string teamcolor;
    int  issimulator;

    //gazebo_ModelStates subscriber
    ros::Subscriber Gazebo_Model_Name_sub;

    //ball subscriber
    ros::Subscriber ball_sub;

    //robot subscriber
    ros::Subscriber robot_1_pos_sub  ;
    ros::Subscriber robot_2_pos_sub  ;
    ros::Subscriber robot_3_pos_sub  ;
    ros::Subscriber robotOpt_1_pos_sub  ;
    ros::Subscriber robotOpt_2_pos_sub  ;
    ros::Subscriber robotOpt_3_pos_sub  ;

    ros::Subscriber GameState;
    ros::Subscriber TeamColor;
    ros::Subscriber Vision;
    ros::Subscriber IsSimulator;
    ros::Subscriber Imu3d;
    ros::Subscriber MotorFB;

    //BlackObject
    ros::Subscriber BlackObject;
    ros::Subscriber redObject;
    ros::Subscriber smpicture;

    //robot role publisher
    //no robot_1_role_sub, because robot_1 is always goal keeper
    ros::Subscriber robot_2_role_sub;
    ros::Subscriber robot_3_role_sub;

    //robot speed publisher
    ros::Publisher robot_1_speed_pub;
    ros::Publisher robot_2_speed_pub;
    ros::Publisher robot_3_speed_pub;
    ros::Publisher robotOpt_1_speed_pub;
    ros::Publisher robotOpt_2_speed_pub;
    ros::Publisher robotOpt_3_speed_pub;

    //one_robot speed
    ros::Publisher robot_speed_pub;

    /// load param begin
    std::vector<double> SPlanning_Velocity;
    std::vector<double> Distance_Settings;
    /// load param end

    bool run_one = false;

    void Transfer(int);

    double c_v_x,c_v_y;

    void rotateXY(double rotate,double inX,double inY,double &newX,double &newY);
    void pubSpeed(ros::Publisher *puber,double v_x,double v_y,double v_yaw,double robot_rot);
    void velocity_S_planning(geometry_msgs::Twist *msg);
    #ifdef GAZEBO_SIMULATOR
    void velocity_S_planning(nubot_common::VelCmd *msg);
    #endif
    //find Gazebo_msgs::ModelStates name
    void find_gazebo_model_name_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){//----------------------------------printf here is ok, but printf next row will crash if i open over one robot map
//        printf("testttttttttttttt\n");

        if(run_one) return;

        int model_length;
        model_length = msg->name.size();
        /*
        for(int i =0;i<11;i++){
            if(msg->name[i].c_str()==NULL){
                model_length = i;
            }
            if(model_length!= 0) i = 11;
//            printf("%d\n",model_length);
        }*/
        printf("model_length= %d",model_length);
        for(int i = 0; i<model_length;i++){
            model_array[i] = msg->name[i];
//            printf("%d = %s\n",i,model_array[i].c_str());
        }
        run_one = true;
//        printf("%s\n",model_array[6].c_str());
    }
    int get_model_num(const std::string ModelName){
        int send_back_num;
        for(int i =0; i< 11; i++){
            if(model_array[i] == ModelName) return i;
        }
    }

//Use_topic_gazebo_msgs_Model_States to get model position
    void  ball_sub_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("soccer");
//        printf("model_num=%d\n",model_num);
//        ROS_INFO("name=%s",msg->name[1].c_str());
        geometry_msgs::Pose tPose = msg->pose[model_num];
//        ROS_INFO("x=%lf",tPose.position.x);
        global_env->currentBall.pos.x = tPose.position.x;
        global_env->currentBall.pos.y = tPose.position.y;

//        printf("ball_x = %lf,ball_y = %lf\n",  tPose.position.x, tPose.position.y);

    }
    void robot_1_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("R1");
//        printf("model_num=%d\n",model_num);
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->home[0].pos.x = tPose.position.x;
        global_env->home[0].pos.y = tPose.position.y;
        Transfer(0);

        double w = tPose.orientation.w;
        double x = tPose.orientation.x;
        double y = tPose.orientation.y;
        double z = tPose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->home[0].rotation = yaw;
//        printf("yaw =%lf\n",yaw);

    }
    void robot_2_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("R2");
//        printf("model_num=%d\n",model_num);
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->home[1].pos.x = tPose.position.x;
        global_env->home[1].pos.y = tPose.position.y;
        Transfer(1);

        double w = tPose.orientation.w;
        double x = tPose.orientation.x;
        double y = tPose.orientation.y;
        double z = tPose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->home[1].rotation = yaw;
//        printf("yaw =%lf\n",yaw);

    }
    void  robot_3_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("R3");
//        printf("model_num=%d\n",model_num);
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->home[2].pos.x = tPose.position.x;
        global_env->home[2].pos.y = tPose.position.y;
        Transfer(2);

        double w = tPose.orientation.w;
        double x = tPose.orientation.x;
        double y = tPose.orientation.y;
        double z = tPose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->home[2].rotation = yaw;
//        printf("yaw =%lf\n",yaw);

    }


    void  robotOpt_1_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("FIRA_Opt_1");
//        printf("model_num=%d\n",model_num);
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->opponent[0].pos.x = tPose.position.x;
        global_env->opponent[0].pos.y = tPose.position.y;

        double w = tPose.orientation.w;
        double x = tPose.orientation.x;
        double y = tPose.orientation.y;
        double z = tPose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->opponent[0].rotation = yaw;
//        printf("yaw =%lf\n",yaw);
    }


    void  robotOpt_2_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("FIRA_Opt_2");
//        printf("model_num=%d\n",model_num);
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->opponent[1].pos.x = tPose.position.x;
        global_env->opponent[1].pos.y = tPose.position.y;

        double w = tPose.orientation.w;
        double x = tPose.orientation.x;
        double y = tPose.orientation.y;
        double z = tPose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->opponent[1].rotation = yaw;
//        printf("yaw =%lf\n",yaw);
    }


    void  robotOpt_3_pos_fun(const gazebo_msgs::ModelStates::ConstPtr &msg){
        int model_num;
        model_num = get_model_num("FIRA_Opt_3");
//        printf("model_num=%d\n",model_num);
        geometry_msgs::Pose tPose = msg->pose[model_num];
        global_env->opponent[2].pos.x = tPose.position.x;
        global_env->opponent[2].pos.y = tPose.position.y;

        double w = tPose.orientation.w;
        double x = tPose.orientation.x;
        double y = tPose.orientation.y;
        double z = tPose.orientation.z;
        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
        global_env->opponent[2].rotation = yaw;
//        printf("yaw =%lf\n",yaw);
    }
//    void  ball_sub_fun(const FIRA_status_plugin::ModelMsg::ConstPtr &msg){
//        global_env->currentBall.pos.x = msg->x;
//        global_env->currentBall.pos.y = msg->y;

//        //std::cout << "[ball] x=" << msg->x <<",y=" << msg->y << std::endl;
//    }

//    void  robot_1_pos_fun(const nav_msgs::Odometry::ConstPtr &msg){
//        global_env->home[0].pos.x = msg->pose.pose.position.x;
//        global_env->home[0].pos.y = msg->pose.pose.position.y;


//        double w = msg->pose.pose.orientation.w;
//        double x = msg->pose.pose.orientation.x;
//        double y = msg->pose.pose.orientation.y;
//        double z = msg->pose.pose.orientation.z;
//        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
//        global_env->home[0].rotation = yaw;

//        //std::cout << "[robot-1] x=" << global_env->home[0].pos.x <<",y=" << global_env->home[0].pos.y << std::endl;
//    }
//    void  robot_2_pos_fun(const nav_msgs::Odometry::ConstPtr &msg){
//        global_env->home[1].pos.x = msg->pose.pose.position.x;
//        global_env->home[1].pos.y = msg->pose.pose.position.y;

//        double w = msg->pose.pose.orientation.w;
//        double x = msg->pose.pose.orientation.x;
//        double y = msg->pose.pose.orientation.y;
//        double z = msg->pose.pose.orientation.z;
//        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
//        global_env->home[1].rotation = yaw;

//        //std::cout << "[robot-2] x=" << global_env->home[1].pos.x <<",y=" << global_env->home[1].pos.y << std::endl;

//    }
//    void  robot_3_pos_fun(const nav_msgs::Odometry::ConstPtr &msg){
//        global_env->home[2].pos.x = msg->pose.pose.position.x;
//        global_env->home[2].pos.y = msg->pose.pose.position.y;

//        double w = msg->pose.pose.orientation.w;
//        double x = msg->pose.pose.orientation.x;
//        double y = msg->pose.pose.orientation.y;
//        double z = msg->pose.pose.orientation.z;
//        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
//        global_env->home[2].rotation = yaw;

//        //std::cout << "[robot-3] x=" << global_env->home[2].pos.x <<",y=" << global_env->home[2].pos.y << std::endl;

//    }


//    void  robotOpt_1_pos_fun(const nav_msgs::Odometry::ConstPtr &msg){
//        global_env->opponent[0].pos.x = msg->pose.pose.position.x;
//        global_env->opponent[0].pos.y = msg->pose.pose.position.y;

//        double w = msg->pose.pose.orientation.w;
//        double x = msg->pose.pose.orientation.x;
//        double y = msg->pose.pose.orientation.y;
//        double z = msg->pose.pose.orientation.z;
//        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
//        global_env->opponent[0].rotation = yaw;

//        //std::cout << "[robotOpt-1] x=" << global_env->opponent[0].pos.x <<",y=" << global_env->opponent[0].pos.y << std::endl;

//    }


//    void  robotOpt_2_pos_fun(const nav_msgs::Odometry::ConstPtr &msg){
//        global_env->opponent[1].pos.x = msg->pose.pose.position.x;
//        global_env->opponent[1].pos.y = msg->pose.pose.position.y;
//        double w = msg->pose.pose.orientation.w;
//        double x = msg->pose.pose.orientation.x;
//        double y = msg->pose.pose.orientation.y;
//        double z = msg->pose.pose.orientation.z;
//        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
//        global_env->opponent[1].rotation = yaw;
//    }


//    void  robotOpt_3_pos_fun(const nav_msgs::Odometry::ConstPtr &msg){
//        global_env->opponent[2].pos.x = msg->pose.pose.position.x;
//        global_env->opponent[2].pos.y = msg->pose.pose.position.y;
//        double w = msg->pose.pose.orientation.w;
//        double x = msg->pose.pose.orientation.x;
//        double y = msg->pose.pose.orientation.y;
//        double z = msg->pose.pose.orientation.z;
//        double yaw = atan2(  2*(w*z+x*y),  1-2*(y*y+z*z)  )*rad2deg;
//        global_env->opponent[2].rotation = yaw;
//    }


    void  robot_2_role_fun(const std_msgs::Int32::ConstPtr &msg){

        roleAry[1] = msg->data;
        //if(roleAry[1] == Role_Attack)   std::cout << "Robot-2 is Role_Attack"
         //std::cout << "Robot-2 is Role=" << roleAry[1] << std::endl;
    }

    void  robot_3_role_fun(const std_msgs::Int32::ConstPtr &msg){

        roleAry[2] = msg->data;

        //std::cout << "Robot-3 is Role=" << roleAry[2] << std::endl;
    }
    void subGameState(const std_msgs::Int32::ConstPtr &msg){
        gamestate=msg->data;
        //std::cout<<"receive"<<std::endl;
    }
    void subTeamColor(const std_msgs::String::ConstPtr &msg){
        teamcolor= msg->data;
        //std::cout<<"receive"<<std::endl;
    }
    void subImu3d(const imu_3d::inertia::ConstPtr &msg){
        global_env->home[global_env->RobotNumber].FB_yaw= (((msg->yaw-M_PI)*rad2deg)>0)?-((msg->yaw-M_PI)*rad2deg-180):-((msg->yaw-M_PI)*rad2deg+180);
     //   std::cout<<msg->yaw<<std::endl;
    }
    void subMotorFB(const geometry_msgs::Twist::ConstPtr &msg){
        global_env->home[global_env->RobotNumber].FB_x= msg->linear.x;
        global_env->home[global_env->RobotNumber].FB_y= msg->linear.y;

//        std::cout<<msg->yaw<<std::endl;
    }

    void submpicture(const vision::visionlook::ConstPtr &msg){
        global_env->picture_m=msg->mpicture;
        global_env->gray_ave=msg->gray_ave;
    }
    void subVision(const vision::Object::ConstPtr &msg){
        double ball_distance,yellow_distance,blue_distance;

        yellow_distance = msg->yellow_dis;
        blue_distance = msg->blue_dis;
        if(global_env->teamcolor == "Blue"){
            global_env->home[global_env->RobotNumber].op_goal.distance= blue_distance/100;
            global_env->home[global_env->RobotNumber].op_goal.angle = msg->blue_ang;
            global_env->home[global_env->RobotNumber].goal.distance = yellow_distance/100;
            global_env->home[global_env->RobotNumber].goal.angle = msg->yellow_ang;

        }else if(global_env->teamcolor == "Yellow"){
            global_env->home[global_env->RobotNumber].op_goal.distance= yellow_distance/100;
            global_env->home[global_env->RobotNumber].op_goal.angle = msg->yellow_ang;
            global_env->home[global_env->RobotNumber].goal.distance= blue_distance/100;
            global_env->home[global_env->RobotNumber].goal.angle = msg->blue_ang;
        }

       ball_distance = msg->ball_dis;
       global_env->home[global_env->RobotNumber].ball.distance = ball_distance/100;
       global_env->home[global_env->RobotNumber].ball.angle = msg->ball_ang;

    }
    void subBlackObject(const std_msgs::Int32MultiArray::ConstPtr &msg){
        int All_Line_distance[360];        
        for(int i=0; i<360/Blackangle; i++){
             All_Line_distance[i]= msg-> data[i];
        }
        //blackdis[60]轉為All_Line_distance[0](車頭角)
        // int j=60;
        // for(int i=0; i<=60; i++){
        //     global_env->blackdis[j] =  All_Line_distance[i];
        //     j--;
        // }
        // j=119;
        // for(int i=61; i<=119; i++){
        //     global_env->blackdis[j] =  All_Line_distance[i];
        //     j--;
        // }
        for(int i = 0; i < 120; i++){
            global_env->blackdis[((119-i)+120-59)%120] = All_Line_distance[i];
            //std::cout<< i << "  "<<((119-i)+120-59)%120<<std::endl;
        }
    }
    void subredObject(const std_msgs::Int32MultiArray::ConstPtr &msg){
        int All_Line_distance[360];
        for(int i=0; i<360/Blackangle; i++){
             All_Line_distance[i]= msg-> data[i];
        }
        // int j=60;
        // for(int i=0; i<=60; i++){
        //     global_env->reddis[j] =  All_Line_distance[i];
        //     j--;
        // }
        // j=119;
        // for(int i=61; i<=119; i++){
        //     global_env->reddis[j] =  All_Line_distance[i];
        //     j--;
        // }
        for(int i = 0; i < 120; i++){
            global_env->reddis[((119-i)+120-59)%120] = All_Line_distance[i];
            //std::cout<< i << "  "<<((119-i)+120-59)%120<<std::endl;
        }
    }
    /*void subBlackObject(const std_msgs::Int32MultiArray::ConstPtr &msg){
        int All_Line_distance[360];
        int j=119,k=60;
        for(int i=0; i<360/Blackangle; i++){ //Blackangle=3
        All_Line_distance[i] = msg -> data[i];
        //            ROS_INFO("%d=%d\n",i,All_Line_distance[i]);
        }
        for(int i = 0 ; i < 61 ; i++){
         global_env->blackdis[i] =  All_Line_distance[k];
         global_env->blackangle[i] = k*3;
         k--;
        //             ROS_INFO("%d=%d\n",i,  global_env->blackdis[i]);
        }

        for(int i = 61 ; i <= 120 ; i++){
             global_env->blackdis[i] =  All_Line_distance[j];
             global_env->blackangle[i] = j*3;
             if(global_env->blackangle[i] > 180){
                global_env->blackangle[i] = -(360 - global_env->blackangle[i]);
             }
             j--;
        }
    }*/
    void subIsSimulator(const std_msgs::Int32::ConstPtr &msg){
        issimulator=msg->data;
        if(issimulator==1){
            global_env->issimulator = 1;
            //Use_topic_gazebo_msgs_Model_States to get model position
/*
            ball_sub = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&Strategy_nodeHandle::ball_sub_fun,this);

            //robot subscriber
            robot_1_pos_sub   = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&Strategy_nodeHandle::robot_1_pos_fun,this);
            robot_2_pos_sub   = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&Strategy_nodeHandle::robot_2_pos_fun,this);
            robot_3_pos_sub   = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&Strategy_nodeHandle::robot_3_pos_fun,this);
            robotOpt_1_pos_sub = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&Strategy_nodeHandle::robotOpt_1_pos_fun,this);
            robotOpt_2_pos_sub = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&Strategy_nodeHandle::robotOpt_2_pos_fun,this);
            robotOpt_3_pos_sub = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&Strategy_nodeHandle::robotOpt_3_pos_fun,this);
*/
        }
        else{
            web_connected = true;
            global_env->issimulator = 0;
            //contact image
            Vision = n->subscribe<vision::Object>(Vision_Topic,1000,&Strategy_nodeHandle::subVision,this);
            BlackObject = n->subscribe<std_msgs::Int32MultiArray>(BlackObject_Topic,1000,&Strategy_nodeHandle::subBlackObject,this);
            redObject = n->subscribe<std_msgs::Int32MultiArray>("/vision/redRealDis",1000,&Strategy_nodeHandle::subredObject,this);
            smpicture=n->subscribe<vision::visionlook>("/vision/picture_m",1000,&Strategy_nodeHandle::submpicture,this);
        }
    }
};

#endif /* NODE_HPP_ */
