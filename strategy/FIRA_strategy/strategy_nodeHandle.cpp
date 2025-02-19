#include "strategy_nodeHandle.h"
#include "FIRA_pathplan.h"


Strategy_nodeHandle::
Strategy_nodeHandle(int argc, char** argv):
    BaseNode(argc,argv,Node_Name)
{
    roleAry[0] = Role_Goalkeeper;
}


void Strategy_nodeHandle::ros_comms_init(){
    n = new ros::NodeHandle();
    web_connected = false;
    issimulator = 0;
    /*
    global_env->blue.pos.x = 3.35;
    global_env->blue.pos.y = 0;
    global_env->blue.pos.z = 0;

    global_env->yellow.pos.x = -3.35;
    global_env->yellow.pos.y = 0;
    global_env->yellow.pos.z = 0;
    */
    ////std::cout << "blue goal   x="<<global_env->blue.pos.x << ",y="<<global_env->blue.pos.y << std::endl;
    ////std::cout << "yellow goal   x="<<global_env->yellow.pos.x << ",y="<<global_env->yellow.pos.y << std::endl;


    //robotSpeedPub = n->advertise<FIRA_status_plugin::RobotSpeedMsg>(Pub_Speed_Topic,1000);
    shoot = n->advertise<std_msgs::Int32>("/shoot",1000);
    std::string robot_prefix = Robot_Topic_Prefix;
    std::string robotOpt_prefix = RobotOpt_Topic_Prefix;
    std::string robotPos_suffix = Robot_Position_Topic_Suffix;

    //gazebo_ModelStates subscriber
    //Gazebo_Model_Name_sub = n->subscribe<gazebo_msgs::ModelStates>(ModelState_Topic_Name,1000,&Strategy_nodeHandle::find_gazebo_model_name_fun,this);

    //Is Gazebo simulation mode?
    IsSimulator = n->subscribe<std_msgs::Int32>(IsSimulator_Topic,1000,&Strategy_nodeHandle::subIsSimulator,this);

    GameState = n->subscribe<std_msgs::Int32>(GameState_Topic,1000,&Strategy_nodeHandle::subGameState,this);
    TeamColor = n->subscribe<std_msgs::String>(TeamColor_Topic,1000,&Strategy_nodeHandle::subTeamColor,this);

    Imu3d = n->subscribe<imu_3d::inertia>("/imu_3d",1000,&Strategy_nodeHandle::subImu3d,this);
    MotorFB = n->subscribe<geometry_msgs::Twist>("/motion/motionFB",1000,&Strategy_nodeHandle::subMotorFB,this);
    //ball subscriber
//    ball_sub = n->subscribe<FIRA_status_plugin::ModelMsg>(Ball_Topic_Name,1000,&Strategy_nodeHandle::ball_sub_fun,this);

//    //robot subscriber
//    robot_1_pos_sub   = n->subscribe<nav_msgs::Odometry>(robot_prefix  +"1"+robotPos_suffix,1000,&Strategy_nodeHandle::robot_1_pos_fun,this);
//    robot_2_pos_sub   = n->subscribe<nav_msgs::Odometry>(robot_prefix  +"2"+robotPos_suffix,1000,&Strategy_nodeHandle::robot_2_pos_fun,this);
//    robot_3_pos_sub   = n->subscribe<nav_msgs::Odometry>(robot_prefix  +"3"+robotPos_suffix,1000,&Strategy_nodeHandle::robot_3_pos_fun,this);
//    robotOpt_1_pos_sub = n->subscribe<nav_msgs::Odometry>(robotOpt_prefix  +"1"+robotPos_suffix,1000,&Strategy_nodeHandle::robotOpt_1_pos_fun,this);
//    robotOpt_2_pos_sub = n->subscribe<nav_msgs::Odometry>(robotOpt_prefix  +"2"+robotPos_suffix,1000,&Strategy_nodeHandle::robotOpt_2_pos_fun,this);
//    robotOpt_3_pos_sub = n->subscribe<nav_msgs::Odometry>(robotOpt_prefix  +"3"+robotPos_suffix,1000,&Strategy_nodeHandle::robotOpt_3_pos_fun,this);

    //robot_role
    std::cout << "Strategy_nodeHandle::ros_comms_init() say opponent = " << opponent << std::endl;
    std::string robot_role_prefix = opponent ?  RobotOpt_Topic_Prefix : Robot_Topic_Prefix;
    std::string robot_role_suffix = Robot_Role_Topic_Suffix;

    //role subscriber
    robot_2_role_sub = n->subscribe<std_msgs::Int32>(robot_role_prefix  +"2"+robot_role_suffix,1000,&Strategy_nodeHandle::robot_2_role_fun,this);
    robot_3_role_sub = n->subscribe<std_msgs::Int32>(robot_role_prefix  +"3"+robot_role_suffix,1000,&Strategy_nodeHandle::robot_3_role_fun,this);

    //speed subscriber
    std::string robotSpeed_suffix = RobotSpeed_Topic_Suffix;
    robot_1_speed_pub = n->advertise<geometry_msgs::Twist>(robot_prefix+"1"+robotSpeed_suffix,1000);
    robot_2_speed_pub = n->advertise<geometry_msgs::Twist>(robot_prefix+"2"+robotSpeed_suffix,1000);
    robot_3_speed_pub = n->advertise<geometry_msgs::Twist>(robot_prefix+"3"+robotSpeed_suffix,1000);
    robotOpt_1_speed_pub = n->advertise<geometry_msgs::Twist>(robotOpt_prefix+"1"+robotSpeed_suffix,1000);
    robotOpt_2_speed_pub = n->advertise<geometry_msgs::Twist>(robotOpt_prefix+"2"+robotSpeed_suffix,1000);
    robotOpt_3_speed_pub = n->advertise<geometry_msgs::Twist>(robotOpt_prefix+"3"+robotSpeed_suffix,1000);

    //one_Robot speed publish
    std::string robotSpeed= Robot_Topic_Speed;
    #ifndef GAZEBO_SIMULATOR
        robot_speed_pub = n->advertise<geometry_msgs::Twist>(Robot_Topic_Speed,1000);
        gazebo = false;
    #else
        robot_speed_pub = n->advertise<nubot_common::VelCmd>("nubot1/nubotcontrol/velcmd",1000);
        gazebo = true;
    #endif

    //std::cout << "PersonalStrategy ros_comms_init() finish" << std::endl;


}

void Strategy_nodeHandle::Transfer(int r_number){
    /* Transfer the value of the x, y axis of robot, ball and goal into the distance and angle
     * , in order to synchrnize the value of simulator and reality*/

/*-------------------The input value of the simulator -----------------------------------*/
    Vector3D Ball = global_env->currentBall.pos;
    Vector3D Robot = global_env->home[r_number].pos;
    Vector3D Goal;
    Vector3D Goal_op;

    int Team_color = 0;
    if(global_env->teamcolor == "Blue")Team_color = Team_Blue;
    else if(global_env->teamcolor == "Yellow")Team_color = Team_Yellow;


    /* extinguish the team color and then set the goal */

    if(Team_color == Team_Blue){
        Goal = global_env->yellow.pos;
        Goal_op = global_env->blue.pos;
//        goal_color = Goal_Yellow;

    }else if (Team_color == Team_Yellow){
        Goal = global_env->blue.pos;
        Goal_op = global_env->yellow.pos;
//        goal_color = Goal_Blue;

    }

    double vectorbr_x = Ball.x - Robot.x;
    double vectorbr_y = Ball.y - Robot.y;
    double vectordr_x = Goal.x - Robot.x;
    double vectordr_y = Goal.y - Robot.y;
    double vectordr_x_op = Goal_op.x - Robot.x;
    double vectordr_y_op = Goal_op.y - Robot.y;

    double phi_br = atan2(vectorbr_y, vectorbr_x) * rad2deg;
    double phi_dr = atan2(vectordr_y, vectordr_x) * rad2deg; //target deg
    double phi_dr_op = atan2(vectordr_y_op, vectordr_x_op) * rad2deg;
/*----------------------The input value of the real robot------------------------------------------------*/
    global_env->home[r_number].ball.distance = hypot(vectorbr_x,vectorbr_y);
    global_env->home[r_number].goal.distance = hypot(vectordr_x,vectordr_y);
    global_env->home[r_number].op_goal.distance = hypot(vectordr_x_op, vectordr_y_op);

    double goal_angle = phi_dr - global_env->home[r_number].rotation;
    if(goal_angle > 180)
    {
        goal_angle = goal_angle - 360;
    }else if(goal_angle < -180){
        goal_angle = goal_angle + 360;
    }
    double ball_angle = phi_br - global_env->home[r_number].rotation;
    if(ball_angle > 180)
    {
        ball_angle = ball_angle - 360;
    }else if(ball_angle < -180){
        ball_angle = ball_angle + 360;
    }
    double op_goal_angle = phi_dr_op - global_env->home[r_number].rotation;;
    if(op_goal_angle > 180)
    {
        op_goal_angle = op_goal_angle - 360;
    }else if(op_goal_angle < -180){
        op_goal_angle = op_goal_angle + 360;
    }
    global_env->home[r_number].goal.angle = goal_angle; // angle between robot to our attacking goal and robot's head direction
    global_env->home[r_number].ball.angle = ball_angle; // angle between robot to the ball and robot's head direction
    global_env->home[r_number].op_goal.angle = op_goal_angle;// angle between robot to opponent's attacking goal and robot's head direction

}
void Strategy_nodeHandle::rotateXY(double rotate,double inX,double inY,double &newX,double &newY){
    double rad = rotate*deg2rad;
    newX = inX*cos(rad)+inY*sin(rad);
    newY = -inX*sin(rad)+inY*cos(rad);
}

void Strategy_nodeHandle::pubSpeed(ros::Publisher *puber,double v_x,double v_y,double v_yaw,double robot_rot){
    double r_v_x,r_v_y;
    rotateXY(robot_rot,v_x,v_y,r_v_x,r_v_y);//relatively 2 absolutely
    #ifndef GAZEBO_SIMULATOR
        geometry_msgs::Twist speedMsg;
        //=====simulator use=====
        speedMsg.linear.x = /*r_*/-v_x;
        speedMsg.linear.y = /*r_*/v_y;
        speedMsg.angular.z = v_yaw ;
    #else
        nubot_common::VelCmd speedMsg;
        //=====simulator use=====
        speedMsg.Vx = /*r_*/v_y;
        speedMsg.Vy = /*r_*/v_x;
        speedMsg.w = v_yaw ;
    #endif
//    speedMsg.linear.x = 0;
//    speedMsg.linear.y = 0;
//    speedMsg.angular.z = 0 ;
    //=====entity use=====
//    speedMsg.linear.x = /*r_*/v_y;
//    speedMsg.linear.y = /*r_*/v_x;
//    speedMsg.angular.z = v_yaw ;
//    std::cout << "yaw="<< speedMsg.angular.z << std::endl;
    #ifndef GAZEBO_SIMULATOR
        if(issimulator==true){
            speedMsg.linear.x = /*r_*/v_y;
            speedMsg.linear.y = /*r_*/-v_x;
            puber->publish(speedMsg);
        }else if(issimulator==false){
            velocity_S_planning(&speedMsg);
            puber->publish(speedMsg);
    }
    #else
        velocity_S_planning(&speedMsg);
        puber->publish(speedMsg);
    #endif
}

void Strategy_nodeHandle::velocity_S_planning(geometry_msgs::Twist *msg){
//    msg->linear.x = 0;
//    msg->linear.y= 0;
//    msg->angular.z= 0 ;
    double Vdis = hypot(msg->linear.x,msg->linear.y);
    double alpha = atan(-msg->linear.x/msg->linear.y)*rad2deg;
    if(msg->linear.y>0){
        if(msg->linear.x>0)
            alpha+=180;
        else
            alpha-=180;
    }
//    std::cout << "alpha="<<alpha<<std::endl;
    double angle = msg->angular.z * rad2deg;
    bool IsVectorZero=0;
    double Vdis_max = SPlanning_Velocity[0];//3
    double Vdis_min = SPlanning_Velocity[1];//0.3
    double VTdis_max = SPlanning_Velocity[2];//60
    double VTdis_min = SPlanning_Velocity[3];//30
    double VTdis;
    double Tangle_max = SPlanning_Velocity[4];// 20
    double angle_max = SPlanning_Velocity[6];//144;
    double Tangle_min = SPlanning_Velocity[5];//3
    double angle_min = SPlanning_Velocity[7];
    double Tangle;
////Transfer vector to [0,100]
    if(Vdis==0)
        IsVectorZero=1;
    else if(Vdis>Vdis_max)
        VTdis=VTdis_max;
    else if(Vdis<Vdis_min)
        VTdis=VTdis_min;
    else
        VTdis = (VTdis_max-VTdis_min)*(cos(pi*((Vdis-Vdis_min)/(Vdis_max-Vdis_min)-1))+1)/2+VTdis_min;
////Transfer yaw to [0,100]
    if(fabs(angle)<0.1)
        Tangle=0;
    else if(fabs(angle)>angle_max)
        Tangle=Tangle_max;
    else if(fabs(angle)<angle_min)
        Tangle=Tangle_min;
    else
        Tangle=(Tangle_max-Tangle_min)*(cos(pi*((fabs(angle)-angle_min)/(angle_max-angle_min)-1))+1)/2+Tangle_min;
//// [-100,100]
    if(angle<0)
        Tangle = -Tangle;
        if(IsVectorZero){
         msg->linear.x = 0;
         msg->linear.y = 0;
        }else{
        msg->linear.x = VTdis*sin(alpha*deg2rad);
        msg->linear.y = -VTdis*cos(alpha*deg2rad);
        }
        msg->angular.z = Tangle;
//    std::cout<<"yaw = "<<msg->angular.z <<"\tmsg->linear.x ="<<msg->linear.x <<"\tmsg->linear.y ="<<msg->linear.y<< "\tangle="<<alpha<<std::endl;
}
#ifdef GAZEBO_SIMULATOR
void Strategy_nodeHandle::velocity_S_planning(nubot_common::VelCmd *msg){
    double Vdis = hypot(msg->Vx,msg->Vy);
    double alpha = atan(-msg->Vx/msg->Vy)*rad2deg;
    if(msg->Vy>0){
        if(msg->Vx>0)
            alpha+=180;
        else
            alpha-=180;
    }
//    std::cout << "alpha="<<alpha<<std::endl;
    double angle = msg->w * rad2deg;
    bool IsVectorZero=0;
    double Vdis_max = SPlanning_Velocity[0];//3
    double Vdis_min = SPlanning_Velocity[1];//0.3
    double VTdis_max = SPlanning_Velocity[2];//60
    double VTdis_min = SPlanning_Velocity[3];//30
    double VTdis;
    double Tangle_max = SPlanning_Velocity[4];// 20
    double angle_max = SPlanning_Velocity[6];//144;
    double Tangle_min = SPlanning_Velocity[5];//3
    double angle_min = SPlanning_Velocity[7];
    double Tangle;
////Transfer vector to [0,100]
    if(Vdis==0)
        IsVectorZero=1;
    else if(Vdis>Vdis_max)
        VTdis=VTdis_max;
    else if(Vdis<Vdis_min)
        VTdis=VTdis_min;
    else
        VTdis = (VTdis_max-VTdis_min)*(cos(pi*((Vdis-Vdis_min)/(Vdis_max-Vdis_min)-1))+1)/2+VTdis_min;
////Transfer yaw to [0,100]
    if(fabs(angle)<0.1)
        Tangle=0;
    else if(fabs(angle)>angle_max)
        Tangle=Tangle_max;
    else if(fabs(angle)<angle_min)
        Tangle=Tangle_min;
    else
        Tangle=(Tangle_max-Tangle_min)*(cos(pi*((fabs(angle)-angle_min)/(angle_max-angle_min)-1))+1)/2+Tangle_min;
//// [-100,100]
    if(angle<0)
        Tangle = -Tangle;
    if(IsVectorZero){
        msg->Vx = 0;
        msg->Vy = 0;
    }else{
        msg->Vx = VTdis*sin(alpha*deg2rad);
        msg->Vy = -VTdis*cos(alpha*deg2rad);
        // msg->Vx = VTdis*cos(alpha*deg2rad);
        // msg->Vy = VTdis*sin(alpha*deg2rad);
    }
    msg->w = Tangle;
}
#endif
void Strategy_nodeHandle::pubGrpSpeed(){
//    if(issimulator==true){
//        ////--------------------speed test----------------
//        double dVectorMax = VectorMax;
//        double dVectorMin = VectorMin;
//        double dVector[3];
//        for(int i=0;i<3;i++){
//            dVector[i] = hypot(global_env->home[i].v_x,global_env->home[i].v_y);
//            double gama = atan2(global_env->home[i].v_y,global_env->home[i].v_x);
//            if(dVector[i]>dVectorMax){
//                c_v_x=VectorMax*cos(gama);
//                c_v_y=VectorMax*sin(gama);

//                global_env->home[i].v_x = c_v_x;
//                global_env->home[i].v_y = c_v_y;
//             }else if(dVector[i]<dVectorMin && dVector[i]>0){
//                c_v_x=VectorMin*cos(gama);
//                c_v_y=VectorMin*sin(gama);

//                global_env->home[i].v_x = c_v_x;
//                global_env->home[i].v_y = c_v_y;
//            }

//    //        printf("%d. c_v_x=%lf, c_v_y=%lf ",i,c_v_x,c_v_y);
//        }printf("\n");*/
//    ////-------------------------pub----------------------------------------
//        if(!opponent){
//            pubSpeed(&robot_1_speed_pub  ,global_env->home[0].v_x,global_env->home[0].v_y,global_env->home[0].v_yaw,global_env->home[0].rotation);
//            pubSpeed(&robot_2_speed_pub  ,global_env->home[1].v_x,global_env->home[1].v_y,global_env->home[1].v_yaw,global_env->home[1].rotation);
//            pubSpeed(&robot_3_speed_pub  ,global_env->home[2].v_x,global_env->home[2].v_y,global_env->home[2].v_yaw,global_env->home[2].rotation);
//        }else{
//            pubSpeed(&robotOpt_1_speed_pub,global_env->opponent[0].v_x,global_env->opponent[0].v_y,global_env->opponent[0].v_yaw,global_env->opponent[0].rotation);
//            pubSpeed(&robotOpt_2_speed_pub,global_env->opponent[1].v_x,global_env->opponent[1].v_y,global_env->opponent[1].v_yaw,global_env->opponent[1].rotation);
//            pubSpeed(&robotOpt_3_speed_pub,global_env->opponent[2].v_x,global_env->opponent[2].v_y,global_env->opponent[2].v_yaw,global_env->opponent[2].rotation);
//    //            printf("1: %lf %lf %lf \n",global_env->opponent[0].v_x,global_env->opponent[0].v_y,global_env->opponent[0].v_yaw);
//    //            printf("2: %lf %lf %lf \n",global_env->opponent[1].v_x,global_env->opponent[1].v_y,global_env->opponent[1].v_yaw);
//    //            printf("3: %lf %lf %lf \n",global_env->opponent[2].v_x,global_env->opponent[2].v_y,global_env->opponent[2].v_yaw);
   //     }
 //   }else if(issimulator==false){
       pubSpeed(&robot_speed_pub,global_env->home[global_env->RobotNumber].v_x,global_env->home[global_env->RobotNumber].v_y,global_env->home[global_env->RobotNumber].v_yaw,global_env->home[global_env->RobotNumber].rotation);
//    }
}
void Strategy_nodeHandle::loadParam(ros::NodeHandle *n){
     n->getParam("/AvoidChallenge/IsSimulator",issimulator);
     if(n->getParam("/FIRA/blackItem/angle",Blackangle)){
//     std::cout << "param Blackangle=" << Blackangle <<std::endl;
    }
     if(n->getParam("/FIRA/RobotNumber",global_env->RobotNumber)){
//     std::cout << "param RobotNumber=" << global_env->RobotNumber<<std::endl;
    }
     if(n->getParam("/AvoidChallenge/SPlanning_Velocity", SPlanning_Velocity)){
//         for(int i=0;i<8;i++)
//             std::cout<< "param SPlanning_Velocity["<< i << "]=" << SPlanning_Velocity[i] << std::endl;
//     std::cout << "====================================" << std::endl;
     }
    if(n->getParam("/FIRA/Distance_Settings", Distance_Settings)){
//        for(int i=0;i<3;i++)
//            std::cout<< "param Distance_Settings["<< i << "]=" << Distance_Settings[i] << std::endl;
//     std::cout << "====================================" << std::endl;
    }
}
