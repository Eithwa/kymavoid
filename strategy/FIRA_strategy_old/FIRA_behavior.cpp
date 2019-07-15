#include "FIRA_behavior.h"
#include "math.h"



FIRA_behavior_class::FIRA_behavior_class(){
   opponent = false;
}

//start---simulator---
void FIRA_behavior_class::setEnv(Environment iEnv){
//void FIRA_behavior_class::setEnv(){
   //Vector3D ball = iEnv.currentBall.pos;
   //printf("global_env   b_x=%lf,b_y=%lf\n",ball.x,ball.y);
   env = iEnv;

   if(opponent){
       for(int i = 0;i < PLAYERS_PER_SIDE;i++){
           env.home[i] = iEnv.opponent[i];
           env.opponent[i] = iEnv.home[i];
//           printf("in opponent behavior\n");
       }

   }
}

void FIRA_behavior_class::readroleAry(int robotIndex,int role){
//    for(int i=0; i<PLAYERS_PER_SIDE;i++){
        switch(role){
            case Role_AvoidBarrier:
                behavior_AvoidBarrier(robotIndex);
                break;
            case Role_Halt:
                behavior_Halt(robotIndex);
                break;
        }
//    }
}

int* FIRA_behavior_class::getactionAry(){
    return actionAry;
}
void FIRA_behavior_class::behavior_Halt(int robotIndex){
    actionAry[robotIndex] = action_Halt;
//    printf("halt\n");
}

void FIRA_behavior_class::behavior_AvoidBarrier(int robotIndex){
    actionAry[robotIndex] = action_AvoidBarrier;
//    printf("behavior_goalkeeper\n");
}
void FIRA_behavior_class::loadParam(ros::NodeHandle *n){
    if(n->getParam("/FIRA_Behavior/Attack_Strategy", Attack_Strategy)){
//        for(int i=0;i<3;i++)
//            std::cout<< "param Attack_Strategy["<< i << "]=" << Attack_Strategy[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA_Behavior/Chase_Strategy", Chase_Strategy)){
//        for(int i=0;i<5;i++)
//            std::cout<< "param Chase_Strategy["<< i << "]=" << Chase_Strategy[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA_Behavior/Corner_Kick", Corner_Kick)){
//        for(int i=0;i<2;i++)
//            std::cout<< "param Corner_Kick["<< i << "]=" << Corner_Kick[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA_Behavior/Side_Speed_UP", Side_Speed_UP)){
//        for(int i=0;i<2;i++)
//            std::cout<< "param Side_Speed_UP["<< i << "]=" << Side_Speed_UP[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA_Behavior/TypeS_Attack", TypeS_Attack)){
//        for(int i=0;i<3;i++)
//            std::cout<< "param TypeS_Attack["<< i << "]=" << TypeS_Attack[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA_Behavior/TypeU_Chase", TypeU_Chase)){
//        for(int i=0;i<2;i++)
//            std::cout<< "param TypeU_Chase["<< i << "]=" << TypeU_Chase[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
    if(n->getParam("/FIRA_Behavior/Zone_Attack", Zone_Attack)){
//        for(int i=0;i<1;i++)
//            std::cout<< "param Zone_Attack["<< i << "]=" << Zone_Attack[i] << std::endl;
//    std::cout << "====================================" << std::endl;
    }
}
