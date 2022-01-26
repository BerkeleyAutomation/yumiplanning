#include <iostream>
#include "single_arm.h"
#include "dual_arm.h"

int main(){
    YuMiPlanning::CollisionChecker checker("/home/jkerr/yumi/yumiplanning/yumi_description/");
    std::vector<double> l={-1.24839656, -1.09802876,  1.06634394,  0.31386161,  1.90125141,
                        1.3205139 ,  2.43563939};
    std::vector<double> r={1.21442839, -1.03205606, -1.10072738,  0.2987352,
                             -1.85257716,  1.25363652, -2.42181893};
    YuMiPlanning::SingleArmPlanner planner(checker,true,true);
    std::vector<double> s={1.7886884440180186, -0.44371156619454205, -1.7481374222057635, -0.40361105137018755, 0.16508589386459488, 1.1032037623419368, -0.1919595195294482};
    std::vector<double> g ={1.7889450395890478, 0.5727117937361313, -0.1202846167553847, 1.105436756336941, -0.8347407460212457, -0.9100656608720741, -0.17140514236767576};
    std::vector<double> other_q =r;
    // YuMiPlanning::DualArmPlanner planner(checker);
    ompl::geometric::PathGeometric path = planner.planPath(s,g,other_q,1);
    // ompl::geometric::PathGeometric path = planner.planPath(s,g,1,-1);
    if(!path.check()){
        std::cerr<<"Failed to plan path\n";
        return 1;
    }
    std::cout<<"Path cost: "<<path.length()<<std::endl;
    path.print(std::cout);
    return 0;
}