#pragma once
#include "collision_checker.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
namespace YuMiPlanning{
class SingleArmValidity {
public:
    /*
        left specifies whether this plans for left or right arm, use_other specifies whether to plan
        around the other arm (false means ignore collisions with the other arm)
    */
    SingleArmValidity(CollisionChecker checker, bool left):colChecker(checker),left(left){}
    void setOther(std::vector<double> &new_other){
        other_q = new_other;
    }
    bool isValidSingle(const ompl::base::State *state){
        auto q = state->as<ompl::base::RealVectorStateSpace::StateType>();
        std::vector<double> cur_q(7);
        for(int i=0;i<7;i++)cur_q[i] = (*q)[i];
        if(left){
            return !colChecker.isColliding(cur_q,colChecker.L_TIP);
        }else{
            return !colChecker.isColliding(cur_q,colChecker.R_TIP);
        }
    }
    bool isValidOther(const ompl::base::State *state){
        auto q = state->as<ompl::base::RealVectorStateSpace::StateType>();
        std::vector<double> cur_q(7);
        for(int i=0;i<7;i++)cur_q[i] = (*q)[i];
        if(left){
            return !colChecker.isColliding(cur_q,other_q);
        }else{
            return !colChecker.isColliding(other_q,cur_q);
        }
    }
private:
    CollisionChecker colChecker;
    bool left;
    std::vector<double> other_q;
};
class SingleArmPlanner{
public:
    SingleArmPlanner(CollisionChecker checker, bool left, bool use_other){
        //initialize the state space
        ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(7));
        ompl::base::RealVectorBounds bounds(7);
        checker.getLeftJointLims(bounds.low.data(),bounds.high.data());
        space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
        //use simplesetup to make a planner
        setup = ompl::geometric::SimpleSetupPtr(new ompl::geometric::SimpleSetup(space));
        validity=std::make_shared<SingleArmValidity>(checker,left);
        std::function<bool(const ompl::base::State*)> fun;
        if(use_other){
            fun=std::bind<bool>(&SingleArmValidity::isValidOther,validity,std::placeholders::_1);
        }else{
            fun=std::bind<bool>(&SingleArmValidity::isValidSingle,validity,std::placeholders::_1);
        }
        setup->setStateValidityChecker(fun);
        setup->getSpaceInformation()->setStateValidityCheckingResolution(0.003);//This is fraction of state space, not radians
        ompl::base::PlannerPtr planner(new ompl::geometric::RRTConnect(setup->getSpaceInformation()));
        planner->as<ompl::geometric::RRTConnect>()->setRange(.1);
        setup->setPlanner(planner);
    }
    ompl::geometric::PathGeometric planPath(std::vector<double> s,std::vector<double>g,std::vector<double> other,double timeout){
        validity->setOther(other);
        return planPath(s,g,timeout);
    }
    ompl::geometric::PathGeometric planPath(std::vector<double> s,std::vector<double>g,double timeout){
        //execute the solve
        auto start=getState(s,setup);
        auto goal=getState(g,setup);
        setup->setStartAndGoalStates(start,goal);
        setup->solve(timeout);
        setup->simplifySolution();
        if(setup->haveSolutionPath()){
            std::cout<<"Solution cost "<<setup->getSolutionPath().length()<<std::endl;
            return setup->getSolutionPath();
        }
        return ompl::geometric::PathGeometric(setup->getSpaceInformation());
    }
    std::vector<std::vector<double> > planPathPy(std::vector<double> s,std::vector<double>g,std::vector<double> other,double timeout){
        //execute the solve
        ompl::geometric::PathGeometric path=planPath(s,g,other,timeout);
        std::vector<std::vector<double> > res;
        for(int s=0;s<path.getStateCount();s++){
            std::vector<double> q(7);
            for(int i=0;i<7;i++)q[i] = (*(path.getState(s)->as<ompl::base::RealVectorStateSpace::StateType>()))[i];
            res.push_back(q);
        }
        return res;
    }
private:
    ompl::geometric::SimpleSetupPtr setup;
    std::shared_ptr<SingleArmValidity> validity;
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> getState(std::vector<double> q,
                                    ompl::geometric::SimpleSetupPtr setup){
        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state(setup->getSpaceInformation());
        for(int i=0;i<7;i++){
            state[i]=q[i];
        }
        return state;
    }
};
}