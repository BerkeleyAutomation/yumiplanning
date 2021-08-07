#include <iostream>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <string>
#include <urdf/model.h>
#include <memory>
#include <vector>
#include <fcl/BVH/BVH_model.h>
#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/broadphase_bruteforce.h>
#include "stl_reader.h"
#include <unordered_map>
#include <functional>

namespace YuMiPlanning{

typedef fcl::BVHModel<fcl::OBBRSS> Model;
typedef std::shared_ptr<fcl::CollisionObject> ColObjPtr;
typedef std::shared_ptr<urdf::Link> LinkPtr;

class CollisionChecker{
public:
    CollisionChecker(std::string desc_path){
        robot_model.initFile(desc_path+"/urdf/yumi.urdf");\
        if (!kdl_parser::treeFromUrdfModel(robot_model, tree)){
            std::cerr << "Failed to extract kdl tree from xml robot description\n";
        }
        std::vector<LinkPtr> links;
        robot_model.getLinks(links);
        for(auto l:links){
            if(l->collision!=nullptr){
                //need to load in the mesh for this
                std::string frame_name = l->name;
                if(l->collision->geometry->type==urdf::Geometry::MESH){
                    auto mesh = std::static_pointer_cast<urdf::Mesh>(l->collision->geometry);
                    int i=mesh->filename.find("meshes");
                    std::string stlfilename = desc_path + mesh->filename.substr(i);
                    ColObjPtr obj = loadMesh(stlfilename);
                    std::string *userdat = new std::string(frame_name);
                    obj->setUserData((void*)userdat);//TODO free this data
                    objects[frame_name] = obj;
                }
            }
        }
        //initialize the table object
        double h=.5;
        std::shared_ptr<fcl::Box> table=std::make_shared<fcl::Box>(2,2,.5);
        fcl::Vec3f T(0,0,-h/2);
        tableObj = std::make_shared<fcl::CollisionObject>(table);
        tableObj->setTranslation(T);
        //initialize joints from urdf for bounds
        l_joints=getJoints(L_TIP);
        r_joints=getJoints(R_TIP);
        //initialize the broadphase collision managers
        l_manager=std::make_shared<fcl::DynamicAABBTreeCollisionManager>();
        r_manager=std::make_shared<fcl::DynamicAABBTreeCollisionManager>();
        setupManager(l_manager,L_TIP);
        setupManager(r_manager,R_TIP);
    }
    void getLeftJointLims(std::vector<double> &lower,std::vector<double> &upper){
        lower.clear();
        upper.clear();
        for(auto j:l_joints){
            lower.push_back(j->limits->lower);
            upper.push_back(j->limits->upper);
        }
    }
    void getRightJointLims(std::vector<double> &lower,std::vector<double> &upper){
        lower.clear();
        upper.clear();
        for(auto j:r_joints){
            lower.push_back(j->limits->lower);
            upper.push_back(j->limits->upper);
        }
    }
    /*
        Returns true if the arm is colliding with itself 
        //TODO add inter-arm collisions
    */
    bool isColliding(std::vector<double> l_joints, std::vector<double> r_joints){
        //IMPORTANT: the intercollision must happen after both isColliding, because that sets the transforms for all the objects
        //!!!!!!!!!!!!!!!!!!!!!!!!
        return isColliding(l_joints,L_TIP) || isColliding(r_joints,R_TIP) || interCollision();
    }
    bool interCollision(){
        l_manager->update();
        r_manager->update();
        int collision=0;
        auto blank_cb = [](fcl::CollisionObject *o1,fcl::CollisionObject *o2, void* dat){
            int *col=(int*)dat;
            // std::string *name1=(std::string*)o1->getUserData();
            // std::string *name2=(std::string*)o2->getUserData();
            // fcl::Vec3f t1=o1->getTranslation();
            // fcl::Vec3f t2=o2->getTranslation();
            fcl::CollisionRequest req;
            fcl::CollisionResult res;
            fcl::collide(o1,o2,req,res);
            if(res.isCollision())*col=1;
            return res.isCollision();
        };
        l_manager->collide(r_manager.get(),&collision,blank_cb);
        return collision==1;
    }
    /*
        returns true if the arm from base link to tip frame is colliding with itself
    */
    bool isColliding(std::vector<double>joints, std::string tip_frame) {
        KDL::Chain chain;
        tree.getChain(BASE,tip_frame,chain);
        std::vector<ColObjPtr> objsToCheck;//in order from base to elbow
        objsToCheck.push_back(objects[BASE]);
        for(int i=0;i<chain.getNrOfSegments();i++){
            const std::string name = chain.getSegment(i).getName();
            auto obj=objects.find(name);
            if(obj != objects.end()){
                ColObjPtr& o = (*obj).second;
                KDL::Frame H = getFK(name,joints);//TODO optimize this to not be a separate function?
                fcl::Matrix3f rot(H(0,0),H(0,1),H(0,2),
                                  H(1,0),H(1,1),H(1,2),
                                  H(2,0),H(2,1),H(2,2));
                fcl::Vec3f trans(H(0,3),H(1,3),H(2,3));
                o->setTransform(rot,trans);
                objsToCheck.push_back(o);
            }
        }
        for(int obj1=0;obj1<objsToCheck.size();obj1++){
            for(int obj2=0;obj2<objsToCheck.size();obj2++){
                if(obj1-obj2<=1 && obj1-obj2>=-1 || (obj1>=5 && obj2>=5))continue;//skip neighbor links
                fcl::CollisionRequest req;
                fcl::CollisionResult res;
                fcl::collide(objsToCheck[obj1].get(),objsToCheck[obj2].get(),req,res);
                if(res.isCollision()){
                    return true;
                }
            }   
        }
        //check last object collision with table
        fcl::CollisionRequest req;
        fcl::CollisionResult res;
        fcl::collide(objsToCheck[objsToCheck.size()-1].get(),tableObj.get(),req,res);
        return res.isCollision();
    }
    const std::string L_TIP = "gripper_l_base";
    const std::string R_TIP = "gripper_r_base";
    const std::string BASE = "base_link";
private:
    void setupManager(std::shared_ptr<fcl::BroadPhaseCollisionManager> man, std::string tip_frame){
        KDL::Chain chain;
        tree.getChain(BASE,tip_frame,chain);
        for(int i=0;i<chain.getNrOfSegments();i++){
            const std::string name = chain.getSegment(i).getName();
            auto obj=objects.find(name);
            if(obj != objects.end()){
                man->registerObject((*obj).second.get());
            }
        }
        man->setup();
    }
    std::vector<urdf::JointSharedPtr> getJoints(std::string tip_frame){
        urdf::LinkConstSharedPtr tmp = robot_model.getLink(tip_frame);
        std::vector<urdf::JointSharedPtr> js(7);
        int i=6;
        while(tmp->name!=BASE){
            urdf::JointSharedPtr j=tmp->parent_joint;
            if(j->type==urdf::Joint::REVOLUTE){
                js[i--]=j;
            }
            tmp=robot_model.getLink(tmp->parent_joint->parent_link_name);
        }
        return js;
    }
    ColObjPtr loadMesh(std::string filename){
        std::vector<fcl::Triangle> tris;
        std::vector<fcl::Vec3f> verts;
        try {
            stl_reader::StlMesh <float, unsigned int> mesh (filename);
            verts.resize(mesh.num_vrts());
            for(int verti=0;verti<mesh.num_vrts();verti++){
                const float* c = mesh.vrt_coords(verti);
                verts[verti] = fcl::Vec3f(c[0],c[1],c[2]);
            }
            for(size_t itri = 0; itri < mesh.num_tris(); ++itri) {
                tris.emplace_back(mesh.tri_corner_ind(itri,0),mesh.tri_corner_ind(itri,1),mesh.tri_corner_ind(itri,2));
            }
        }catch (std::exception& e) {
            std::cout << e.what() << std::endl;
        }
        std::shared_ptr<Model> geom=std::make_shared<Model>();
        geom->beginModel();
        geom->addSubModel(verts,tris);
        geom->endModel();
        std::shared_ptr<fcl::CollisionObject> obj = std::make_shared<fcl::CollisionObject>(geom);
        return obj;
    }
    KDL::Frame getFK(std::string frame_name,std::vector<double> jointvec){
        /*
        jointvec is a 7-vector. this function downfilters it into the correct number and computes forward kin
        */
        KDL::Chain chain;
        if(!tree.getChain("base_link",frame_name,chain)){
            std::cerr<<"couldn't find link in tree\n";
        }
        KDL::ChainFkSolverPos_recursive solver(chain);
        unsigned int nJoints = chain.getNrOfJoints();
        KDL::JntArray kdlJ(nJoints);
        for(int i=0;i<nJoints;i++){
            kdlJ(i) = jointvec[i];
        }
        KDL::Frame result;
        solver.JntToCart(kdlJ,result);
        return result;
    }
    KDL::Tree tree;
    urdf::Model robot_model;
    std::unordered_map<std::string,ColObjPtr> objects;//dict that keeps track of all the loaded meshes
    ColObjPtr tableObj;
    std::shared_ptr<fcl::DynamicAABBTreeCollisionManager> l_manager,r_manager;
    std::vector<urdf::JointSharedPtr> l_joints;
    std::vector<urdf::JointSharedPtr> r_joints;
};
class SingleArmValidity {
public:
    /*
        left specifies whether this plans for left or right arm, use_other specifies whether to plan
        around the other arm (false means ignore collisions with the other arm)
    */
    SingleArmValidity(ompl::base::SpaceInformationPtr si, CollisionChecker checker, bool left, bool use_other):
                colChecker(checker),left(left),use_other(use_other){

    }
    void setOther(std::vector<double> &new_other){
        other_q = new_other;
    }
    bool isValid(const ompl::base::State *state){
        auto q = state->as<ompl::base::RealVectorStateSpace::StateType>();
        std::vector<double> cur_q(7);
        for(int i=0;i<7;i++)cur_q[i] = (*q)[i];
        if(use_other){
            if(left){
                return !colChecker.isColliding(cur_q,other_q);
            }else{
                return !colChecker.isColliding(other_q,cur_q);
            }
        }else{
            if(left){
                return !colChecker.isColliding(cur_q,colChecker.L_TIP);
            }else{
                return !colChecker.isColliding(cur_q,colChecker.R_TIP);
            }
        }
    }
private:
    CollisionChecker colChecker;
    bool left,use_other;
    std::vector<double> other_q;
};
class SingleArmPlanner{
public:
    SingleArmPlanner(CollisionChecker checker, bool left, bool use_other){
        //initialize the state space
        ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(7));
        ompl::base::RealVectorBounds bounds(7);
        checker.getLeftJointLims(bounds.low,bounds.high);
        space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
        //use simplesetup to make a planner
        setup = ompl::geometric::SimpleSetupPtr(new ompl::geometric::SimpleSetup(space));
        validity=std::make_shared<SingleArmValidity>(setup->getSpaceInformation(),checker,left,use_other);
        std::function<bool(const ompl::base::State*)> fun=std::bind<bool>(&SingleArmValidity::isValid,validity,std::placeholders::_1);
        setup->setStateValidityChecker(fun);
        setup->getSpaceInformation()->setStateValidityCheckingResolution(0.001);
        ompl::base::PlannerPtr planner(new ompl::geometric::RRTstar(setup->getSpaceInformation()));
        setup->setPlanner(planner);
    }
    ompl::geometric::PathGeometric planPath(std::vector<double> s,std::vector<double>g,std::vector<double> other){
        validity->setOther(other);
        return planPath(s,g);
    }
    ompl::geometric::PathGeometric planPath(std::vector<double> s,std::vector<double>g){
        //execute the solve
        auto start=getState(s,setup);
        auto goal=getState(g,setup);
        setup->setStartAndGoalStates(start,goal);
        setup->solve(1);
        // setup->simplifySolution();
        if(setup->haveSolutionPath()){
            ompl::geometric::PathGeometric path = setup->getSolutionPath();
            return path;
        }
        return ompl::geometric::PathGeometric(setup->getSpaceInformation());
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

}//end namespace
int main(){
    YuMiPlanning::CollisionChecker checker("/home/jkerr/yumi/yumiplanning/yumi_description/");
    YuMiPlanning::SingleArmPlanner planner(checker,true,true);
    std::vector<double> s={1.7886884440180186, -0.44371156619454205, -1.7481374222057635, -0.40361105137018755, 0.16508589386459488, 1.1032037623419368, -0.1919595195294482};
    std::vector<double> g ={1.7889450395890478, 0.5727117937361313, -0.1202846167553847, 1.105436756336941, -0.8347407460212457, -0.9100656608720741, -0.17140514236767576};
    std::vector<double> other_q(7,0);
    ompl::geometric::PathGeometric path = planner.planPath(s,g,other_q);
    if(!path.check()){
        std::cerr<<"Failed to plan path\n";
        return 1;
    }
    path.print(std::cout);
    return 0;
}