#pragma once
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <string>
#include <urdf/model.h>
#include <memory>
#include <vector>
#include <fcl/BVH/BVH_model.h>
#include <fcl/collision.h>
#include <fcl/distance.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/broadphase_bruteforce.h>
#include "stl_reader.h"
#include <unordered_map>
#include <functional>
#include <limits>
#include <pthread.h>
namespace YuMiPlanning{

typedef fcl::BVHModel<fcl::OBBRSS> Model;
typedef std::shared_ptr<fcl::CollisionObject> ColObjPtr;
typedef urdf::LinkSharedPtr LinkPtr;

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
                    auto mesh = (urdf::Mesh*)(l->collision->geometry.get());
                    int i=mesh->filename.find("meshes");
                    std::string stlfilename = desc_path + mesh->filename.substr(i);
                    ColObjPtr obj = loadMesh(stlfilename);
                    objects[frame_name] = obj;
                }
            }
        }
        //initialize the table object
        std::shared_ptr<fcl::Box> table=std::make_shared<fcl::Box>(2,2,table_h);
        tableObj = std::make_shared<fcl::CollisionObject>(table);
        //initialize the camera object
        double wx=.2;
        double wy=.4;
        double wz=.3;
        std::shared_ptr<fcl::Box> cam=std::make_shared<fcl::Box>(wx,wy,wz);
        fcl::Vec3f camT(0.318598, -0.087999, 1+wz/2);
        camObj = std::make_shared<fcl::CollisionObject>(cam);
        camObj->setTranslation(camT);
        //initialize the left bar object
        std::shared_ptr<fcl::Box> pillar_box=std::make_shared<fcl::Box>(.04,.04,3);
        leftPillarObj = std::make_shared<fcl::CollisionObject>(pillar_box);
        leftPillarObj->setTranslation(fcl::Vec3f(-.03,-.45,0));
        //initialize the right bar object
        rightPillarObj = std::make_shared<fcl::CollisionObject>(pillar_box);
        rightPillarObj->setTranslation(fcl::Vec3f(-.03,.45,0));
        //initialize joints from urdf for bounds
        l_joints=getJoints(L_TIP);
        r_joints=getJoints(R_TIP);
        //initialize the broadphase collision managers
        l_manager=std::make_shared<fcl::DynamicAABBTreeCollisionManager>();
        r_manager=std::make_shared<fcl::DynamicAABBTreeCollisionManager>();
        setupManager(l_manager,L_TIP);
        setupManager(r_manager,R_TIP);
    }
    void getLeftJointLims(double *lower,double *upper){
        for(int i=0;i<7;i++){
            lower[i]=l_joints[i]->limits->lower;
            upper[i]=l_joints[i]->limits->upper;
        }
    }
    void getRightJointLims(double *lower,double *upper){
        for(int i=0;i<7;i++){
            lower[i]=r_joints[i]->limits->lower;
            upper[i]=r_joints[i]->limits->upper;
        }
    }
    /*
        Returns true if the arm is colliding with itself 
    */
    bool isColliding(std::vector<double> l_joints, std::vector<double> r_joints,double table_z=.02){
        //IMPORTANT: the intercollision must happen after both isColliding, because that sets the transforms for all the objects
        //!!!!!!!!!!!!!!!!!!!!!!!!
        bool intraarm = isColliding(l_joints,L_TIP) || isColliding(r_joints,R_TIP); 
        if(intraarm)return true;
        l_manager->update();
        r_manager->update();
        return environCollision(table_z) || interCollision();
    }
    bool isInBounds(std::vector<double> l_joints, std::vector<double> r_joints){
        //mostly only from python side to sample configs
        std::vector<double> l(7);
        std::vector<double> u(7);
        getLeftJointLims(l.data(),u.data());
        for(int i=0;i<7;i++){
            if(l_joints[i]<l[i] || l_joints[i]>u[i]){
                return false;
            }
        }
        getRightJointLims(l.data(),u.data());
        for(int i=0;i<7;i++){
            if(r_joints[i]<l[i] || r_joints[i]>u[i]){
                return false;
            }
        }
        return true;
    }
    bool interCollision(){
        int collision=0;
        auto col_cb = [](fcl::CollisionObject *o1,fcl::CollisionObject *o2, void* dat){
            int *col=(int*)dat;
            fcl::CollisionRequest req;
            fcl::CollisionResult res;
            fcl::collide(o1,o2,req,res);
            if(res.isCollision())*col=1;
            return res.isCollision();
        };
        l_manager->collide(r_manager.get(),&collision,col_cb);
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
        for(int obj1=0;obj1<5;obj1++){
            //skip links that are past the wrist (these are impossible to collide)
            fcl::CollisionRequest req;
            fcl::CollisionResult res;
            for(int obj2=0;obj2<5;obj2++){
                if(std::abs(obj1-obj2)<=1)continue;
                //skip neighbor links
                fcl::collide(objsToCheck[obj1].get(),objsToCheck[obj2].get(),req,res);
                if(res.isCollision()){
                    return true;
                }
            }
        }
        return false;
    }
    const std::string L_TIP = "gripper_l_finger_l";//TODO ideally gripper collisions would be approximated as a larger box
    const std::string R_TIP = "gripper_r_finger_r";
    const std::string BASE = "base_link";
    double table_h=.5;
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

    bool environCollision(float table_z){
        //return true if the arms collide with the table (or camera)
        fcl::Vec3f tableT(0,0,-table_h/2 + table_z);
        tableObj->setTranslation(tableT);
        int collision=0;
        auto col_cb = [](fcl::CollisionObject *o1,fcl::CollisionObject *o2, void* dat){
            int *col=(int*)dat;
            fcl::CollisionRequest req;
            fcl::CollisionResult res;
            fcl::collide(o1,o2,req,res);
            if(res.isCollision())*col=1;
            return res.isCollision();
        };
        //table
        l_manager->collide(tableObj.get(),&collision,col_cb);
        if(collision==1)return true;
        r_manager->collide(tableObj.get(),&collision,col_cb);
        if(collision==1)return true;
        //camera
        l_manager->collide(camObj.get(),&collision,col_cb);
        if(collision==1)return true;
        r_manager->collide(camObj.get(),&collision,col_cb);
        //left pillar
        l_manager->collide(leftPillarObj.get(),&collision,col_cb);
        if(collision==1)return true;
        r_manager->collide(leftPillarObj.get(),&collision,col_cb);
        if(collision==1)return true;
        //right pillar
        l_manager->collide(rightPillarObj.get(),&collision,col_cb);
        if(collision==1)return true;
        r_manager->collide(rightPillarObj.get(),&collision,col_cb);
        if(collision==1)return true;
        return collision==1;
    }

    ColObjPtr loadMesh(std::string filename){
        std::vector<fcl::Triangle> tris;
        std::vector<fcl::Vec3f> verts;
        try {
            stl_reader::StlMesh <float, unsigned int> mesh (filename);
            verts.resize(mesh.num_vrts());
            for(int verti=0;verti<mesh.num_vrts();verti++){
                const float* c = mesh.vrt_coords(verti);
                verts[verti] = INFLATE*fcl::Vec3f(c[0],c[1],c[2]);
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
        if(solvers.find(frame_name)==solvers.end()){
            KDL::Chain chain;
            tree.getChain(BASE,frame_name,chain);  
            solvers.insert({frame_name,chain});
        }
        auto chain_iter = solvers.find(frame_name);
        KDL::Chain chain = chain_iter->second;
        unsigned int nJoints=chain.getNrOfJoints();
        KDL::ChainFkSolverPos_recursive solver(chain);
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
    ColObjPtr tableObj,camObj,leftPillarObj,rightPillarObj;
    std::shared_ptr<fcl::DynamicAABBTreeCollisionManager> l_manager,r_manager;
    std::vector<urdf::JointSharedPtr> l_joints;
    std::vector<urdf::JointSharedPtr> r_joints;
    //map from frame name to (num joints,solver)
    std::unordered_map<std::string,KDL::Chain> solvers;
    const double INFLATE=1.11;//was 1.11
    //all link meshes are slightly blown up to give the yumi controller enough buffer
};
}