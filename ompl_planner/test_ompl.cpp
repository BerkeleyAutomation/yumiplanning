#include <iostream>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <string>
#include <urdf/model.h>
#include <memory>
#include <vector>
#include <fcl/BVH/BVH_model.h>
#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include "stl_reader.h"
#include <unordered_map>
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
        l_joints=getJoints(L_TIP);
        r_joints=getJoints(R_TIP);
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
    bool isColliding(std::vector<double> l_joints,std::vector<double> r_joints){
        return isColliding(l_joints,L_TIP) || isColliding(r_joints,R_TIP);
    }
    /*
        returns true if the arm from base link to tip frame is colliding with itself
        //TODO add table collisions
    */
    bool isColliding(std::vector<double>joints, std::string tip_frame){
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
                if(obj1-obj2<=1 && obj1-obj2>=-1)continue;//skip neighbor links
                fcl::CollisionRequest req;
                fcl::CollisionResult res;
                fcl::collide(objsToCheck[obj1].get(),objsToCheck[obj2].get(),req,res);
                if(res.isCollision())return true;
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
    std::vector<urdf::JointSharedPtr> l_joints;
    std::vector<urdf::JointSharedPtr> r_joints;
};
}//end namespace
int main(){
    YuMiPlanning::CollisionChecker checker("/home/jkerr/yumi/yumiplanning/yumi_description/");
    std::vector<double> q={-1.2407575639621562, -1.4772790256426007, 0.9846829738212685, -0.5476077054575812, 2.1117175005455615, 1.2273630621015483, 3.2424116766608138};
    bool collides = checker.isColliding(q,checker.L_TIP);
    std::cout<<collides<<std::endl;
    std::vector<double> l;
    std::vector<double> u;
    checker.getLeftJointLims(l,u);
    for(int i=0;i<7;i++){
        std::cout<<"joint "<<i<<" limits: "<<l[i]<<" -> "<<u[i]<<std::endl;
    }
    checker.getRightJointLims(l,u);
    for(int i=0;i<7;i++){
        std::cout<<"joint "<<i<<" limits: "<<l[i]<<" -> "<<u[i]<<std::endl;
    }
    return 0;
}