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
#include "stl_reader.h"
#include <unordered_map>
namespace YuMiPlanning{

typedef fcl::BVHModel<fcl::OBBRSS> Model;
typedef std::shared_ptr<fcl::CollisionObject> ColObjPtr;
typedef std::shared_ptr<const urdf::Link> LinkPtr;
class CollisionChecker{
public:
    CollisionChecker(urdf::Model robot_model):robot_model(robot_model){
        if (!kdl_parser::treeFromUrdfModel(robot_model, tree)){
            std::cerr << "Failed to extract kdl tree from xml robot description\n";
        }
        //1. parse through and load the mesh files into fcl
    }
    /*
        Returns true if the arm is colliding with itself (for now doesn't include inter-arm collisions)
    */
    bool isColliding(std::vector<double> l_joints,std::vector<double> r_joints){
        
        return isColliding(l_joints,L_TIP) || isColliding(r_joints,R_TIP);
    }
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
        return false;
    }
    const std::string L_TIP = "gripper_l_base";
    const std::string R_TIP = "gripper_r_base";
    const std::string BASE = "base_link";
private:
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
};
}
int main(){
    urdf::Model robot_model;
    robot_model.initFile("/home/jkerr/yumi/yumiplanning/yumi_description/urdf/yumi.urdf");\
    YuMiPlanning::CollisionChecker checker(robot_model);
    std::vector<double> zero(7,0.0);
    checker.isColliding(zero,checker.L_TIP);
    return 0;
}