#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "collision_checker.h"
#include "dual_arm.h"
#include "single_arm.h"
namespace py=pybind11;

PYBIND11_MODULE(yumiplanning_ompl,m){
    py::class_<YuMiPlanning::CollisionChecker>(m,"CollisionChecker")
        .def(py::init<const std::string &>(),"Provide the urdf path")
	.def("isColliding",py::overload_cast<std::vector<double>,std::vector<double>>(&YuMiPlanning::CollisionChecker::isColliding),"Test for collision")
    .def("isInBounds",&YuMiPlanning::CollisionChecker::isInBounds,"check that a state is in bounds of urdf joint lims");
    py::class_<YuMiPlanning::DualArmPlanner>(m,"DualArmPlanner")
        .def(py::init<YuMiPlanning::CollisionChecker>(),"Provide a constructed CollisionChecker")
        .def("planPath",&YuMiPlanning::DualArmPlanner::planPathPy,"Plan a path");
    py::class_<YuMiPlanning::SingleArmPlanner>(m,"SingleArmPlanner")
        .def(py::init<YuMiPlanning::CollisionChecker, bool,bool>(),"Provide a constructed CollisionChecker")
        .def("planPath",&YuMiPlanning::SingleArmPlanner::planPathPy,"Plan a path");
}
