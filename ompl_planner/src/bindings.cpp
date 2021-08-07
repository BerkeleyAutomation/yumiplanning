#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "collision_checker.h"
#include "dual_arm.h"
#include "single_arm.h"
namespace py=pybind11;
PYBIND11_MODULE(yumiplanning_ompl,m){
    py::class_<YuMiPlanning::CollisionChecker>(m,"CollisionChecker")
        .def(py::init<const std::string &>(),"Provide the urdf path");
    py::class_<YuMiPlanning::DualArmPlanner>(m,"DualArmPlanner")
        .def(py::init<YuMiPlanning::CollisionChecker>(),"Provide a constructed CollisionChecker")
        .def("planPath",&YuMiPlanning::DualArmPlanner::planPath,"Test for planning");
}