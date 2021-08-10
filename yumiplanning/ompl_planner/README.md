To use this planner, cd into this folder and run `make bindings`. Ignore all warnings. After that you should be able to 
call `from yumiplanning.yumi_planner import Planner` without issues and use the planner!

This module only depends on some packages to install with apt-get: `libompl-dev`, `libeigen3-dev`, `liburdf-dev`, `libkdl-parser-dev`, `liborocos-kdl-dev`, `libfcl-dev`. Also make sure that pybind is installed `pip3 install pybind11`.