CC := g++
FLAGS := -std=c++17 -O3 -fopenmp
PYFLAGS := $(FLAGS) -shared
$(eval PYINCLUDE= $(shell python3 -m pybind11 --includes))#magic to set the pybind include dirs
$(eval PYEXT=$(shell python3-config --extension-suffix))
INCLUDE := -I/usr/include/eigen3 -Iinclude 
LINK := -lompl -lorocos-kdl -lurdf -lkdl_parser -lfcl -lpthread
main: 
	$(CC) $(FLAGS) $(INCLUDE) src/test_ompl.cpp -o test.out $(LINK)
bindings:
	$(CC) $(PYFLAGS) $(INCLUDE) -fPIC $(PYINCLUDE) src/bindings.cpp -o yumiplanning_ompl$(PYEXT) $(LINK)
