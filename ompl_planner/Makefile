CC := g++
FLAGS := -std=c++11
INCLUDE := -I/usr/include/eigen3 -Iinclude
LINK := -lompl -lorocos-kdl -lurdf -lkdl_parser -lfcl
main:
	$(CC) $(FLAGS) $(INCLUDE) test_ompl.cpp -o test.out $(LINK)
