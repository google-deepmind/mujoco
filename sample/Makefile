# This Makefile assumes that you have GLFW libraries and headers installed on,
# which is commonly available through your distro's package manager.
# On Debian and Ubuntu, GLFW can be installed via `apt install libglfw3-dev`.

COMMON=-O2 -I../include -L../lib -std=c++11 -pthread -Wl,-rpath,'$$ORIGIN'/../lib

all:
	$(CXX) $(COMMON) testxml.cc    -lmujoco_nogl               -o ../bin/testxml
	$(CXX) $(COMMON) testspeed.cc  -lmujoco_nogl               -o ../bin/testspeed
	$(CXX) $(COMMON) compile.cc    -lmujoco_nogl               -o ../bin/compile
	$(CXX) $(COMMON) derivative.cc -lmujoco_nogl -fopenmp      -o ../bin/derivative
	$(CXX) $(COMMON) basic.cc      -lmujoco -lGL -lglew -lglfw -o ../bin/basic
	$(CXX) $(COMMON) record.cc     -lmujoco -lGL -lglew -lglfw -o ../bin/record
	$(CC) -c -O2 -I../include uitools.c
	$(CXX) $(COMMON) uitools.o simulate.cc -lmujoco -lGL -lglew -lglfw -o ../bin/simulate
	rm *.o
