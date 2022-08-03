# This Makefile assumes that you have GLFW libraries and headers installed on,
# which is commonly available through your distro's package manager.
# On Debian and Ubuntu, GLFW can be installed via `apt install libglfw3-dev`.

COMMON=-O2 -I../include -L../lib -pthread -Wl,-no-as-needed -Wl,-rpath,'$$ORIGIN'/../lib

all:
	$(CXX) $(COMMON) -std=c++17 -c simulate.cc
	$(CC)  $(COMMON) -std=c11   -c uitools.c
	$(CXX) $(COMMON) -std=c++17 main.cc simulate.o uitools.o -lmujoco -lglfw -o ../bin/simulate
	rm uitools.o simulate.o
