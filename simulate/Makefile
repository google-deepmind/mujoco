# This Makefile assumes that you have GLFW libraries and headers installed on,
# which is commonly available through your distro's package manager.
# On Debian and Ubuntu, GLFW can be installed via `apt install libglfw3-dev`.

COMMON=-O2 -I../include -L../lib -std=c++11 -pthread -Wl,-rpath,'$$ORIGIN'/../lib

all:
	$(CC)  -c -O2 -fPIC -I../include uitools.c
	$(CXX) -c -O2 -fPIC -I../include simulate.cc
	$(CXX) -shared -o libmjsimulate.so simulate.o uitools.o
	mv libmjsimulate.so ../lib/libmjsimulate.so
	$(CXX) $(COMMON) -Wl,-no-as-needed main.cc -lmjsimulate -lmujoco -lGL -lglfw -o ../bin/simulate
