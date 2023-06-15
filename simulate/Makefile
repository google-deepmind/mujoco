# This Makefile assumes that you have GLFW libraries and headers installed on,
# which is commonly available through your distro's package manager.
# On Debian and Ubuntu, GLFW can be installed via `apt install libglfw3-dev`.

COMMON=-O2 -I../include -L../lib -pthread -Wl,-no-as-needed -Wl,-rpath,'$$ORIGIN'/../lib

all:
	$(CXX) $(COMMON) -std=c++17 -c simulate.cc
	$(CC)  $(COMMON) -std=c++17 -c platform_ui_adapter.cc
	$(CC)  $(COMMON) -std=c++17 -c glfw_adapter.cc
	$(CC)  $(COMMON) -std=c++17 -c glfw_dispatch.cc
	$(CXX) $(COMMON) -std=c++17 main.cc simulate.o platform_ui_adapter.o glfw_adapter.o glfw_dispatch.o -lmujoco -lglfw -o ../bin/simulate2
	rm platform_ui_adapter.o simulate.o glfw_adapter.o glfw_dispatch.o