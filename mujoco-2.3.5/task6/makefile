COMMON=-O2 -I../include -I. -L../lib -std=c++17 -pthread -Wl,-no-as-needed -Wl,-rpath,'$$ORIGIN'/../lib
.PHONY: all
all:
	$(CXX) $(COMMON) task6.cpp     -lmujoco -lglfw   -o ../bin/task6