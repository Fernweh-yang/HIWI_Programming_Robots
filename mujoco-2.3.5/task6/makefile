COMMON=-O2 -I../include -L../lib -std=c++17 -pthread -Wl,-no-as-needed -Wl,-rpath,'$$ORIGIN'/../lib

.PHONY: all
all:
	$(CXX) $(COMMON) task6.cc     -lmujoco -lglfw   -o ../bin/task6