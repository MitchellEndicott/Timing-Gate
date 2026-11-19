CXX ?= g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2 -I../include

test: test_gate_core
	./test_gate_core

test_gate_core: test_gate_core.cpp ../include/gate_core.h
	$(CXX) $(CXXFLAGS) -o $@ test_gate_core.cpp

clean:
	rm -f test_gate_core
