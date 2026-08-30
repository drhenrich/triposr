CXX ?= g++
CXXFLAGS ?= -std=c++17 -O1 -Wall -Wextra -Wshadow -g
FIXTURE ?= ../../../tests/wire_fixture.txt

.PHONY: test clean

test: run_tests
	./run_tests $(FIXTURE)

run_tests: test_main.cpp ../../src/dense_capsule.h ../../src/stream_proto.h ../../src/yaw_model.h
	$(CXX) $(CXXFLAGS) -o $@ test_main.cpp

clean:
	rm -f run_tests
