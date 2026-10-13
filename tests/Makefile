# Host-side unit tests (Linux/macOS/WSL, or MinGW on Windows). Needs g++.
SRC = ../Drone/src
CXXFLAGS = -std=c++17 -Wall -Wextra -O1 -Istub -I$(SRC)

test: run_tests
	./run_tests

run_tests: test_main.cpp $(SRC)/crtp.h $(SRC)/apparm.h $(SRC)/rcmath.h $(SRC)/mixer.cpp $(SRC)/pid.cpp $(SRC)/ahrs.cpp
	$(CXX) $(CXXFLAGS) -o $@ $(filter %.cpp,$^)

verbose: run_tests
	./run_tests -v

clean:
	rm -f run_tests run_tests.exe

.PHONY: test verbose clean
