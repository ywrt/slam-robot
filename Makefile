
CC=g++
CFLAGS=-g
OPT=-O3 -ffast-math -msse4.2 
CXXFLAGS = -fmessage-length=0 \
              -std=c++0x -g -Wall $(OPT)
LDFLAGS= -Lceres-solver-1.8.0/BUILD/lib
CPPFLAGS= -Iceres-solver-1.8.0/include -I./eigen-eigen-ffa86ffb5570
OBJS = octave.o octaveset.o grid.o slam.o localmap.o planner.o \
    tracking.o histogram.o descriptor.o corners.o faster.o faster1.o

LDLIBS = -lceres -lopencv_highgui -lopencv_core -lopencv_features2d \
       -lopencv_flann -lopencv_imgproc -lglog -lgomp -lpthread \
       -lprotobuf -lblas -llapack -lcholmod -lm

TARGET = slam
DEPS = make.deps

all: $(DEPS) $(TARGET)

test: $(DEPS) octave_test octaveset_test region_test grid_test histogram_test descriptor_test corners_test

$(TARGET): $(OBJS) main.o

%_test: %_test.o $(OBJS)
	$(CXX) -o $@ $^ $(LDLIBS) -lgtest
	./$@


$(DEPS): $(wildcard *.cpp) Makefile
	$(CXX) $(CXXFLAGS) -MM $(wildcard *.cpp) > $(DEPS)

# Can't compile with optimization turned on. too large!
faster.o: faster.h faster.cpp
	g++ -std=c++0x -c faster.cpp

faster1.o: faster.h faster1.cpp
	g++ -std=c++0x -c faster1.cpp

-include $(DEPS)
