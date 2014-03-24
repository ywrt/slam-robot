
CC=g++
CFLAGS=-g
#OPT=-O3 -ffast-math -msse4.2 
CXXFLAGS = -fmessage-length=0 \
              -std=c++0x -g -Wall $(OPT)
LDFLAGS= -Lceres-solver-1.8.0/BUILD/lib
CPPFLAGS= -Iceres-solver-1.8.0/include -I./eigen-eigen-ffa86ffb5570
OBJS = octave.o octaveset.o grid.o slam.o localmap.o planner.o \
    histogram.o \
    imgtypes.o matcher.o

#faster.o faster1.o \

TESTS = octave octaveset region grid histogram descriptor corners

LDLIBS = -lceres -lopencv_calib3d -lopencv_highgui -lopencv_core -lopencv_features2d \
       -lopencv_flann -lopencv_imgproc -lglog -lgomp -lpthread \
       -lprotobuf -lblas -llapack -lcholmod -lm

TARGET = slam
DEPS = make.deps

all: $(DEPS) $(TARGET)

test: $(DEPS) $(TESTS:=_run)

$(TARGET): $(OBJS) main.o

%_test: %_test.o $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS) -lgtest

%_run: %_test $(OBJS)
	./$(@:%_run=%_test)

$(DEPS): $(wildcard *.cpp) Makefile
	$(CXX) $(CXXFLAGS) -MM $(wildcard *.cpp) > $(DEPS)

# Can't compile with optimization turned on. too large!
faster.o: faster.h faster.c
	gcc -std=c99 -O -c faster.c

faster1.o: faster.h faster1.c
	gcc -std=c99 -O -c faster1.c

-include $(DEPS)
