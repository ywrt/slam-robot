
SANITIZE= #-fsanitize=address -fno-omit-frame-pointer 
CC=g++
CFLAGS=-g
OPT=-O3 -ffast-math -msse4.2 
CXXFLAGS = -fmessage-length=72 \
              -std=c++0x -g -Wall $(OPT) $(SANITIZE)
LDFLAGS= -Lceres-solver-1.8.0/BUILD/lib $(SANITIZE)
CPPFLAGS= -Iceres-solver-1.8.0/include -I./eigen-eigen-ffa86ffb5570
OBJS = slam.o \
       localmap.o \
       planner.o \
       histogram.o \
       matcher.o

TESTS = region grid histogram descriptor

LDLIBS = -lceres \
	 -lopencv_highgui \
	 -lopencv_core \
       -lopencv_imgproc \
       -lpthread \
       -lgomp \
       -lblas \
       -llapack \
       -lcholmod \
       -lm \
       -lglog \
       -lprotobuf \
       -lgflags \

TARGET = slam
DEPS = make.deps

all: $(DEPS) $(TARGET)

test: $(DEPS) $(TESTS:=_run)

$(TARGET): $(OBJS) main.o

%_test: %_test.o $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS) -lgtest

%_run: %_test $(OBJS)
	./$(@:%_run=%_test)

$(DEPS): $(wildcard *.cpp) $(wildcard *.h) Makefile
	$(CXX) $(CXXFLAGS) -MM $(wildcard *.cpp) > $(DEPS)

-include $(DEPS)
