
CFLAGS=-g
OPT=-O3 -ffast-math -msse4.2 
CXXFLAGS = -fmessage-length=0 -Ieigen-eigen-304c88ca3aff -std=c++0x -g -Wall $(OPT) -I../Eigen
LDFLAGS = 
OBJS = octave.o octaveset.o grid.o slam.o

LIBS = -lceres -lopencv_highgui -lopencv_core -lopencv_features2d \
       -lopencv_flann -lopencv_imgproc -lglog -lgomp -lpthread \
       -lprotobuf -lcxsparse  -lblas -llapack -lcholmod

TARGET = slam
DEPS = make.deps

all: $(DEPS) $(TARGET)

test: $(DEPS) octave_test octaveset_test region_test grid_test

$(TARGET): $(OBJS) main.o
	$(CXX) -o $@ $^ $(LIBS)


%_test: %_test.o $(OBJS)
	$(CXX) -o $@ $^ $(LIBS) -lgtest
	./$@


$(DEPS): $(wildcard *.cpp) Makefile
	$(CXX) $(CXXFLAGS) -MM $(wildcard *.cpp) > $(DEPS)

-include $(DEPS)
