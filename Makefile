
CFLAGS=-g
OPT=-O3 -ffast-math -msse4.2 
CXXFLAGS = -fmessage-length=0 -Ieigen-eigen-304c88ca3aff -std=c++0x -g -Wall $(OPT) -I../Eigen
LDFLAGS = 
OBJS = octave.o octaveset.o

LIBS = -lceres -lopencv_highgui -lopencv_core -lopencv_features2d \
       -lopencv_flann -lopencv_imgproc -lglog -lgomp -lpthread \
       -lprotobuf -lcxsparse  -lblas -llapack -lcholmod

TARGET = slam

all: $(TARGET)

test: octave_test octaveset_test region_test

$(TARGET): $(OBJS) main.o
	$(CXX) -o $@ $^ $(LIBS)


%_test: %_test.o $(OBJS)
	$(CXX) -o $@ $^ $(LIBS) -lgtest
	./$@

octave.o: octave.cpp util.h octave.h imgtypes.h region.h
octaveset.o: octaveset.cpp octave.h imgtypes.h region.h util.h \
 octaveset.h
octaveset_test.o: octaveset_test.cpp octaveset.h imgtypes.h region.h
octave_test.o: octave_test.cpp octave.h imgtypes.h region.h
region_test.o: region_test.cpp region.h imgtypes.h
