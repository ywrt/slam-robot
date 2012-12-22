
CFLAGS=-g
OPT=-O2 -ffast-math -msse4.2 
CXXFLAGS = -fmessage-length=0 -Ieigen-eigen-304c88ca3aff -std=c++0x -g -Wall $(OPT) -I../Eigen
LDFLAGS = 
OBJS =  main.o

LIBS = -lceres -lopencv_highgui -lopencv_core -lopencv_features2d \
       -lopencv_flann -lopencv_imgproc -lglog -lgomp -lpthread \
       -lprotobuf -lcxsparse

TARGET = slam

all: $(TARGET)

slam: $(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

