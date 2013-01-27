CC=g++ --std=c++11
CPP_FILES := $(wildcard src/*.cpp)
OBJ_FILES := $(addprefix obj/,$(notdir $(CPP_FILES:.cpp=.o)))
CFLAGS =-ggdb

INC = -I/usr/local/include/opencv -I/

LIBS = -lopencv_core\
       -lopencv_imgproc\
       -lopencv_calib3d\
       -lopencv_video\
       -lopencv_features2d\
       -lopencv_ml\
       -lopencv_highgui\
       -lopencv_objdetect\
       -lopencv_contrib\
       -lopencv_legacy\
       -lopencv_stitching

HEADERS = src/DataTypes.h\
          src/GPSStitcher.h\
          src/AdjacentFeaturesMatcher.h

stitch: $(OBJ_FILES) $(HEADERS)
	$(CC) $(CFLAGS) $(LDFLAGS) $(INC) $(LIBS) $(OBJECTS) -o $@ $(OBJ_FILES)

obj/%.o: src/%.cpp
	$(CC) $(CFLAGS) $(INC) -c -o $@ $<

clean: 
	rm -f *.o stitch

